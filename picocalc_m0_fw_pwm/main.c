/* SPDX-License-Identifier: GPL-2.0 */
/* M0 delta-sigma audio: single TIMER0_CH5 ISR drives both GPIO4_B2 (L) and B3 (R) */

#include "rk3506_regs.h"
#include "shmem.h"

#define REG(addr)   (*(volatile uint32_t *)(addr))

/*
 * Fixed config: fewer memory loads in the ISR and no format/sample-rate logic.
 * ALSA on the host can do sample-rate conversion to this rate.
 * Linux driver must use the same sample_rate and buf_size.
 */
#define M0_FIXED_SAMPLE_RATE_HZ  48000U
#define M0_FIXED_BUF_SIZE        8192U
#define M0_FIXED_BUF_MASK         (M0_FIXED_BUF_SIZE - 1U)

static m0_audio_shmem_t *const shmem = (m0_audio_shmem_t *)M0_SHMEM_ADDR;

static int32_t integ1_l, integ2_l, integ1_r, integ2_r;
static uint32_t phase_acc;
static uint32_t read_idx;
static int32_t last_l, last_r;  /* hold current sample when not advancing */
static uint8_t *buf_ptr;        /* shmem->buffer pointer, cached at play-start */
static uint32_t shmem_update_counter; /* batches shmem->read_idx writes */
static uint32_t out_l, out_r;   /* previous DSM output (0 or 1), for integ1 feedback */

__attribute__((always_inline)) static inline void gpio_write_both(uint32_t bit_l, uint32_t bit_r)
{
	REG(GPIO4_BASE + GPIO_DR_L) = GPIO4_DR_WRITE(bit_l, bit_r);
}

static void timer5_start(void)
{
	REG(TIMER0_CH5_BASE + TIMER_LOAD0) = DS_PERIOD_TICKS;
	REG(TIMER0_CH5_BASE + TIMER_CTRL) = TIMER_RUN;
}

static void timer5_stop(void)
{
	REG(TIMER0_CH5_BASE + TIMER_CTRL) = TIMER_STOP;
}

__attribute__((always_inline)) static inline void clear_timer5_irq(void)
{
	REG(TIMER0_CH5_BASE + TIMER_INTSTAT) = 1;
}

/* Advance read_idx by step bytes and batch-write to shared memory every 8 frames. */
__attribute__((always_inline)) static inline void advance_read_idx(uint32_t step)
{
	read_idx = (read_idx + step) & M0_FIXED_BUF_MASK;
	if (++shmem_update_counter >= 8) {
		shmem_update_counter = 0;
		shmem->read_idx = read_idx;
		__dmb();
	}
}

/* S16 LE stereo only (fixed config). Inlined into ISR to remove call overhead. */
__attribute__((always_inline)) static inline void consume_s16_stereo(int32_t *s32_l, int32_t *s32_r)
{
	uint32_t i = read_idx & M0_FIXED_BUF_MASK;
	*s32_l = (int16_t)(buf_ptr[i] | (buf_ptr[(i + 1) & M0_FIXED_BUF_MASK] << 8));
	i = (read_idx + 2) & M0_FIXED_BUF_MASK;
	*s32_r = (int16_t)(buf_ptr[i] | (buf_ptr[(i + 1) & M0_FIXED_BUF_MASK] << 8));
	advance_read_idx(4);
}

/*
 * ISR is named TIMER0_CH5_IRQHandler so the vector table jumps here directly
 * (no asm wrapper: no push/bl timer5_isr/pop), saving ~8 cycles. Load order
 * (rate before phase) helps reduce load-use stalls. The compiler may still
 * use callee-saved regs (r4–r7); a hand-crafted naked asm ISR could save
 * the remaining push/pop if needed for a tighter cycle budget.
 */
__attribute__((section(".ramfunc")))
void TIMER0_CH5_IRQHandler(void)
{
	REG(TIMER0_CH5_BASE + TIMER_INTSTAT) = 1;

	{
		uint32_t phase = phase_acc + M0_FIXED_SAMPLE_RATE_HZ;
		if (phase >= DS_RATE_HZ) {
			phase -= DS_RATE_HZ;
			consume_s16_stereo(&last_l, &last_r);
		}
		phase_acc = phase;
	}

	/* 2nd-order delta-sigma, left (GPIO4_B2). Write-back each step to
	 * keep live values in r0–r3 and avoid callee-saved regs. */
	{
		uint32_t i1 = (uint32_t)((int32_t)integ1_l + last_l + (int32_t)DSM_HALF_SCALE - (int32_t)(out_l << 15));
		integ1_l = (int32_t)i1;
		uint32_t i2 = (uint32_t)((int32_t)integ2_l + (int32_t)i1);
		out_l = 1u - (i2 >> 31);
		integ2_l = (int32_t)((int32_t)i2 + (int32_t)DSM_FULL_SCALE - (int32_t)(out_l << 16));
	}

	/* Right channel (GPIO4_B3), same structure. */
	{
		uint32_t i1 = (uint32_t)((int32_t)integ1_r + last_r + (int32_t)DSM_HALF_SCALE - (int32_t)(out_r << 15));
		integ1_r = (int32_t)i1;
		uint32_t i2 = (uint32_t)((int32_t)integ2_r + (int32_t)i1);
		out_r = 1u - (i2 >> 31);
		integ2_r = (int32_t)((int32_t)i2 + (int32_t)DSM_FULL_SCALE - (int32_t)(out_r << 16));
	}

	REG(GPIO4_BASE + GPIO_DR_L) = GPIO4_DR_WRITE(out_l, out_r);
}

static void nvic_enable_timer5(void)
{
	/* NVIC_ISER0: set bit 19 to enable IRQ 19 */
	REG(0xE000E100) = (1u << TIMER0_CH5_IRQ);
}

static void nvic_disable_timer5(void)
{
	/* NVIC_ICER0: set bit 19 to disable IRQ 19 */
	REG(0xE000E180) = (1u << TIMER0_CH5_IRQ);
}

static void hardware_init(void)
{
	/* CRU: enable TIMER0_CH5 and PCLK_TIMER */
	REG(CRU_BASE + CRU_GATE_CON06) = CRU_TIMER5_EN;
	REG(CRU_BASE + CRU_CLKSEL_CON23) = CRU_TIMER5_100M;
	REG(CRU_BASE + CRU_GATE_CON13) = CRU_GPIO4_EN;

	/* GPIO4 B2/B3: digital mode then output, both low */
	REG(GPIO4_IOC_BASE + SARADC_CON) = SARADC_CON_B23_EN;
	REG(GPIO4_BASE + GPIO_DDR_L) = GPIO4_B23_OUT_DIR;
	gpio_write_both(0, 0);

	/* Timer: load period, don't start yet */
	REG(TIMER0_CH5_BASE + TIMER_LOAD0) = DS_PERIOD_TICKS;
	REG(TIMER0_CH5_BASE + TIMER_CTRL) = TIMER_STOP;
}

int main(void)
{
	hardware_init();

	for (;;) {
		while (shmem->magic != M0_AUDIO_MAGIC)
			;

		if (shmem->ctrl == M0_CTRL_PLAY) {
			buf_ptr = (uint8_t *)shmem->buffer;
			read_idx = shmem->read_idx;
			shmem_update_counter = 0;
			integ1_l = integ2_l = 0;
			integ1_r = integ2_r = 0;
			out_l = out_r = 0;
			phase_acc = 0;
			last_l = last_r = 0;
			__asm volatile("" ::: "memory"); /* compiler barrier: all statics visible before IRQ fires */
			nvic_enable_timer5();
			timer5_start();
			while (shmem->ctrl == M0_CTRL_PLAY)
				;
			timer5_stop();
			REG(TIMER0_CH5_BASE + TIMER_INTSTAT) = 1; /* clear any pending IRQ before masking */
			nvic_disable_timer5();
			gpio_write_both(0, 0);
		}
	}
}
