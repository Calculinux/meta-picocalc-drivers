/* SPDX-License-Identifier: GPL-2.0 */
/* M0 delta-sigma audio: single TIMER0_CH5 ISR drives both GPIO4_B2 (L) and B3 (R) */

#include "rk3506_regs.h"
#include "shmem.h"

#define REG(addr)   (*(volatile uint32_t *)(addr))

static m0_audio_shmem_t *const shmem = (m0_audio_shmem_t *)M0_SHMEM_ADDR;

static int32_t integ1_l, integ2_l, integ1_r, integ2_r;
static uint32_t phase_acc;
static uint32_t sample_rate_hz;
static uint32_t buf_size;
static uint32_t read_idx;
static uint32_t channels;
static uint32_t format;
static int16_t last_l, last_r;  /* hold current sample when not advancing */
static uint32_t buf_mask;       /* buf_size - 1, cached at play-start */
static uint8_t *buf_ptr;        /* shmem->buffer pointer, cached at play-start */
static uint32_t shmem_update_counter; /* batches shmem->read_idx writes */
static unsigned int out_l, out_r;     /* previous DSM output, for integ1 feedback */

typedef void (*consume_fn_t)(int16_t *, int16_t *);
static consume_fn_t consume_fn;

__attribute__((always_inline)) static inline void gpio_write_both(unsigned int bit_l, unsigned int bit_r)
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
	read_idx = (read_idx + step) & buf_mask;
	if (++shmem_update_counter >= 8) {
		shmem_update_counter = 0;
		shmem->read_idx = read_idx;
		__dmb();
	}
}

static void consume_s16_stereo(int16_t *s16_l, int16_t *s16_r)
{
	uint32_t i = read_idx & buf_mask;
	*s16_l = (int16_t)(buf_ptr[i] | (buf_ptr[(i + 1) & buf_mask] << 8));
	i = (read_idx + 2) & buf_mask;
	*s16_r = (int16_t)(buf_ptr[i] | (buf_ptr[(i + 1) & buf_mask] << 8));
	advance_read_idx(4);
}

static void consume_s16_mono(int16_t *s16_l, int16_t *s16_r)
{
	uint32_t i = read_idx & buf_mask;
	*s16_l = (int16_t)(buf_ptr[i] | (buf_ptr[(i + 1) & buf_mask] << 8));
	*s16_r = *s16_l;
	advance_read_idx(2);
}

static void consume_u8(int16_t *s16_l, int16_t *s16_r)
{
	uint8_t u = buf_ptr[read_idx & buf_mask];
	*s16_l = (int16_t)((u << 8) - 32768);
	*s16_r = *s16_l;
	advance_read_idx(1);
}

__attribute__((section(".ramfunc")))
void timer5_isr(void)
{
	clear_timer5_irq();

	phase_acc += sample_rate_hz;
	if (phase_acc >= DS_RATE_HZ) {
		phase_acc -= DS_RATE_HZ;
		consume_fn(&last_l, &last_r);
	}

	/* 2nd-order delta-sigma for left (GPIO4_B2) — branchless: (2*out-1) encodes ±1 */
	{
		uint32_t fb_l = (out_l << 1) - 1u;
		integ1_l += last_l - (int32_t)(fb_l << DSM_HALF_SCALE_SHIFT);
		integ2_l += integ1_l;
		out_l = ((uint32_t)integ2_l >> 31) ^ 1u;
		fb_l = (out_l << 1) - 1u;
		integ2_l -= (int32_t)(fb_l << DSM_FULL_SCALE_SHIFT);
	}

	/* 2nd-order delta-sigma for right (GPIO4_B3) */
	{
		uint32_t fb_r = (out_r << 1) - 1u;
		integ1_r += last_r - (int32_t)(fb_r << DSM_HALF_SCALE_SHIFT);
		integ2_r += integ1_r;
		out_r = ((uint32_t)integ2_r >> 31) ^ 1u;
		fb_r = (out_r << 1) - 1u;
		integ2_r -= (int32_t)(fb_r << DSM_FULL_SCALE_SHIFT);
	}

	gpio_write_both(out_l, out_r);
}

static void nvic_enable_timer5(void)
{
	/* NVIC_ISER0: IRQ 19 is bit 19 of word at offset 0 */
	REG(0xE000E100) = (1u << TIMER0_CH5_IRQ);
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
	nvic_enable_timer5();
}

int main(void)
{
	hardware_init();

	for (;;) {
		while (shmem->magic != M0_AUDIO_MAGIC)
			;

		if (shmem->ctrl == M0_CTRL_PLAY) {
			sample_rate_hz = shmem->sample_rate;
			buf_size = shmem->buf_size;
			buf_mask = buf_size - 1;
			buf_ptr = (uint8_t *)shmem->buffer;
			read_idx = shmem->read_idx;
			channels = shmem->channels;
			format = shmem->format;
			shmem_update_counter = 0;
			integ1_l = integ2_l = 0;
			integ1_r = integ2_r = 0;
			out_l = out_r = 0;
			phase_acc = 0;
			last_l = last_r = 0;
			if (format == M0_FMT_S16_LE && channels >= 2)
				consume_fn = consume_s16_stereo;
			else if (format == M0_FMT_S16_LE)
				consume_fn = consume_s16_mono;
			else
				consume_fn = consume_u8;
			timer5_start();
			while (shmem->ctrl == M0_CTRL_PLAY)
				;
			/* Stop the timer, then clear INTSTAT (deassert IRQ level) and NVIC
			 * pending so no stray ISR fires after this point. */
			timer5_stop();
			REG(TIMER0_CH5_BASE + TIMER_INTSTAT) = 1;
			REG(NVIC_ICPR0) = (1u << TIMER0_CH5_IRQ);
			gpio_write_both(0, 0);
		}
	}
}
