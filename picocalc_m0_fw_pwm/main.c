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

static void gpio_write_both(unsigned int bit_l, unsigned int bit_r)
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

static void clear_timer5_irq(void)
{
	REG(TIMER0_CH5_BASE + TIMER_INTSTAT) = 1;
}

/* Advance read_idx by step bytes and batch-write to shared memory every 8 frames. */
static void advance_read_idx(uint32_t step)
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

	/* 2nd-order delta-sigma for left (GPIO4_B2) */
	integ1_l += last_l - (out_l ? DSM_HALF_SCALE : -DSM_HALF_SCALE);
	integ2_l += integ1_l;
	out_l = (integ2_l >= 0) ? 1 : 0;
	integ2_l -= out_l ? (int32_t)DSM_FULL_SCALE : -(int32_t)DSM_FULL_SCALE;

	/* 2nd-order delta-sigma for right (GPIO4_B3) */
	integ1_r += last_r - (out_r ? DSM_HALF_SCALE : -DSM_HALF_SCALE);
	integ2_r += integ1_r;
	out_r = (integ2_r >= 0) ? 1 : 0;
	integ2_r -= out_r ? (int32_t)DSM_FULL_SCALE : -(int32_t)DSM_FULL_SCALE;

	gpio_write_both(out_l, out_r);
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
			nvic_enable_timer5();
			timer5_start();
			while (shmem->ctrl == M0_CTRL_PLAY)
				;
			timer5_stop();
			nvic_disable_timer5();
			gpio_write_both(0, 0);
		}
	}
}
