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
static uint8_t playing;
static int16_t last_l, last_r;  /* hold current sample when not advancing */

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

/* Get next sample(s) and advance read_idx. Returns L in *s16_l, R in *s16_r (stereo). */
static void consume_frame(int16_t *s16_l, int16_t *s16_r)
{
	uint32_t mask = buf_size - 1;
	uint8_t *buf = (uint8_t *)shmem->buffer;

	if (format == M0_FMT_S16_LE) {
		uint32_t i = read_idx & mask;
		*s16_l = (int16_t)(buf[i] | (buf[i + 1] << 8));
		if (channels >= 2) {
			i = (read_idx + 2) & mask;
			*s16_r = (int16_t)(buf[i] | (buf[i + 1] << 8));
		} else {
			*s16_r = *s16_l;
		}
		read_idx = (read_idx + (channels >= 2 ? 4 : 2)) & mask;
		shmem->read_idx = read_idx;
	} else {
		uint8_t u = buf[read_idx & mask];
		*s16_l = (int16_t)((u << 8) - 32768);
		*s16_r = *s16_l;
		read_idx = (read_idx + 1) & mask;
		shmem->read_idx = read_idx;
	}
}

void timer5_isr(void)
{
	clear_timer5_irq();
	if (!playing) {
		gpio_write_both(0, 0);
		return;
	}

	phase_acc += sample_rate_hz;
	if (phase_acc >= DS_RATE_HZ) {
		phase_acc -= DS_RATE_HZ;
		consume_frame(&last_l, &last_r);
	}

	/* 2nd-order delta-sigma for left (GPIO4_B2) */
	integ1_l += last_l;
	integ2_l += integ1_l;
	unsigned int out_l = (integ2_l >= 0) ? 1 : 0;
	integ2_l -= out_l ? (int32_t)DSM_FULL_SCALE : (int32_t)(-(int32_t)DSM_FULL_SCALE);

	/* 2nd-order delta-sigma for right (GPIO4_B3) */
	integ1_r += last_r;
	integ2_r += integ1_r;
	unsigned int out_r = (integ2_r >= 0) ? 1 : 0;
	integ2_r -= out_r ? (int32_t)DSM_FULL_SCALE : (int32_t)(-(int32_t)DSM_FULL_SCALE);

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
			read_idx = shmem->read_idx;
			channels = shmem->channels;
			format = shmem->format;
			integ1_l = integ2_l = 0;
			integ1_r = integ2_r = 0;
			phase_acc = 0;
			last_l = last_r = 0;
			playing = 1;
			timer5_start();
			while (shmem->ctrl == M0_CTRL_PLAY)
				;
			playing = 0;
			timer5_stop();
			gpio_write_both(0, 0);
		}
	}
}
