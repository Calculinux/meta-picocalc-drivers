/* SPDX-License-Identifier: GPL-2.0 */
/* M0 delta-sigma audio: single TIMER0_CH5 ISR drives both GPIO4_B2 (L) and B3 (R).
 * Power saving: WFI when idle, SysTick wakes to poll shmem (no host→M0 IRQ);
 * optional clock gating when not playing. */

#include "rk3506_regs.h"
#include "shmem.h"

#define REG(addr)   (*(volatile uint32_t *)(addr))

/* Idle polling: SysTick period (ms). WFI until SysTick fires, then re-check shmem. */
#define M0_IDLE_POLL_MS   10U
/* M0 core clock (Hz) for SysTick reload. Adjust if core runs at a different rate. */
#define M0_CPU_HZ         200000000U

/* Cortex-M SysTick (ARM standard, base 0xE000E010) — used only when idle for WFI wake. */
#define SYSTICK_BASE      0xE000E010U
#define SYSTICK_CSR       (SYSTICK_BASE + 0x00U)
#define SYSTICK_RVR       (SYSTICK_BASE + 0x04U)
#define SYSTICK_CVR       (SYSTICK_BASE + 0x08U)
#define SYSTICK_CSR_ENABLE   (1u << 0)
#define SYSTICK_CSR_TICKINT  (1u << 1)
#define SYSTICK_CSR_CLKSOURCE (1u << 2)  /* 1 = processor clock */
#define __WFI()            __asm volatile ("wfi")

/*
 * Fixed config: fewer memory loads in the ISR and no format/sample-rate logic.
 * ALSA on the host can do sample-rate conversion to this rate.
 * Linux driver must use the same sample_rate and buf_size.
 */
#define M0_FIXED_SAMPLE_RATE_HZ  48000U
#define M0_FIXED_BUF_SIZE        8192U
#define M0_FIXED_BUF_MASK         (M0_FIXED_BUF_SIZE - 1U)

/*
 * Single struct for hand-tuned asm ISR: state and constants inlined (one load per
 * access). Only timer_irq, gpio_dr, buf_ptr, shmem_base are addresses; rest are values.
 * Layout byte offsets must match isr.S. Left and right DSM state are contiguous
 * for LDM/STM (I1, I2, LAST, OUT per channel).
 */
typedef struct {
	uint32_t timer_irq;
	uint32_t gpio_dr;
	uint32_t phase_acc;
	uint32_t read_idx;
	uint32_t buf_ptr;
	uint32_t shmem_ctr;
	uint32_t _pad0;
	uint32_t _pad1;
	/* Left channel DSM state (contiguous for LDM/STM) */
	int32_t  integ1_l;
	int32_t  integ2_l;
	int32_t  last_l;
	uint32_t out_l;
	/* Right channel DSM state (contiguous for LDM/STM) */
	int32_t  integ1_r;
	int32_t  integ2_r;
	int32_t  last_r;
	uint32_t out_r;
	uint32_t shmem_base;
	uint32_t rate_48k;
	uint32_t ds_rate;
	uint32_t buf_mask;
	uint32_t dsm_half;
	uint32_t dsm_full;
	uint32_t batch;
	uint32_t gpio_base_mask;
} m0_isr_globs_t;

m0_isr_globs_t m0_isr_globs __attribute__((used)) = {
	.timer_irq     = TIMER0_CH5_BASE + TIMER_INTSTAT,
	.gpio_dr       = GPIO4_BASE + GPIO_DR_L,
	.rate_48k      = M0_FIXED_SAMPLE_RATE_HZ,
	.ds_rate       = DS_RATE_HZ,
	.buf_mask      = M0_FIXED_BUF_MASK,
	.dsm_half      = (uint32_t)DSM_HALF_SCALE,
	.dsm_full      = (uint32_t)DSM_FULL_SCALE,
	.batch         = 8,
	.gpio_base_mask = (0x0C00U << 16),
	.shmem_base    = (uint32_t)M0_SHMEM_ADDR,
};

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
	m0_isr_globs.read_idx = (m0_isr_globs.read_idx + step) & m0_isr_globs.buf_mask;
	if (++m0_isr_globs.shmem_ctr >= m0_isr_globs.batch) {
		m0_isr_globs.shmem_ctr = 0;
		((m0_audio_shmem_t *)m0_isr_globs.shmem_base)->read_idx = m0_isr_globs.read_idx;
		__dmb();
	}
}

/* S16 LE stereo only (fixed config). Used from main loop only; ISR uses asm consume. */
__attribute__((always_inline)) static inline void consume_s16_stereo(int32_t *s32_l, int32_t *s32_r)
{
	uint8_t *buf = (uint8_t *)m0_isr_globs.buf_ptr;
	uint32_t i = m0_isr_globs.read_idx & m0_isr_globs.buf_mask;
	*s32_l = (int16_t)(buf[i] | (buf[(i + 1) & m0_isr_globs.buf_mask] << 8));
	i = (m0_isr_globs.read_idx + 2) & m0_isr_globs.buf_mask;
	*s32_r = (int16_t)(buf[i] | (buf[(i + 1) & m0_isr_globs.buf_mask] << 8));
	advance_read_idx(4);
}

/* Hand-tuned asm ISR: load-order scheduling to avoid stalls, minimal push/pop. */
void TIMER0_CH5_IRQHandler(void);

/* Set by SysTick when idle; wakes CPU from WFI so we re-poll shmem. No host→M0 IRQ. */
static volatile uint32_t systick_wake_flag;

void SysTick_Handler(void)
{
	systick_wake_flag = 1;
}

static void systick_start_idle(void)
{
	/* Reload value for M0_IDLE_POLL_MS; SysTick fires every (RVR+1) cycles. */
	uint32_t rvr = (M0_CPU_HZ / 1000U) * M0_IDLE_POLL_MS;
	if (rvr > 0x00FFFFFFU)
		rvr = 0x00FFFFFFU;
	REG(SYSTICK_RVR) = rvr - 1U;
	REG(SYSTICK_CVR) = 0U;
	systick_wake_flag = 0;
	REG(SYSTICK_CSR) = SYSTICK_CSR_ENABLE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_CLKSOURCE;
}

static void systick_stop_idle(void)
{
	REG(SYSTICK_CSR) = 0U;
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

/* Re-enable TIMER5 and GPIO4 clocks before starting playback (idle may gate them). */
static void clocks_ungate_play(void)
{
	REG(CRU_BASE + CRU_GATE_CON06) = CRU_TIMER5_EN;
	REG(CRU_BASE + CRU_CLKSEL_CON23) = CRU_TIMER5_100M;
	REG(CRU_BASE + CRU_GATE_CON13) = CRU_GPIO4_EN;
}

/* Gate TIMER5/GPIO4 clocks when idle to save power. Optional: set values per TRM. */
static void clocks_gate_idle(void)
{
	/* If TRM defines disable values for CRU_GATE_CON06 / CRU_GATE_CON13 (e.g. write-enable
	 * plus clear gate bit), add REG(CRU_BASE + ...) = value here. Same register may control
	 * other peripherals — use read-modify-write if needed. */
}

int main(void)
{
	hardware_init();

	for (;;) {
		m0_audio_shmem_t *shmem = (m0_audio_shmem_t *)m0_isr_globs.shmem_base;

		systick_start_idle();
		while (shmem->magic != M0_AUDIO_MAGIC)
			__WFI();
		while (shmem->ctrl != M0_CTRL_PLAY)
			__WFI();
		systick_stop_idle();

		clocks_ungate_play();
		m0_isr_globs.buf_ptr = (uint32_t)shmem->buffer;
		m0_isr_globs.read_idx = shmem->read_idx;
		m0_isr_globs.shmem_ctr = 0;
		m0_isr_globs.integ1_l = m0_isr_globs.integ2_l = 0;
		m0_isr_globs.integ1_r = m0_isr_globs.integ2_r = 0;
		m0_isr_globs.out_l = m0_isr_globs.out_r = 0;
		m0_isr_globs.phase_acc = 0;
		m0_isr_globs.last_l = m0_isr_globs.last_r = 0;
		__asm volatile("" ::: "memory"); /* compiler barrier: state visible before IRQ */
		nvic_enable_timer5();
		timer5_start();
		while (shmem->ctrl == M0_CTRL_PLAY)
			;
		timer5_stop();
		REG(TIMER0_CH5_BASE + TIMER_INTSTAT) = 1; /* clear any pending IRQ before masking */
		nvic_disable_timer5();
		gpio_write_both(0, 0);
		clocks_gate_idle();
	}
}
