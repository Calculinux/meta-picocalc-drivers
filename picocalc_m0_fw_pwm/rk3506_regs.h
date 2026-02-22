/* SPDX-License-Identifier: GPL-2.0 */
/* RK3506 register definitions for M0 delta-sigma audio (from TRM) */

#ifndef RK3506_REGS_H
#define RK3506_REGS_H

#include "stdint.h"

/* GPIO4 — bit-bang output (GPIO4_B2 = left, GPIO4_B3 = right) */
#define GPIO4_BASE        0xFF1E0000U
#define GPIO_DR_L          0x0000U
#define GPIO_DDR_L         0x0008U
#define GPIO4_B2_MASK     (1u << (16+10))
#define GPIO4_B3_MASK     (1u << (16+11))
#define GPIO4_B23_OUT_DIR 0x0C000C00U
#define GPIO4_B23_CLR     (GPIO4_B2_MASK | GPIO4_B3_MASK)

/* Build atomic GPIO write: mask bits 26,27; data bits 10,11 from bit_l and bit_r */
#define GPIO4_DR_WRITE(bit_l, bit_r) \
	((0x0C00U << 16) | ((bit_l) << 10) | ((bit_r) << 11))

/* GPIO4 IOC — enable digital mode for B2/B3 */
#define GPIO4_IOC_BASE    0xFF4D8000U
#define SARADC_CON        0x0840U
#define SARADC_CON_B23_EN 0x00C000C0U

/* TIMER0_CH5 — 2 MHz delta-sigma interrupt */
#define TIMER0_CH5_BASE   0xFF255000U
#define TIMER_LOAD0       0x0000U
#define TIMER_LOAD1       0x0004U
#define TIMER_CTRL        0x0010U
#define TIMER_INTSTAT     0x0018U
#define TIMER_RUN         0x07U
#define TIMER_STOP        0x00U

/* CRU */
#define CRU_BASE          0xFF9A0000U
#define CRU_GATE_CON06    0x0818U
#define CRU_GATE_CON13    0x0834U
#define CRU_CLKSEL_CON23  0x035CU
#define CRU_TIMER5_EN     0x01040000U
#define CRU_GPIO4_EN      0x000C0000U
#define CRU_TIMER5_100M   0x01C00040U

/* Delta-sigma: 100 MHz clock, 50 ticks = 2 MHz */
#define DS_PERIOD_TICKS   50U
#define DS_RATE_HZ        2000000U
#define DSM_FULL_SCALE    32768

#define TIMER0_CH5_IRQ    19

#endif /* RK3506_REGS_H */
