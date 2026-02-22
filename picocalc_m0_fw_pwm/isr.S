/* SPDX-License-Identifier: GPL-2.0 */
/* Hand-tuned TIMER0_CH5 ISR: load-order scheduling to avoid load-use stalls on Cortex-M0.
 * Cycle counts per instruction (Cortex-M0, zero-wait-state RAM):
 *   1: ALU (adds, subs, movs, lsls, lsrs, asrs, orrs, ands), cmp, mov (reg)
 *   2: ldr, str, ldrsh, b/bl (taken), push/pop 1 reg
 *   3: dmb
 *   LDM/STM: 1+N cycles for N registers. Save/restore via r8-r11 + push {lr}/pop {pc}.
 */

.syntax unified
.thumb
.cpu cortex-m0

.section .ramfunc, "ax", %progbits
.align 2
.global TIMER0_CH5_IRQHandler
.thumb_func
.type TIMER0_CH5_IRQHandler, %function

/* m0_isr_globs struct: all state and constants at direct offsets (no indirection).
 * Left/right DSM state contiguous for LDM/STM (1+N cycles). */
.equ O_TIMER,   0
.equ O_GPIO,    4
.equ O_PHASE,   8
.equ O_READ_IDX, 12
.equ O_BUF_PTR, 16
.equ O_CTR,     20
.equ O_I1L,     32
.equ O_I2L,     36
.equ O_LAST_L,  40
.equ O_OUTL,    44
.equ O_I1R,     48
.equ O_I2R,     52
.equ O_LAST_R,  56
.equ O_OUTR,    60
.equ O_SHMEM,   64
.equ O_RATE,    68
.equ O_DSRATE,  72
.equ O_MASK,    76
.equ O_HALF,    80
.equ O_FULL,    84
.equ O_BATCH,   88
.equ O_GPIO_MASK, 92
.equ SHMEM_READ_IDX_OFF, 12

TIMER0_CH5_IRQHandler:
	push    {lr}                         @ 2
	mov     r8, r4                       @ 1
	mov     r9, r5                       @ 1
	mov     r10, r6                      @ 1
	mov     r11, r7                      @ 1
	ldr     r5, .L_globs                 @ 2

	/* Clear timer IRQ; then phase (direct load). */
	movs    r0, #1                       @ 1
	ldr     r1, [r5, #O_TIMER]          @ 2
	str     r0, [r1]                     @ 2
	ldr     r1, [r5, #O_PHASE]          @ 2
	ldr     r0, [r5, #O_RATE]           @ 2
	adds    r1, r1, r0                   @ 1
	ldr     r0, [r5, #O_DSRATE]         @ 2
	cmp     r1, r0                       @ 1
	bls     .L_no_consume                @ 2

	/* phase >= DS_RATE: subtract, store phase, then consume inlined. */
	subs    r1, r1, r0                   @ 1
	str     r1, [r5, #O_PHASE]           @ 2
	/* Consume: read_idx and buf_ptr direct; keep mask in r7 for read_idx update. */
	ldr     r1, [r5, #O_READ_IDX]       @ 2
	ldr     r3, [r5, #O_BUF_PTR]        @ 2
	ldr     r0, [r5, #O_MASK]           @ 2
	mov     r7, r0                       @ 1
	mov     r2, r1                       @ 1
	ands    r2, r0                       @ 1
	adds    r4, r3, r2                   @ 1
	movs    r3, #0                       @ 1
	ldrsh   r0, [r4, r3]                @ 2
	str     r0, [r5, #O_LAST_L]         @ 2
	adds    r4, r4, #2                   @ 1
	ldrsh   r0, [r4, r3]                @ 2
	str     r0, [r5, #O_LAST_R]         @ 2
	adds    r1, r1, #4                   @ 1
	ands    r1, r7                       @ 1
	str     r1, [r5, #O_READ_IDX]        @ 2
	ldr     r0, [r5, #O_CTR]            @ 2
	adds    r0, r0, #1                   @ 1
	str     r0, [r5, #O_CTR]             @ 2
	ldr     r3, [r5, #O_BATCH]          @ 2
	cmp     r0, r3                       @ 1
	blo     .L_consume_done              @ 2
	movs    r0, #0                       @ 1
	str     r0, [r5, #O_CTR]             @ 2
	ldr     r2, [r5, #O_SHMEM]          @ 2
	adds    r2, r2, #SHMEM_READ_IDX_OFF @ 1
	str     r1, [r2]                     @ 2
	dmb     sy                           @ 3
.L_consume_done:
	b       .L_dsm                       @ 2

.L_no_consume:
	str     r1, [r5, #O_PHASE]           @ 2

.L_dsm:
	/* Left DSM: LDM 4 words. Thumb-1: base = r5+O_I1L in two steps (imm8 only). */
	mov     r0, r5                       @ 1
	adds    r0, #O_I1L                   @ 1
	ldm     r0!, {r1, r2, r3, r4}       @ 5  r1=i1l, r2=i2l, r3=last_l, r4=outl
	ldr     r0, [r5, #O_HALF]           @ 2  half (low reg for adds)
	adds    r1, r1, r3                   @ 1  integ1 += last_l
	adds    r1, r1, r0                   @ 1  integ1 += half
	lsls    r4, r4, #15                  @ 1
	subs    r1, r1, r4                   @ 1  integ1_new
	str     r1, [r5, #O_I1L]             @ 2
	adds    r2, r2, r1                   @ 1  integ2 += integ1_new
	asrs    r4, r2, #31                  @ 1  sign
	movs    r0, #1                       @ 1
	subs    r4, r0, r4                   @ 1  out_new
	ldr     r0, [r5, #O_FULL]           @ 2
	adds    r2, r2, r0                   @ 1  integ2 += full
	lsls    r0, r4, #16                  @ 1
	subs    r2, r2, r0                   @ 1  integ2_new
	str     r2, [r5, #O_I2L]             @ 2
	str     r4, [r5, #O_OUTL]             @ 2
	mov     r7, r4                       @ 1  keep out_l for GPIO

	/* Right DSM: LDM 4 words; load HALF again (Thumb-1 has no high-reg adds). */
	mov     r0, r5                       @ 1
	adds    r0, #O_I1R                   @ 1
	ldm     r0!, {r1, r2, r3, r4}       @ 5
	ldr     r0, [r5, #O_HALF]           @ 2
	adds    r1, r1, r3                   @ 1
	adds    r1, r1, r0                   @ 1
	lsls    r4, r4, #15                  @ 1
	subs    r1, r1, r4                   @ 1
	str     r1, [r5, #O_I1R]             @ 2
	adds    r2, r2, r1                   @ 1
	asrs    r4, r2, #31                  @ 1
	movs    r0, #1                       @ 1
	subs    r4, r0, r4                   @ 1
	ldr     r0, [r5, #O_FULL]           @ 2
	adds    r2, r2, r0                   @ 1  integ2 += full
	lsls    r0, r4, #16                  @ 1
	subs    r2, r2, r0                   @ 1  integ2_new
	str     r2, [r5, #O_I2R]             @ 2
	str     r4, [r5, #O_OUTR]             @ 2

	/* GPIO: precomputed base mask + (out_l<<10) | (out_r<<11). */
	mov     r1, r7                       @ 1  out_l
	ldr     r2, [r5, #O_GPIO_MASK]      @ 2
	lsls    r1, r1, #10                  @ 1
	lsls    r0, r4, #11                  @ 1  out_r (r4 from right DSM)
	orrs    r1, r1, r0                   @ 1
	orrs    r2, r2, r1                   @ 1
	ldr     r0, [r5, #O_GPIO]           @ 2
	str     r2, [r0]                     @ 2

	mov     r4, r8                       @ 1
	mov     r5, r9                       @ 1
	mov     r6, r10                      @ 1
	mov     r7, r11                      @ 1
	pop     {pc}                         @ 2

.L_globs:
	.word   m0_isr_globs
.size TIMER0_CH5_IRQHandler, . - TIMER0_CH5_IRQHandler
