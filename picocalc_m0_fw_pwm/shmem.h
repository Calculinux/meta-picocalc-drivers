/* SPDX-License-Identifier: GPL-2.0 */
/* Shared memory layout between Linux driver and M0 (must match both sides) */

#ifndef M0_SHMEM_H
#define M0_SHMEM_H

#include "stdint.h"

#define M0_AUDIO_MAGIC    0x4D305057U  /* "M0PW" */
#define M0_CTRL_PLAY      (1u << 0)
#define M0_CTRL_STOP      0u
#define M0_FMT_U8         0
#define M0_FMT_S16_LE     1

#define M0_SHMEM_ADDR     0x03C00000U

typedef struct {
	volatile uint32_t magic;
	volatile uint32_t ctrl;
	volatile uint32_t write_idx;
	volatile uint32_t read_idx;
	volatile uint32_t period_bytes;
	volatile uint32_t buf_size;
	volatile uint32_t sample_rate;
	volatile uint32_t channels;
	volatile uint32_t format;
	uint32_t          _reserved[7];
	uint8_t           buffer[];
} m0_audio_shmem_t;

#define M0_HEADER_SIZE    (64U)

#endif /* M0_SHMEM_H */
