/* SPDX-License-Identifier: GPL-2.0
 * Host-side check of M0 ring copy/space/underrun math (no kernel).
 * cc -O2 -Wall -Wextra -o check_ring check_ring.c && ./check_ring
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define BUF 8192U
#define MASK (BUF - 1U)
#define FRAME 4U
#define PERIOD 1024U
#define ALSA_BUF 32768U

static uint32_t ring_space(uint32_t write_idx, uint32_t read_idx, uint32_t buf_size)
{
	if (write_idx >= read_idx)
		return buf_size - (write_idx - read_idx) - 1;
	return read_idx - write_idx - 1;
}

static uint32_t appl_avail(uint32_t appl_off, uint32_t copy_off, uint32_t buffer_bytes)
{
	if (appl_off >= copy_off)
		return appl_off - copy_off;
	return buffer_bytes - copy_off + appl_off;
}

static uint32_t copy_size(uint32_t period, uint32_t space, uint32_t avail, uint32_t frame)
{
	uint32_t n = period;

	if (n > space)
		n = space;
	if (n > avail)
		n = avail;
	return n - (n % frame);
}

static int consume(uint32_t *read_idx, uint32_t write_idx, uint32_t mask)
{
	if (write_idx == *read_idx)
		return 0;
	*read_idx = (*read_idx + FRAME) & mask;
	return 1;
}

int main(void)
{
	uint32_t w = 0, r = 0, copied = 0, n, i;

	assert(ring_space(0, 0, BUF) == BUF - 1);
	assert(ring_space(0, 4, BUF) == 3);
	assert(ring_space(100, 0, BUF) == BUF - 100 - 1);

	/* Space-limited copy must stay frame-aligned (old bug: space = 4k-1). */
	n = copy_size(PERIOD, ring_space(0, 0, BUF), PERIOD, FRAME);
	assert(n == PERIOD);
	n = copy_size(8192, ring_space(0, 0, BUF), 8192, FRAME);
	assert(n == 8192 - 4);
	assert(n % FRAME == 0);

	/* Streaming past one ALSA buffer (old bug: copied + n <= ALSA_BUF). */
	for (i = 0; i < 40; i++) {
		uint32_t appl = (copied + PERIOD) % ALSA_BUF;

		assert(appl_avail(appl, copied, ALSA_BUF) >= PERIOD);
		n = copy_size(PERIOD, ring_space(w, r, BUF),
			      appl_avail(appl, copied, ALSA_BUF), FRAME);
		assert(n > 0);
		w = (w + n) & MASK;
		copied += n;
		if (copied >= ALSA_BUF)
			copied -= ALSA_BUF;
		assert(copied < ALSA_BUF);
		/* M0 consumes the period we just wrote */
		{
			uint32_t left = n;

			while (left >= FRAME) {
				assert(consume(&r, w, MASK) == 1);
				left -= FRAME;
			}
		}
	}

	/* Underrun: hold last, do not advance read_idx. */
	r = w;
	assert(consume(&r, w, MASK) == 0);
	assert(r == w);
	w = (w + FRAME) & MASK;
	assert(consume(&r, w, MASK) == 1);
	assert(r == w);

	/* Wrap write past end of ring. */
	w = BUF - FRAME;
	r = 0;
	n = copy_size(PERIOD, ring_space(w, r, BUF), PERIOD, FRAME);
	assert(n % FRAME == 0);
	w = (w + n) & MASK;
	assert(w < BUF);

	/* appl_ptr wrap vs copy offset */
	assert(appl_avail(0, ALSA_BUF - PERIOD, ALSA_BUF) == PERIOD);
	assert(appl_avail(0, 0, ALSA_BUF) == 0);

	puts("check_ring: ok");
	return 0;
}
