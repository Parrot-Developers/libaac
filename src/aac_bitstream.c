/**
 * Copyright (c) 2023 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "aac_priv.h"


static int aac_bs_ensure_capacity(struct aac_bitstream *bs, size_t capacity)
{
	uint8_t *newbuf = NULL;

	if (capacity <= bs->len)
		return 0;
	if (!bs->dynamic)
		return -EIO;

	/* Wanted capacity round up */
	capacity = (capacity + 255) & ~255;

	/* Allocate new buffer */
	newbuf = realloc(bs->data, capacity);
	if (newbuf == NULL)
		return -ENOMEM;

	/* Setup new buffer */
	bs->data = newbuf;
	bs->len = capacity;
	return 0;
}


static int aac_bs_flush(struct aac_bitstream *bs)
{
	int res = aac_bs_ensure_capacity(bs, bs->off + 1);
	if (res < 0)
		return res;
	bs->data[bs->off] = bs->cache;
	bs->cache = 0;
	bs->cachebits = 0;
	bs->off++;
	return 0;
}


int aac_bs_write_bits(struct aac_bitstream *bs, uint64_t v, uint32_t n)
{
	int res = 0;
	uint32_t bits = 0;
	uint32_t mask = 0;
	uint32_t part = 0;

	/* Ensure that 'n' is not larger than the number of digits of v */
	ULOG_ERRNO_RETURN_ERR_IF(n > 64, EINVAL);

	while (n > 0) {
		/* Write as many bits to current byte */
		bits = 8 - bs->cachebits;
		if (bits >= n)
			bits = n;
		mask = (1 << bits) - 1;
		part = (v >> (n - bits)) & mask;
		bs->cache |= part << (8 - bs->cachebits - bits);
		n -= bits;
		bs->cachebits += bits;
		res += bits;

		/* Flush data if needed */
		if (bs->cachebits == 8 && aac_bs_flush(bs) < 0)
			return -EIO;
	}

	return res;
}


int aac_bs_next_bits(const struct aac_bitstream *bs, uint32_t *v, uint32_t n)
{
	/* Use a temp stream */
	struct aac_bitstream bs2 = *bs;

	return aac_bs_read_bits(&bs2, v, n);
}


int aac_bs_read_trailing_bits(struct aac_bitstream *bs)
{
	int res = 0;
	uint32_t bit = 0;

	while (!aac_bs_byte_aligned(bs)) {
		/* Read rbsp_alignment_zero_bit, shall be '0' */
		res = aac_bs_read_bits(bs, &bit, 1);
		if (res < 0)
			return res;
		if (bit != 0)
			return -EIO;
	}

	return 0;
}


int aac_bs_write_trailing_bits(struct aac_bitstream *bs)
{
	int res = 0;

	while (!aac_bs_byte_aligned(bs)) {
		/* Write rbsp_alignment_zero_bit */
		res = aac_bs_write_bits(bs, 0, 1);
		if (res < 0)
			return res;
	}

	return 0;
}


int aac_bs_read_raw_bytes(struct aac_bitstream *bs, uint8_t *buf, size_t len)
{
	ULOG_ERRNO_RETURN_ERR_IF(!aac_bs_byte_aligned(bs), EIO);
	ULOG_ERRNO_RETURN_ERR_IF(bs->len - bs->off != len, EIO);
	memcpy(buf, bs->cdata + bs->off, len);
	bs->off += len;
	return 0;
}


int aac_bs_write_raw_bytes(struct aac_bitstream *bs,
			   const uint8_t *buf,
			   size_t len)
{
	int res = 0;
	ULOG_ERRNO_RETURN_ERR_IF(!aac_bs_byte_aligned(bs), EIO);
	res = aac_bs_ensure_capacity(bs, bs->off + len);
	if (res < 0)
		return res;
	memcpy(bs->data + bs->off, buf, len);
	bs->off += len;
	return 0;
}


int aac_bs_acquire_buf(struct aac_bitstream *bs, uint8_t **buf, size_t *len)
{
	ULOG_ERRNO_RETURN_ERR_IF(!aac_bs_byte_aligned(bs), EIO);
	ULOG_ERRNO_RETURN_ERR_IF(!bs->dynamic, EIO);
	*buf = bs->data;
	*len = bs->off;
	bs->dynamic = 0;
	return 0;
}
