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

#ifndef _AAC_BITSTREAM_H_
#define _AAC_BITSTREAM_H_


struct aac_bitstream {
	union {
		/* Data pointer (const) */
		const uint8_t *cdata;

		/* Data pointer */
		uint8_t *data;
	};

	/* Data length */
	size_t len;

	/* Offset in data */
	size_t off;

	/* Partial read/write byte */
	uint8_t cache;

	/* Number of bits in cache */
	uint8_t cachebits;

	/* Dynamic */
	int dynamic;

	/* Private data */
	void *priv;
};


AAC_API
int aac_bs_write_bits(struct aac_bitstream *bs, uint64_t v, uint32_t n);


AAC_API int
aac_bs_next_bits(const struct aac_bitstream *bs, uint32_t *v, uint32_t n);


AAC_API int aac_bs_read_trailing_bits(struct aac_bitstream *bs);


AAC_API int aac_bs_write_trailing_bits(struct aac_bitstream *bs);


AAC_API
int aac_bs_read_raw_bytes(struct aac_bitstream *bs, uint8_t *buf, size_t len);


AAC_API
int aac_bs_write_raw_bytes(struct aac_bitstream *bs,
			   const uint8_t *buf,
			   size_t len);


AAC_API
int aac_bs_acquire_buf(struct aac_bitstream *bs, uint8_t **buf, size_t *len);


static inline void
aac_bs_cinit(struct aac_bitstream *bs, const uint8_t *buf, size_t len)
{
	memset(bs, 0, sizeof(*bs));
	bs->cdata = buf;
	bs->len = len;
}


static inline void
aac_bs_init(struct aac_bitstream *bs, uint8_t *buf, size_t len)
{
	memset(bs, 0, sizeof(*bs));
	bs->data = buf;
	bs->len = len;
	bs->dynamic = (buf == NULL && len == 0);
}


static inline void aac_bs_clear(struct aac_bitstream *bs)
{
	if (bs->dynamic)
		free(bs->data);
	memset(bs, 0, sizeof(*bs));
}


static inline int aac_bs_byte_aligned(const struct aac_bitstream *bs)
{
	return bs->cachebits % 8 == 0;
}


static inline int aac_bs_eos(const struct aac_bitstream *bs)
{
	return bs->off >= bs->len && bs->cachebits == 0;
}


static inline size_t aac_bs_rem_raw_bits(const struct aac_bitstream *bs)
{
	return (bs->len - bs->off) * 8 + bs->cachebits;
}


static inline int aac_bs_fetch(struct aac_bitstream *bs)
{
	if (bs->off < bs->len) {
		bs->cache = bs->cdata[bs->off];
		bs->cachebits = 8;
		bs->off++;
		return 0;
	} else {
		/* End of stream reached */
		return -EIO;
	}
}


static inline int
aac_bs_read_bits(struct aac_bitstream *bs, uint32_t *v, uint32_t n)
{
	int res = 0;
	uint32_t bits = 0;
	uint32_t mask = 0;
	uint32_t part = 0;

	*v = 0;
	while (n > 0) {
		/* Fetch data if needed */
		if (bs->cachebits == 0 && aac_bs_fetch(bs) < 0)
			return -EIO;

		/* Read as many bits from cache */
		bits = n < bs->cachebits ? n : bs->cachebits;
		mask = (1 << bits) - 1;
		part = (bs->cache >> (bs->cachebits - bits)) & mask;
		*v = (*v << bits) | part;
		n -= bits;
		bs->cachebits -= bits;
		res += bits;
	}

	return res;
}


static inline int
aac_bs_read_bits_u(struct aac_bitstream *bs, uint32_t *v, uint32_t n)
{
	return aac_bs_read_bits(bs, v, n);
}


static inline int
aac_bs_write_bits_u(struct aac_bitstream *bs, uint32_t v, uint32_t n)
{
	return aac_bs_write_bits(bs, v, n);
}


static inline int
aac_bs_read_bits_i(struct aac_bitstream *bs, int32_t *v, uint32_t n)
{
	int res = 0;
	uint32_t u32 = 0;

	res = aac_bs_read_bits(bs, &u32, n);
	if (res >= 0) {
		/* Sign extend result */
		if ((u32 & (1 << (n - 1))) != 0)
			*v = (int32_t)(u32 | ((uint32_t)-1) << n);
		else
			*v = (int32_t)u32;
	}

	return res;
}


static inline int
aac_bs_write_bits_i(struct aac_bitstream *bs, int32_t v, uint32_t n)
{
	return aac_bs_write_bits_u(bs, ((uint32_t)v) & ((1 << n) - 1), n);
}


#endif /* !_AAC_BITSTREAM_H_ */
