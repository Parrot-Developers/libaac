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


#define AAC_SYNTAX_OP_NAME write
#define AAC_SYNTAX_OP_KIND AAC_SYNTAX_OP_KIND_WRITE

#define AAC_BITS(_f, _n) AAC_WRITE_BITS(_f, _n)


#include "aac_syntax.h"


int aac_write_asc(struct aac_asc *asc, uint8_t **buf, size_t *len)
{
	int res = 0;
	struct aac_bitstream bs;
	uint8_t *tmpbuf = NULL;
	size_t tmplen;
	ULOG_ERRNO_RETURN_ERR_IF(asc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == NULL, EINVAL);
	/* Setup bitstream */
	aac_bs_init(&bs, NULL, 0);
	/* Write ASC */
	res = _aac_write_AudioSpecificConfig(&bs, asc);
	if (res < 0)
		goto out;
	res = aac_bs_write_trailing_bits(&bs);
	if (res < 0)
		goto out;
	tmplen = bs.off;
	tmpbuf = calloc(1, tmplen);
	if (tmpbuf == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	memcpy(tmpbuf, bs.data, tmplen);

	*buf = tmpbuf;
	*len = tmplen;

out:
	aac_bs_clear(&bs);
	return res;
}


int aac_write_adts(struct aac_adts *adts, uint8_t **buf, size_t *len)
{
	int res = 0;
	struct aac_bitstream bs;
	uint8_t *tmpbuf = NULL;
	size_t tmplen;
	ULOG_ERRNO_RETURN_ERR_IF(adts == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == NULL, EINVAL);
	/* Setup bitstream */
	aac_bs_init(&bs, NULL, 0);
	/* Write ADTS */
	res = _aac_write_adts_fixed_header(&bs, adts);
	if (res < 0)
		goto out;
	res = _aac_write_adts_variable_header(&bs, adts);
	if (res < 0)
		goto out;
	res = aac_bs_write_trailing_bits(&bs);
	if (res < 0)
		goto out;
	tmplen = bs.off;
	tmpbuf = calloc(1, tmplen);
	if (tmpbuf == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("calloc", -res);
		goto out;
	}

	memcpy(tmpbuf, bs.data, tmplen);

	*buf = tmpbuf;
	*len = tmplen;

out:
	aac_bs_clear(&bs);
	return res;
}


int aac_write_silent_frame(struct aac_bitstream *bs,
			   struct aac_ctx *ctx,
			   unsigned int channel_count,
			   unsigned int frame_length)
{
	int res = 0;
	size_t i = 0;
	size_t frame_min_size = 0;
	struct aac_raw_data_block *block;
	ULOG_ERRNO_RETURN_ERR_IF(bs == NULL && frame_length != 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		ctx->data_format == ADEF_AAC_DATA_FORMAT_UNKNOWN, EINVAL);

	switch (ctx->data_format) {
	case ADEF_AAC_DATA_FORMAT_RAW:
		block = &ctx->raw_data_block;
		break;
	case ADEF_AAC_DATA_FORMAT_ADTS:
		block = &ctx->adts_frame.raw_data_block[0];
		frame_min_size += 56; /* ADTS header length in bits */
		break;
	default:
		return -EINVAL;
	}

	switch (channel_count) {
	case 1:
		frame_min_size += 29; /* Silent SCE length in bits */
		break;
	case 2:
		frame_min_size += 43; /* Silent CPE length in bits */
		break;
	default:
		return -EINVAL;
		break;
	}

	frame_min_size += 3; /* END length in bits */

	if (bs == NULL && frame_length == 0) {
		if (frame_min_size % 8 != 0)
			frame_min_size = (frame_min_size + 7) / 8;
		return frame_min_size;
	}

	memset(block, 0, sizeof(*block));
	size_t fill_len = frame_length * 8 - frame_min_size;
	if (frame_length != 0 && fill_len >= 8) {
		size_t tmp = fill_len;
		for (size_t i = 0; i < fill_len; i += 8 * 269) {
			size_t rem = fill_len - i;
			tmp -= 3; /* "FIL" ID in bits */
			tmp -= 4; /* FIL length in bits */
			if (rem >= 15 * 8)
				tmp -= 8; /* FIL ext in bits */
		}
		fill_len = tmp / 8;
		while (fill_len > 0) {
			if (i >= AAC_MAX_SYN_ELE - 2)
				return -EINVAL;
			size_t fill_size = (fill_len > 269) ? 269 : fill_len;
			block->elements[i].id_syn_ele = AAC_SYN_ELE_ID_FIL;
			block->elements[i]
				.fil.extension_payload.extension_type =
				AAC_EXT_TYPE_FILL;
			block->elements[i].fil.count = fill_size;
			fill_len -= fill_size;
			i++;
		}
	}

	if (channel_count == 1) {
		/* Len: 32 bits */
		block->elements[i].id_syn_ele = AAC_SYN_ELE_ID_SCE;
		block->elements[i].sce.element_instance_tag = 0;
		block->elements[i].sce.ics.global_gain = 0x8C;
		block->elements[i].sce.ics.ics_info.window_shape = 1;
	} else {
		/* Len: 46 bits */
		block->elements[i].id_syn_ele = AAC_SYN_ELE_ID_CPE;
		block->elements[i].cpe.element_instance_tag = 0;
		block->elements[i].cpe.common_window = 1;
		block->elements[i].cpe.ics_info.window_shape = 1;
		block->elements[i].cpe.ics1.global_gain = 0x8C;
		block->elements[i].cpe.ics2.global_gain = 0x8C;
	}

	i++;

	/* Len: 3 bits */
	block->elements[i].id_syn_ele = AAC_SYN_ELE_ID_END;

	block->elements_count = i + 1;

	ctx->adts.aac_frame_length = frame_length;

	switch (ctx->data_format) {
	case ADEF_AAC_DATA_FORMAT_RAW:
		res = _aac_write_raw_data_block(bs, ctx, block);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		break;
	case ADEF_AAC_DATA_FORMAT_ADTS:
		res = _aac_write_adts_frame(bs, ctx, NULL, NULL);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
