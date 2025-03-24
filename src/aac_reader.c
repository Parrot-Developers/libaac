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


struct aac_reader {
	struct aac_ctx_cbs cbs;
	void *userdata;
	int stop;
	struct aac_ctx *ctx;
	uint32_t flags;
};


#define AAC_SYNTAX_OP_NAME read
#define AAC_SYNTAX_OP_KIND AAC_SYNTAX_OP_KIND_READ

#define AAC_BITS(_f, _n) AAC_READ_BITS(_f, _n)


#include "aac_syntax.h"


int aac_reader_new(const struct aac_ctx_cbs *cbs,
		   void *userdata,
		   struct aac_reader **ret_obj)
{
	int res = 0;
	struct aac_reader *reader = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);

	/* Allocate structure */
	reader = calloc(1, sizeof(*reader));
	if (reader == NULL)
		return -ENOMEM;

	/* Initialize structure */
	reader->cbs = *cbs;
	reader->userdata = userdata;
	res = aac_ctx_new(&reader->ctx);
	if (res < 0)
		goto error;

	/* Success */
	*ret_obj = reader;
	return 0;

	/* Cleanup in case of error */
error:
	aac_reader_destroy(reader);
	return res;
}


int aac_reader_destroy(struct aac_reader *reader)
{
	if (reader == NULL)
		return 0;
	if (reader->ctx != NULL)
		aac_ctx_destroy(reader->ctx);
	free(reader);
	return 0;
}


struct aac_ctx *aac_reader_get_ctx(struct aac_reader *reader)
{
	return reader == NULL ? NULL : reader->ctx;
}


int aac_reader_stop(struct aac_reader *reader)
{
	ULOG_ERRNO_RETURN_ERR_IF(reader == NULL, EINVAL);
	reader->stop = 1;
	return 0;
}


int aac_reader_parse(struct aac_reader *reader,
		     uint32_t flags,
		     const uint8_t *buf,
		     size_t len,
		     size_t *off)
{
	int res = 0;
	struct aac_bitstream bs;

	ULOG_ERRNO_RETURN_ERR_IF(reader == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(off == NULL, EINVAL);

	reader->stop = 0;
	reader->flags = flags;
	aac_bs_cinit(&bs, buf, len);
	bs.priv = reader;

	if (reader->ctx->data_format == ADEF_AAC_DATA_FORMAT_UNKNOWN) {
		/* Search for ADTS synword (0xFFF) */
		if (len > 2 && buf[0] == 0xFF && (buf[1] >> 4) == 0xF)
			reader->ctx->data_format = ADEF_AAC_DATA_FORMAT_ADTS;
	}

	while (*off < len && !reader->stop && bs.off < bs.len) {
		switch (reader->ctx->data_format) {
		case ADEF_AAC_DATA_FORMAT_RAW:
			res = _aac_read_raw_data_block(
				&bs, reader->ctx, &reader->ctx->raw_data_block);
			*off = bs.off;
			if (res < 0 && res != -EAGAIN)
				goto out;
			break;
		case ADEF_AAC_DATA_FORMAT_ADTS:
			res = _aac_read_adts_frame(&bs,
						   reader->ctx,
						   &reader->cbs,
						   reader->userdata);
			*off = bs.off;
			if (res < 0 && res != -EAGAIN)
				goto out;
			break;
		default:
			res = -EINVAL;
			goto out;
		}
	}
	res = 0;

out:
	aac_bs_clear(&bs);
	return res;
}


int aac_parse_asc(const uint8_t *buf, size_t len, struct aac_asc *asc)
{
	int res = 0;
	struct aac_bitstream bs;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len < 2, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(asc == NULL, EINVAL);
	memset(asc, 0, sizeof(*asc));
	/* Setup bitstream */
	aac_bs_cinit(&bs, buf, len);
	/* Read ASC */
	res = _aac_read_AudioSpecificConfig(&bs, asc);
	aac_bs_clear(&bs);
	return res;
}


int aac_parse_adts(const uint8_t *buf, size_t len, struct aac_adts *adts)
{
	int res = 0;
	struct aac_bitstream bs;
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len < 2, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(adts == NULL, EINVAL);
	memset(adts, 0, sizeof(*adts));
	/* Setup bitstream */
	aac_bs_cinit(&bs, buf, len);
	/* Read ADTS header */
	res = _aac_read_adts_fixed_header(&bs, adts);
	if (res < 0)
		goto out;
	res = _aac_read_adts_variable_header(&bs, adts);
	if (res < 0)
		goto out;
out:
	aac_bs_clear(&bs);
	return res;
}
