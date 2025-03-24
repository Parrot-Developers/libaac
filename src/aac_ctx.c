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


int aac_ctx_new(struct aac_ctx **ret_obj)
{
	struct aac_ctx *ctx = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);
	*ret_obj = NULL;

	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL)
		return -ENOMEM;

	*ret_obj = ctx;
	return 0;
}


int aac_ctx_destroy(struct aac_ctx *ctx)
{
	if (ctx == NULL)
		return 0;
	aac_ctx_clear(ctx);
	free(ctx);
	return 0;
}


int aac_ctx_clear(struct aac_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	memset(ctx, 0, sizeof(*ctx));
	return 0;
}


int aac_ctx_clear_adts(struct aac_ctx *ctx)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	memset(&ctx->adts, 0, sizeof(ctx->adts));
	return 0;
}


const struct aac_adts *aac_ctx_get_adts(struct aac_ctx *ctx)
{
	ULOG_ERRNO_RETURN_VAL_IF(ctx == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(
		ctx->data_format != ADEF_AAC_DATA_FORMAT_ADTS, EINVAL, NULL);
	return &ctx->adts;
}


int aac_ctx_set_adts(struct aac_ctx *ctx, const struct aac_adts *adts)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(adts == NULL, EINVAL);

	ctx->data_format = ADEF_AAC_DATA_FORMAT_ADTS;
	ctx->adts = *adts;
	return 0;
}


const struct aac_asc *aac_ctx_get_asc(struct aac_ctx *ctx)
{
	ULOG_ERRNO_RETURN_VAL_IF(ctx == NULL, EINVAL, NULL);
	ULOG_ERRNO_RETURN_VAL_IF(
		ctx->data_format != ADEF_AAC_DATA_FORMAT_RAW, EINVAL, NULL);
	return &ctx->asc;
}


int aac_ctx_set_asc(struct aac_ctx *ctx, const struct aac_asc *asc)
{
	ULOG_ERRNO_RETURN_ERR_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(asc == NULL, EINVAL);

	ctx->data_format = ADEF_AAC_DATA_FORMAT_RAW;
	ctx->asc = *asc;
	return 0;
}
