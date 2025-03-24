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

#ifndef _AAC_SYNTAX_H_
#define _AAC_SYNTAX_H_

#include "aac_priv.h"
#include "aac_syntax_ops.h"
#include "aac_tables.h"


/* TODO: for debug purpose only */
#define LOG_OFFSET(str)                                                        \
	ULOGD("at %zx: %zd (%zd*8 + %d): " str,                                \
	      bs->off,                                                         \
	      bs->off * 8 + (8 - bs->cachebits),                               \
	      bs->off,                                                         \
	      8 - bs->cachebits);


static int
find_offset_in_bc(struct aac_bitstream *bs, uint32_t (*codebook)[3], int cb_len)
{
	unsigned int len = 0;
	unsigned int cw = 0;
	uint8_t bit = 0;
	for (int off = 0; off < cb_len; ++off) {
		int j = codebook[off][1] - len;
		len += j;
		while (j > 0) {
			AAC_BITS(bit, 1);
			cw = (cw << 1) | (bit & 1);
			--j;
		}
		if (cw == codebook[off][0])
			return off;
	}
	return -ENOENT; /* Code not found in the table */
}


static int get_wxyz(int _unsigned,
		    int dim,
		    int lav,
		    int idx,
		    int *w,
		    int *x,
		    int *y,
		    int *z)
{
	int mod;
	int off;
	if (_unsigned) {
		mod = lav + 1;
		off = 0;
	} else {
		mod = 2 * lav + 1;
		off = lav;
	}
	if (dim == 4) {
		*w = (int)(idx / (mod * mod * mod)) - off;
		idx -= (*w + off) * (mod * mod * mod);
		*x = (int)(idx / (mod * mod)) - off;
		idx -= (*x + off) * (mod * mod);
		*y = (int)(idx / mod) - off;
		idx -= (*y + off) * mod;
		*z = idx - off;
	} else {
		*y = (int)(idx / mod) - off;
		idx -= (*y + off) * mod;
		*z = idx - off;
	}
	return 0;
}


static int huffman_decode_scale_factor(struct aac_bitstream *bs)
{
	int ret = find_offset_in_bc(bs, hcb_sf, 121);
	if (ret < 0)
		return ret;
	return hcb_sf[ret][2];
}


static int huffman_decode_spectral_data(struct aac_bitstream *bs,
					int cb,
					int *w,
					int *x,
					int *y,
					int *z)
{
	int cb_index = INT32_MAX;
	for (size_t i = 0; i < ARRAY_SIZE(hcb_list); i++) {
		if (hcb_list[i].id == cb)
			cb_index = i;
	}
	ULOG_ERRNO_RETURN_ERR_IF(cb_index == INT32_MAX, ENOENT);
	uint32_t(*codebook)[3] = hcb_list[cb_index].codebook;
	int cb_len = hcb_list[cb_index].cb_len;
	int _unsigned = !(hcb_list[cb_index].is_signed);
	int index = 0;
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	uint8_t bit = 0;
#endif
	int ret = find_offset_in_bc(bs, codebook, cb_len);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);

	index = ret;
	ret = get_wxyz(_unsigned,
		       hcb_list[cb_index].dimension,
		       hcb_list[cb_index].lav,
		       hcb_list[cb_index].codebook[index][2],
		       w,
		       x,
		       y,
		       z);
	ULOG_ERRNO_RETURN_ERR_IF(ret < 0, -ret);
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	if (_unsigned) {
		if (hcb_list[cb_index].dimension == 4) {
			if (*w != 0) {
				AAC_BITS(bit, 1);
				if (bit)
					*w = -*w;
			}
			if (*x != 0) {
				AAC_BITS(bit, 1);
				if (bit)
					*x = -*x;
			}
		}
		if (*y != 0) {
			AAC_BITS(bit, 1);
			if (bit)
				*y = -*y;
		}
		if (*z != 0) {
			AAC_BITS(bit, 1);
			if (bit)
				*z = -*z;
		}
	}
#endif
	return 0;
}


#define MAX_QUANTIZED_VALUE 8191


static int get_escape(struct aac_bitstream *bs, bool minus)
{
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	uint8_t bit = 0;
	uint32_t off = 0;
	int i;
	for (i = 4; i < 13; i++) {
		AAC_BITS(bit, 1);
		if (bit == 0)
			break;
	}

	if (i == 13)
		return MAX_QUANTIZED_VALUE + 1;

	AAC_BITS(off, i);
	i = off + (1 << i);

	if (minus)
		i = -i;

	return i;
#else
	return 0;
#endif
}


static inline uint8_t bit_set(int bit, int idx)
{
	return (bit >> idx) & 1;
}


static inline int is_intensity(int sfb_cb_of_g_sfb)
{
	switch (sfb_cb_of_g_sfb) {
	case INTENSITY_HCB:
		return 1;
	case INTENSITY_HCB2:
		return -1;
	default:
		return 0;
	}
}


static inline int is_noise(int sfb_cb_of_g_sfb)
{
	return sfb_cb_of_g_sfb == NOISE_HCB;
}


static inline int has_aacSectionDataResilienceFlag(struct aac_ctx *ctx)
{
	if (ctx->data_format != ADEF_AAC_DATA_FORMAT_RAW)
		return 0;
	return ctx->asc.GASpecificConfig.aacSectionDataResilienceFlag;
}


static inline int has_aacScalefactorDataResilienceFlag(struct aac_ctx *ctx)
{
	if (ctx->data_format != ADEF_AAC_DATA_FORMAT_RAW)
		return 0;
	return ctx->asc.GASpecificConfig.aacScalefactorDataResilienceFlag;
}


static inline int has_aacSpectralDataResilienceFlag(struct aac_ctx *ctx)
{
	if (ctx->data_format != ADEF_AAC_DATA_FORMAT_RAW)
		return 0;
	return ctx->asc.GASpecificConfig.aacSpectralDataResilienceFlag;
}


static int set_dec_info(struct aac_ctx *ctx, struct aac_ics_info *ics_info)
{
	int fs_index;
	switch (ctx->data_format) {
	case ADEF_AAC_DATA_FORMAT_RAW:
		fs_index = ctx->asc.samplingFrequencyIndex;
		break;
	case ADEF_AAC_DATA_FORMAT_ADTS:
		fs_index = ctx->adts.sampling_frequency_index;
		break;
	default:
		return -EINVAL;
	}
	switch (ics_info->window_sequence) {
	case ONLY_LONG_SEQUENCE:
	case LONG_START_SEQUENCE:
	case LONG_STOP_SEQUENCE:
		ctx->info.num_windows = 1;
		ctx->info.num_window_groups = 1;
		ctx->info.window_group_length[ctx->info.num_window_groups - 1] =
			1;
		for (int i = 0; i < ics_info->max_sfb + 1; i++) {
			ctx->info.sect_sfb_offset[0][i] =
				swb_offset_long_window[fs_index][i];
			ctx->info.swb_offset[i] =
				swb_offset_long_window[fs_index][i];
		}
		break;

	case EIGHT_SHORT_SEQUENCE:
		ctx->info.num_windows = 8;
		ctx->info.num_window_groups = 1;
		ctx->info.window_group_length[ctx->info.num_window_groups - 1] =
			1;
		/*int num_swb = num_swb_short_window[fs_index];*/

		for (int i = 0; i < ctx->info.num_windows - 1; i++) {
			if (bit_set(ics_info->scale_factor_grouping, 6 - i) ==
			    0) {
				ctx->info.num_window_groups += 1;
				ctx->info.window_group_length
					[ctx->info.num_window_groups - 1] = 1;
			} else {
				ctx->info.window_group_length
					[ctx->info.num_window_groups - 1] += 1;
			}
		}
		for (int g = 0; g < ctx->info.num_window_groups; g++)
			ctx->info.sect_sfb_offset[g][0] = 0;
		for (int sfb = 0; sfb < ics_info->max_sfb + 1; sfb++) {
			for (int g = 0; g < ctx->info.num_window_groups; g++) {
				ctx->info.sect_sfb_offset[g][sfb] =
					swb_offset_short_window[fs_index][sfb];
				ctx->info.sect_sfb_offset[g][sfb] *=
					ctx->info.window_group_length[g];
			}
		}
		break;

	default:
		break;
	}

	return 0;
}


/**
 * Table 1.16 – Syntax of GetAudioObjectType()
 */
static int AAC_SYNTAX_FCT(get_audioObjectType)(
	struct aac_bitstream *bs,
	AAC_SYNTAX_CONST enum aac_audioObjectType *audioObjectType)
{
	uint8_t _audioObjectType = 0;
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_WRITE
	_audioObjectType = *audioObjectType;
#endif
	AAC_BITS(_audioObjectType, 5);
	if (_audioObjectType == 31) {
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_WRITE
		_audioObjectType -= 32;
#endif
		AAC_BITS(_audioObjectType, 6);
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
		_audioObjectType += 32;
#endif
	}

#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	*audioObjectType = _audioObjectType;
#endif
	return 0;
}


/**
 * Table 4.1 – Syntax of GASpecificConfig()
 */
static int AAC_SYNTAX_FCT(GASpecificConfig)(
	struct aac_bitstream *bs,
	AAC_SYNTAX_CONST struct aac_GASpecificConfig *GASpecificConfig,
	uint8_t samplingFrequencyIndex,
	uint8_t channelConfiguration,
	uint8_t audioObjectType)
{
	AAC_BITS(GASpecificConfig->frameLengthFlag, 1);
	AAC_BITS(GASpecificConfig->dependsOnCoreCoder, 1);
	if (GASpecificConfig->dependsOnCoreCoder)
		AAC_BITS(GASpecificConfig->coreCoderDelay, 14);
	AAC_BITS(GASpecificConfig->extensionFlag, 1);
	if (!channelConfiguration) {
		/* TODO: program_config_element() */
		return -ENOSYS;
	}
	if (audioObjectType == 6 || audioObjectType == 20)
		AAC_BITS(GASpecificConfig->layerNr, 3);
	if (GASpecificConfig->extensionFlag) {
		if (audioObjectType == 22) {
			AAC_BITS(GASpecificConfig->numOfSubFrame, 5);
			AAC_BITS(GASpecificConfig->layer_length, 11);
		}
		if (audioObjectType == 17 || audioObjectType == 19 ||
		    audioObjectType == 20 || audioObjectType == 23) {
			AAC_BITS(GASpecificConfig->aacSectionDataResilienceFlag,
				 1);
			AAC_BITS(GASpecificConfig
					 ->aacScalefactorDataResilienceFlag,
				 1);
			AAC_BITS(
				GASpecificConfig->aacSpectralDataResilienceFlag,
				1);
		}
		AAC_BITS(GASpecificConfig->extensionFlag3, 1);
		/* if GASpecificConfig->extensionFlag3, tbd in version 3 */
	}
	return 0;
}


/**
 * Table 1.15 – Syntax of AudioSpecificConfig()
 */
static int AAC_SYNTAX_FCT(AudioSpecificConfig)(
	struct aac_bitstream *bs,
	AAC_SYNTAX_CONST struct aac_asc *asc)
{
	int res;

	res = AAC_SYNTAX_FCT(get_audioObjectType)(bs, &asc->audioObjectType);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	AAC_BITS(asc->samplingFrequencyIndex, 4);
	if (asc->samplingFrequencyIndex == 0xF)
		AAC_BITS(asc->samplingFrequencyIndex, 24);
	AAC_BITS(asc->channelConfiguration, 4);

	int sbrPresentFlag = -1;
	int psPresentFlag = -1;
	enum aac_audioObjectType extensionAudioObjectType = AAC_AOT_NULL;

	if (asc->audioObjectType == 5 || asc->audioObjectType == 29) {
		extensionAudioObjectType = 5;
		sbrPresentFlag = 1;
		if (asc->audioObjectType == 29)
			psPresentFlag = 1;
		AAC_BITS(asc->extensionSamplingFrequencyIndex, 4);
		if (asc->extensionSamplingFrequencyIndex == 0xF)
			AAC_BITS(asc->extensionSamplingFrequency, 24);
		res = AAC_SYNTAX_FCT(get_audioObjectType)(
			bs, &asc->audioObjectType);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		if (asc->audioObjectType == 22)
			AAC_BITS(asc->extensionChannelConfiguration, 4);
	}

	switch (asc->audioObjectType) {
	case 1:
	case 2:
	case 3:
	case 4:
	case 6:
	case 7:
	case 17:
	case 19:
	case 20:
	case 21:
	case 22:
	case 23:
		res = AAC_SYNTAX_FCT(GASpecificConfig)(
			bs,
			&asc->GASpecificConfig,
			asc->samplingFrequencyIndex,
			asc->channelConfiguration,
			asc->audioObjectType);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		break;
	case 8:
		/* CelpSpecificConfig(); */
		return -ENOSYS;
	case 9:
		/* HvxcSpecificConfig(); */
		return -ENOSYS;
	case 12:
		/* TTSSpecificConfig(); */
		return -ENOSYS;
	case 13:
	case 14:
	case 15:
	case 16:
		/* StructuredAudioSpecificConfig(); */
		return -ENOSYS;
	case 24:
		/* ErrorResilientCelpSpecificConfig(); */
		return -ENOSYS;
	case 25:
		/* ErrorResilientHvxcSpecificConfig(); */
		return -ENOSYS;
	case 26:
	case 27:
		/* ParametricSpecificConfig(); */
		return -ENOSYS;
	case 28:
		/* SSCSpecificConfig(); */
		return -ENOSYS;
	case 30:
		AAC_BITS(asc->sacPayloadEmbedding, 1);
		/* SpatialSpecificConfig(); */
		return -ENOSYS;
	case 32:
	case 33:
	case 34:
		/* MPEG_1_2_SpecificConfig(); */
		return -ENOSYS;
	case 35:
		/* DSTSpecificConfig(); */
		return -ENOSYS;
	case 36:
		AAC_BITS(asc->fillBits, 5);
		/* ALSSpecificConfig(); */
		return -ENOSYS;
	case 37:
	case 38:
		/* SLSSpecificConfig(); */
		return -ENOSYS;
	case 39:
		/* ELDSpecificConfig(channelConfiguration); */
		return -ENOSYS;
	case 40:
	case 41:
		/* SymbolicMusicSpecificConfig(); */
		return -ENOSYS;
	default:
		/* Reserved */
		break;
	}
	switch (asc->audioObjectType) {
	case 17:
	case 19:
	case 20:
	case 21:
	case 22:
	case 23:
	case 24:
	case 25:
	case 26:
	case 27:
	case 39:
		AAC_BITS(asc->epConfig, 2);
		if (asc->epConfig == 2 || asc->epConfig == 3) {
			/* ErrorProtectionSpecificConfig(); */
			return -ENOSYS;
		}
		if (asc->epConfig == 3) {
			AAC_BITS(asc->directMapping, 1);
			/* if !asc->directMapping, tbd */
		}
		break;
	default:
		break;
	}
	if (extensionAudioObjectType == 5 || aac_bs_rem_raw_bits(bs) < 16)
		return 0;
	AAC_BITS(asc->syncExtensionType, 11);
	if (asc->syncExtensionType == 0x2b7) {
		res = AAC_SYNTAX_FCT(get_audioObjectType)(
			bs, &extensionAudioObjectType);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		if (extensionAudioObjectType == 5) {
			AAC_BITS(asc->sbrPresentFlag, 1);
			if (asc->sbrPresentFlag == 1) {
				AAC_BITS(asc->extensionSamplingFrequencyIndex,
					 4);
				if (asc->extensionSamplingFrequencyIndex ==
				    0xf) {
					AAC_BITS(
						asc->extensionSamplingFrequency,
						24);
				}
				if (aac_bs_rem_raw_bits(bs) >= 12) {
					AAC_BITS(asc->syncExtensionType, 11);
					if (asc->syncExtensionType == 0x548)
						AAC_BITS(asc->psPresentFlag, 1);
				}
			}
		}
		if (extensionAudioObjectType == 22) {
			AAC_BITS(asc->sbrPresentFlag, 1);
			if (asc->sbrPresentFlag == 1) {
				AAC_BITS(asc->extensionSamplingFrequencyIndex,
					 4);
				if (asc->extensionSamplingFrequencyIndex ==
				    0xf) {
					AAC_BITS(
						asc->extensionSamplingFrequency,
						24);
				}
			}
			AAC_BITS(asc->extensionChannelConfiguration, 4);
		}
	}
	return 0;
}


/**
 * 1.A.3.2.1 Fixed Header of ADTS
 */
static int AAC_SYNTAX_FCT(adts_fixed_header)(
	struct aac_bitstream *bs,
	AAC_SYNTAX_CONST struct aac_adts *adts)
{
	AAC_BITS(adts->syncword, 12);
	ULOG_ERRNO_RETURN_ERR_IF((adts->syncword & 0xFFF) != 0xFFF, EINVAL);
	AAC_BITS(adts->ID, 1);
	AAC_BITS(adts->layer, 2);
	AAC_BITS(adts->protection_absent, 1);
	AAC_BITS(adts->profile_ObjectType, 2);
	AAC_BITS(adts->sampling_frequency_index, 4);
	AAC_BITS(adts->private_bit, 1);
	AAC_BITS(adts->channel_configuration, 3);
	AAC_BITS(adts->original_copy, 1);
	AAC_BITS(adts->home, 1);

	return 0;
}


/**
 * 1.A.3.2.2 Variable Header of ADTS
 */
static int AAC_SYNTAX_FCT(adts_variable_header)(
	struct aac_bitstream *bs,
	AAC_SYNTAX_CONST struct aac_adts *adts)
{
	AAC_BITS(adts->copyright_identification_bit, 1);
	AAC_BITS(adts->copyright_identification_start, 1);
	AAC_BITS(adts->aac_frame_length, 13);
	AAC_BITS(adts->adts_buffer_fullness, 11);
	AAC_BITS(adts->number_of_raw_data_blocks_in_frame, 2);

	return 0;
}


/**
 * Table 1.A.8 – Syntax of adts_error_check
 */
static int AAC_SYNTAX_FCT(adts_error_check)(struct aac_bitstream *bs,
					    struct aac_ctx *ctx)
{
	uint32_t crc_check = 0;

	if (ctx->adts.protection_absent == 0)
		AAC_BITS(crc_check, 16);

	return 0;
}


/**
 * Table 1.A.9 – Syntax of adts_header_error_check
 */
static int AAC_SYNTAX_FCT(adts_header_error_check)(struct aac_bitstream *bs,
						   struct aac_ctx *ctx)
{
	uint32_t crc_check = 0;
	uint32_t raw_data_block_position[4] = {0};

	if (ctx->adts.protection_absent == 0) {
		int i;
		for (i = 0; i < ctx->adts.number_of_raw_data_blocks_in_frame;
		     i++) {
			AAC_BITS(raw_data_block_position[i], 16);
		}
		AAC_BITS(crc_check, 16);
	}

	return 0;
}


/**
 * Table 1.A.10 – Syntax of adts_raw_data_block_error_check
 */
static int AAC_SYNTAX_FCT(adts_raw_data_block_error_check)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx)
{
	uint32_t crc_check = 0;

	if (ctx->adts.protection_absent == 0)
		AAC_BITS(crc_check, 16);

	return 0;
}


#define PRED_SFB_MAX 40


/**
 * Table 4.6 – Syntax of ics_info()
 */
static int AAC_SYNTAX_FCT(ics_info)(struct aac_bitstream *bs,
				    struct aac_ctx *ctx,
				    struct aac_ics_info *ics_info,
				    int common_window)
{
#if AAC_SYNTAX_OP_KIND != AAC_SYNTAX_OP_KIND_WRITE
	memset(ics_info, 0, sizeof(*ics_info));
#endif

	int audioObjectType =
		ctx->adts.profile_ObjectType; /* TODO: check minus 1? */
	AAC_BITS(ics_info->ics_reserved_bit, 1);
	AAC_BITS(ics_info->window_sequence, 2);
	AAC_BITS(ics_info->window_shape, 1);
	if (ics_info->window_sequence == EIGHT_SHORT_SEQUENCE) {
		AAC_BITS(ics_info->max_sfb, 4);
		AAC_BITS(ics_info->scale_factor_grouping, 7);
		return 0;
	}
	AAC_BITS(ics_info->max_sfb, 6);
	AAC_BITS(ics_info->predictor_data_present, 1);
	if (ics_info->predictor_data_present) {
		if (audioObjectType == 1) {
			AAC_BITS(ics_info->predictor_reset, 1);
			if (ics_info->predictor_reset) {
				AAC_BITS(ics_info->predictor_reset_group_number,
					 5);
			}
			for (int sfb = 0;
			     sfb < Min(ics_info->max_sfb, PRED_SFB_MAX);
			     sfb++) {
				AAC_BITS(ics_info->prediction_used[sfb], 1);
			}
		} else {
			AAC_BITS(ics_info->ltp_data_present, 1);
			if (ics_info->ltp_data_present) {
				/* ltp_data(); */
				return -ENOSYS;
			}
			if (common_window) {
				AAC_BITS(ics_info->ltp_data_present, 1);
				if (ics_info->ltp_data_present) {
					/* ltp_data(); */
					return -ENOSYS;
				}
			}
		}
	}
	return 0;
}


/**
 * Table 4.52 – Syntax of section_data()
 */
static int
	AAC_SYNTAX_FCT(section_data)(struct aac_bitstream *bs,
				     struct aac_ctx *ctx,
				     struct aac_individual_channel_stream *ics,
				     struct aac_section_data *section_data)
{
	int sect_esc_val;
	int sect_bits;
	if (ics->ics_info.window_sequence == EIGHT_SHORT_SEQUENCE) {
		sect_esc_val = (1 << 3) - 1;
		sect_bits = 3;
	} else {
		sect_esc_val = (1 << 5) - 1;
		sect_bits = 5;
	}
	for (int g = 0; g < ctx->info.num_window_groups; g++) {
		int k = 0;
		int i = 0;
		while (k < ics->ics_info.max_sfb) {
			if (has_aacSectionDataResilienceFlag(ctx))
				AAC_BITS(section_data->sect_cb[g][i], 5);
			else
				AAC_BITS(section_data->sect_cb[g][i], 4);
			int sect_len = 0;
			int sect_len_incr = 0;
			if (!has_aacSectionDataResilienceFlag(ctx) ||
			    section_data->sect_cb[g][i] < 11 ||
			    (section_data->sect_cb[g][i] > 11 &&
			     section_data->sect_cb[g][i] < 16)) {
				AAC_BITS(sect_len_incr, sect_bits);
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
				while (sect_len_incr == sect_esc_val) {
					sect_len += sect_esc_val;
					AAC_BITS(sect_len_incr, sect_bits);
				}
#endif
			} else {
				sect_len_incr = 1;
			}
			sect_len += sect_len_incr;
			section_data->sect_start[g][i] = k;
			section_data->sect_end[g][i] = k + sect_len;
			for (int sfb = k; sfb < k + sect_len; sfb++) {
				section_data->sfb_cb[g][sfb] =
					section_data->sect_cb[g][i];
			}
			k += sect_len;
			i++;
		}
		section_data->num_sec[g] = i;
	}
	return 0;
}


/**
 * Table 4.52 – Syntax of scale_factor_data()
 */
static int AAC_SYNTAX_FCT(scale_factor_data)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_individual_channel_stream *ics,
	struct aac_scale_factor_data *scale_factor_data)
{
	int res;
	int noise_pcm_flag;

	if (has_aacScalefactorDataResilienceFlag(ctx))
		return -ENOSYS;

	noise_pcm_flag = 1;
	for (int g = 0; g < ctx->info.num_window_groups; g++) {
		for (int sfb = 0; sfb < ics->ics_info.max_sfb; sfb++) {
			if (ics->section_data.sfb_cb[g][sfb] == ZERO_HCB)
				continue;
			if (is_intensity(ics->section_data.sfb_cb[g][sfb])) {
				res = huffman_decode_scale_factor(bs);
				ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
				scale_factor_data->dpcm_is_position[g][sfb] =
					res;
			} else if (is_noise(ics->section_data.sfb_cb[g][sfb])) {
				if (noise_pcm_flag) {
					noise_pcm_flag = 0;
					AAC_BITS(scale_factor_data
							 ->dpcm_noise_nrg[g]
									 [sfb],
						 9);
				} else {
					res = huffman_decode_scale_factor(bs);
					ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
					scale_factor_data
						->dpcm_noise_nrg[g][sfb] = res;
				}
			} else {
				res = huffman_decode_scale_factor(bs);
				ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
				scale_factor_data->dpcm_sf[g][sfb] = res;
			}
		}
	}
	return 0;
}


/**
 * Table 4.7 – Syntax of pulse_data()
 */
static int AAC_SYNTAX_FCT(pulse_data)(struct aac_bitstream *bs,
				      struct aac_ctx *ctx,
				      struct aac_individual_channel_stream *ics,
				      struct aac_pulse_data *pulse_data)
{
	AAC_BITS(pulse_data->number_pulse, 2);
	AAC_BITS(pulse_data->pulse_start_sfb, 6);
	for (int i = 0; i < pulse_data->number_pulse + 1; i++) {
		AAC_BITS(pulse_data->pulse_offset[i], 5);
		AAC_BITS(pulse_data->pulse_amp[i], 4);
	}
	return 0;
}


/**
 * Table 4.54 – Syntax of tns_data()
 */
static int AAC_SYNTAX_FCT(tns_data)(struct aac_bitstream *bs,
				    struct aac_ctx *ctx,
				    struct aac_individual_channel_stream *ics,
				    struct aac_tns_data *tns_data)
{
	int n_filt_bits = 2;
	int length_bits = 6;
	int order_bits = 5;
	int sum_read = 0;
	if (ics->ics_info.window_sequence == EIGHT_SHORT_SEQUENCE) {
		n_filt_bits = 1;
		length_bits = 4;
		order_bits = 3;
	}

	for (int w = 0; w < ctx->info.num_windows; w++) {
		int start_coef_bits = 3;
		AAC_BITS(tns_data->n_filt[w], n_filt_bits);
		sum_read += n_filt_bits;
		if (tns_data->n_filt[w]) {
			AAC_BITS(tns_data->coef_res[w], 1);
			start_coef_bits += tns_data->coef_res[w];
			sum_read += 1;
		}
		for (int filt = 0; filt < tns_data->n_filt[w]; filt++) {
			AAC_BITS(tns_data->length[w][filt], length_bits);
			sum_read += length_bits;
			AAC_BITS(tns_data->order[w][filt], order_bits);
			sum_read += order_bits;
			if (tns_data->order[w][filt]) {
				AAC_BITS(tns_data->direction[w][filt], 1);
				AAC_BITS(tns_data->coef_compress[w][filt], 1);
				sum_read += 2;
				int coef_bits =
					start_coef_bits -
					tns_data->coef_compress[w][filt];
				for (int i = 0; i < tns_data->order[w][filt];
				     i++) {
					AAC_BITS(tns_data->coef[w][filt][i],
						 coef_bits);
					sum_read += coef_bits;
				}
			}
		}
	}
	return 0;
}


/**
 * Table 4.54 – Syntax of tns_data()
 */
static int AAC_SYNTAX_FCT(gain_control_data)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_individual_channel_stream *ics,
	struct aac_gain_control_data *gain_control_data)
{
	AAC_BITS(gain_control_data->max_band, 2);
	if (ics->ics_info.window_sequence == ONLY_LONG_SEQUENCE) {
		for (int bd = 1; bd < gain_control_data->max_band; bd++) {
			for (int wd = 0; wd < 1; wd++) {
				AAC_BITS(gain_control_data->adjust_num[bd][wd],
					 3);
				for (int ad = 0;
				     ad < gain_control_data->adjust_num[bd][wd];
				     ad++) {
					AAC_BITS(gain_control_data
							 ->alevcode[bd][wd][ad],
						 4);
					AAC_BITS(gain_control_data
							 ->aloccode[bd][wd][ad],
						 5);
				}
			}
		}
	} else if (ics->ics_info.window_sequence == LONG_START_SEQUENCE) {
		for (int bd = 1; bd < gain_control_data->max_band; bd++) {
			for (int wd = 0; wd < 2; wd++) {
				AAC_BITS(gain_control_data->adjust_num[bd][wd],
					 3);
				for (int ad = 0;
				     ad < gain_control_data->adjust_num[bd][wd];
				     ad++) {
					AAC_BITS(gain_control_data
							 ->alevcode[bd][wd][ad],
						 4);
					if (wd == 0) {
						AAC_BITS(gain_control_data
								 ->aloccode[bd]
									   [wd]
									   [ad],
							 4);
					} else {
						AAC_BITS(gain_control_data
								 ->aloccode[bd]
									   [wd]
									   [ad],
							 2);
					}
				}
			}
		}
	} else if (ics->ics_info.window_sequence == EIGHT_SHORT_SEQUENCE) {
		for (int bd = 1; bd < gain_control_data->max_band; bd++) {
			for (int wd = 0; wd < 8; wd++) {
				AAC_BITS(gain_control_data->adjust_num[bd][wd],
					 3);
				for (int ad = 0;
				     ad < gain_control_data->adjust_num[bd][wd];
				     ad++) {
					AAC_BITS(gain_control_data
							 ->alevcode[bd][wd][ad],
						 4);
					AAC_BITS(gain_control_data
							 ->aloccode[bd][wd][ad],
						 2);
				}
			}
		}
	} else if (ics->ics_info.window_sequence == LONG_STOP_SEQUENCE) {
		for (int bd = 1; bd < gain_control_data->max_band; bd++) {
			for (int wd = 0; wd < 2; wd++) {
				AAC_BITS(gain_control_data->adjust_num[bd][wd],
					 3);
				for (int ad = 0;
				     ad < gain_control_data->adjust_num[bd][wd];
				     ad++) {
					AAC_BITS(gain_control_data
							 ->alevcode[bd][wd][ad],
						 4);
					if (wd == 0) {
						AAC_BITS(gain_control_data
								 ->aloccode[bd]
									   [wd]
									   [ad],
							 4);
					} else {
						AAC_BITS(gain_control_data
								 ->aloccode[bd]
									   [wd]
									   [ad],
							 5);
					}
				}
			}
		}
	}
	return 0;
}


#define QUAD_LEN 4
#define PAIR_LEN 2


/**
 * Table 4.56 – Syntax of spectral_data()
 */
static int
	AAC_SYNTAX_FCT(spectral_data)(struct aac_bitstream *bs,
				      struct aac_ctx *ctx,
				      struct aac_individual_channel_stream *ics,
				      struct aac_spectral_data *spectral_data)
{
	int res;
	for (int g = 0; g < ctx->info.num_window_groups; g++) {
		for (int i = 0; i < ics->section_data.num_sec[g]; i++) {
			if (ics->section_data.sect_cb[g][i] == ZERO_HCB ||
			    ics->section_data.sect_cb[g][i] == NOISE_HCB ||
			    ics->section_data.sect_cb[g][i] == INTENSITY_HCB ||
			    ics->section_data.sect_cb[g][i] == INTENSITY_HCB2)
				continue;
			for (int k = ctx->info.sect_sfb_offset
					     [g][ics->section_data
							 .sect_start[g][i]];
			     k <
			     ctx->info.sect_sfb_offset
				     [g][ics->section_data.sect_end[g][i]];) {
				int w, x, y, z;
				if (ics->section_data.sect_cb[g][i] <
				    FIRST_PAIR_HCB) {
					res = huffman_decode_spectral_data(
						bs,
						ics->section_data.sect_cb[g][i],
						&w,
						&x,
						&y,
						&z);
					ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
					k += QUAD_LEN;
					continue;
				}
				/* else */
				res = huffman_decode_spectral_data(
					bs,
					ics->section_data.sect_cb[g][i],
					NULL,
					NULL,
					&y,
					&z);
				ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
				k += PAIR_LEN;
				if (ics->section_data.sect_cb[g][i] ==
				    ESC_HCB) {
					if (Abs(y) == ESC_FLAG)
						y = get_escape(bs, y);
					if (Abs(z) == ESC_FLAG)
						z = get_escape(bs, z);
				}
			}
		}
	}
	return 0;
}


/**
 * Table 4.50 – Syntax of individual_channel_stream()
 */
static int AAC_SYNTAX_FCT(individual_channel_stream)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_individual_channel_stream *ics,
	int common_window,
	int scale_flag)
{
	int res;
	AAC_BITS(ics->global_gain, 8);
	if (!common_window && !scale_flag) {
		res = AAC_SYNTAX_FCT(ics_info)(
			bs, ctx, &ics->ics_info, common_window);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		res = set_dec_info(ctx, &ics->ics_info);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	}
	LOG_OFFSET("section_data")
	res = AAC_SYNTAX_FCT(section_data)(bs, ctx, ics, &ics->section_data);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	LOG_OFFSET("scale_factor_data");
	res = AAC_SYNTAX_FCT(scale_factor_data)(
		bs, ctx, ics, &ics->scale_factor_data);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	if (!scale_flag) {
		AAC_BITS(ics->pulse_data_present, 1);
		if (ics->pulse_data_present) {
			LOG_OFFSET("pulse_data");
			res = AAC_SYNTAX_FCT(pulse_data)(
				bs, ctx, ics, &ics->pulse_data);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
		AAC_BITS(ics->tns_data_present, 1);
		if (ics->tns_data_present) {
			LOG_OFFSET("tns_data");
			res = AAC_SYNTAX_FCT(tns_data)(
				bs, ctx, ics, &ics->tns_data);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
		AAC_BITS(ics->gain_control_data_present, 1);
		if (ics->gain_control_data_present) {
			LOG_OFFSET("gain_control_data");
			res = AAC_SYNTAX_FCT(gain_control_data)(
				bs, ctx, ics, &ics->gain_control_data);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		}
	}
	if (!has_aacSpectralDataResilienceFlag(ctx)) {
		LOG_OFFSET("spectral_data");
		res = AAC_SYNTAX_FCT(spectral_data)(
			bs, ctx, ics, &ics->spectral_data);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	} else {
		AAC_BITS(ics->length_of_reordered_spectral_data, 14);
		AAC_BITS(ics->length_of_longest_codeword, 6);
		/* reordered_spectral_data()*/
		ULOGE("reordered_spectral_data");
		return -ENOSYS;
	}
	return 0;
}


/**
 * Table 4.4 – Syntax of single_channel_element()
 */
static int AAC_SYNTAX_FCT(single_channel_element)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_single_channel_element *sce)
{
	int res;
	AAC_BITS(sce->element_instance_tag, 4);
	AAC_BEGIN_STRUCT(individual_channel_stream);
	res = AAC_SYNTAX_FCT(individual_channel_stream)(
		bs, ctx, &sce->ics, 0, 0);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	AAC_END_STRUCT(individual_channel_stream);

	return 0;
}


/**
 * Table 4.5 – Syntax of channel_pair_element()
 */
static int AAC_SYNTAX_FCT(channel_pair_element)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_channel_pair_element *cpe)
{
	int res;
	AAC_BITS(cpe->element_instance_tag, 4);
	AAC_BITS(cpe->common_window, 1);
	if (cpe->common_window) {
		res = AAC_SYNTAX_FCT(ics_info)(
			bs, ctx, &cpe->ics_info, cpe->common_window);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		res = set_dec_info(ctx, &cpe->ics_info);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		cpe->ics1.ics_info = cpe->ics_info;
		cpe->ics2.ics_info = cpe->ics_info;
		AAC_BITS(cpe->ms_mask_present, 2);
		if (cpe->ms_mask_present == 1) {
			for (int g = 0; g < ctx->info.num_window_groups; g++) {
				for (int sfb = 0; sfb < cpe->ics_info.max_sfb;
				     sfb++) {
					AAC_BITS(cpe->ms_used[g][sfb], 1);
				}
			}
		}
	}

	AAC_BEGIN_ARRAY(individual_channel_stream);
	AAC_BEGIN_ARRAY_ITEM();
	res = AAC_SYNTAX_FCT(individual_channel_stream)(
		bs, ctx, &cpe->ics1, cpe->common_window, 0);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	AAC_END_ARRAY_ITEM();
	AAC_BEGIN_ARRAY_ITEM();
	res = AAC_SYNTAX_FCT(individual_channel_stream)(
		bs, ctx, &cpe->ics2, cpe->common_window, 0);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	AAC_END_ARRAY_ITEM();
	AAC_END_ARRAY(individual_channel_stream);

	return 0;
}


/**
 * Table 4.8 – Syntax of coupling_channel_element()
 */
static int AAC_SYNTAX_FCT(coupling_channel_element)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_coupling_channel_element *cce)
{
	int res;
	AAC_BITS(cce->element_instance_tag, 4);
	AAC_BITS(cce->ind_sw_cce_flag, 1);
	AAC_BITS(cce->num_coupled_element, 3);
	int num_gain_element_lists = 0;
	for (int c = 0; c < cce->num_coupled_element + 1; c++) {
		num_gain_element_lists++;
		AAC_BITS(cce->cc_target_is_cpe[c], 1);
		AAC_BITS(cce->cc_target_tag_select[c], 4);
		if (cce->cc_target_is_cpe[c]) {
			AAC_BITS(cce->cc_l[c], 1);
			AAC_BITS(cce->cc_r[c], 1);
			if (cce->cc_l[c] && cce->cc_r[c])
				num_gain_element_lists++;
		}
	}
	AAC_BITS(cce->cc_domain, 1);
	AAC_BITS(cce->gain_element_sign, 1);
	AAC_BITS(cce->gain_element_scale, 2);

	res = AAC_SYNTAX_FCT(individual_channel_stream)(
		bs, ctx, &cce->ics, 0, 0);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);

	int cge = 0;
	for (int c = 1; c < num_gain_element_lists; c++) {
		if (cce->ind_sw_cce_flag) {
			cge = 1;
		} else {
			AAC_BITS(cce->common_gain_element_present[c], 1);
			cge = cce->common_gain_element_present[c];
		}
		if (cge) {
			res = huffman_decode_scale_factor(bs);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			cce->common_gain_element[c] = res;
			continue;
		}
		/* else */
		for (int g = 0; g < ctx->info.num_window_groups; g++) {
			for (int sfb = 0; sfb < cce->ics.ics_info.max_sfb;
			     sfb++) {
				if (cce->ics.section_data.sfb_cb[g][sfb] !=
				    ZERO_HCB) {
					res = huffman_decode_scale_factor(bs);
					ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
					cce->dpcm_gain_element[c][g][sfb] = res;
				}
			}
		}
	}
	return 0;
}


/**
 * Table 4.10 – Syntax of data_stream_element()
 */
static int
	AAC_SYNTAX_FCT(data_stream_element)(struct aac_bitstream *bs,
					    struct aac_ctx *ctx,
					    struct aac_data_stream_element *dse)
{
	AAC_BITS(dse->element_instance_tag, 4);
	AAC_BITS(dse->data_byte_align_flag, 1);
	AAC_BITS(dse->count, 8);
	int cnt = dse->count;
	if (dse->count == 255) {
		AAC_BITS(dse->esc_count, 8);
		cnt += dse->esc_count;
	}
	for (int i = 0; i < cnt; i++) {
		uint8_t data_stream_byte = 0;
		/* data_stream_byte[element_instance_tag][i]; (8) */
		AAC_BITS(data_stream_byte, 8);
	}

	return 0;
}


/**
 * Table 4.10 – Syntax of program_config_element()
 */
static int AAC_SYNTAX_FCT(program_config_element)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_program_config_element *pce)
{
	AAC_BITS(pce->element_instance_tag, 4);
	AAC_BITS(pce->object_type, 2);
	AAC_BITS(pce->sampling_frequency_index, 4);
	AAC_BITS(pce->num_front_channel_elements, 4);
	AAC_BITS(pce->num_side_channel_elements, 4);
	AAC_BITS(pce->num_back_channel_elements, 4);
	AAC_BITS(pce->num_lfe_channel_elements, 2);
	AAC_BITS(pce->num_assoc_data_elements, 3);
	AAC_BITS(pce->num_valid_cc_elements, 4);
	AAC_BITS(pce->mono_mixdown_present, 1);
	if (pce->mono_mixdown_present)
		AAC_BITS(pce->mono_mixdown_element_number, 4);

	AAC_BITS(pce->stereo_mixdown_present, 1);
	if (pce->stereo_mixdown_present)
		AAC_BITS(pce->stereo_mixdown_element_number, 4);

	AAC_BITS(pce->matrix_mixdown_idx_present, 1);
	if (pce->matrix_mixdown_idx_present) {
		AAC_BITS(pce->matrix_mixdown_idx, 2);
		AAC_BITS(pce->pseudo_surround_enable, 1);
	}

	for (int i = 0; i < pce->num_front_channel_elements; i++) {
		AAC_BITS(pce->front_element_is_cpe[i], 1);
		AAC_BITS(pce->front_element_tag_select[i], 4);
	}

	for (int i = 0; i < pce->num_side_channel_elements; i++) {
		AAC_BITS(pce->side_element_is_cpe[i], 1);
		AAC_BITS(pce->side_element_tag_select[i], 4);
	}

	for (int i = 0; i < pce->num_back_channel_elements; i++) {
		AAC_BITS(pce->back_element_is_cpe[i], 1);
		AAC_BITS(pce->back_element_tag_select[i], 4);
	}

	for (int i = 0; i < pce->num_lfe_channel_elements; i++)
		AAC_BITS(pce->lfe_element_tag_select[i], 4);

	for (int i = 0; i < pce->num_assoc_data_elements; i++)
		AAC_BITS(pce->assoc_data_element_tag_select[i], 4);

	for (int i = 0; i < pce->num_valid_cc_elements; i++) {
		AAC_BITS(pce->cc_element_is_ind_sw[i], 1);
		AAC_BITS(pce->valid_cc_element_tag_select[i], 4);
	}

#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	aac_bs_read_trailing_bits(bs);
#endif

	AAC_BITS(pce->comment_field_bytes, 8);
	for (int i = 0; i < pce->comment_field_bytes; i++)
		AAC_BITS(pce->comment_field_data[i], 8);

	return 0;
}


/**
 * Table 4.57 – Syntax of extension_payload()
 */
static int AAC_SYNTAX_FCT(extension_payload)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_extension_payload *extension_payload,
	int count)
{
	AAC_BITS(extension_payload->extension_type, 4);
	uint8_t align = 4;
	uint8_t other_bits = 0;
	uint8_t fill_nibble = 0;
	uint8_t fill_byte = 0;
	switch (extension_payload->extension_type) {
	case AAC_EXT_TYPE_FILL_DATA:
		ULOGD("AAC_EXT_TYPE_FILL_DATA");
		AAC_BITS(fill_nibble, 4); /* must be '0000' */
		ULOG_ERRNO_RETURN_ERR_IF(fill_nibble != 0, EINVAL);
		for (int i = 0; i < count - 1; i++) {
			AAC_BITS(fill_byte, 8); /* must be '10100101' */
			ULOG_ERRNO_RETURN_ERR_IF(fill_byte != 0xA5, EINVAL);
		}
		return count;

	case AAC_EXT_DATA_ELEMENT:
		ULOGD("AAC_EXT_DATA_ELEMENT");
		return -ENOSYS;

	case AAC_EXT_DYNAMIC_RANGE:
		ULOGD("AAC_EXT_DYNAMIC_RANGE");
		return -ENOSYS;

	case AAC_EXT_SAC_DATA:
		ULOGD("AAC_EXT_SAC_DATA");
		return -ENOSYS;

	case AAC_EXT_SBR_DATA:
		ULOGD("AAC_EXT_SBR_DATA");
		return -ENOSYS;

	case AAC_EXT_SBR_DATA_CRC:
		ULOGD("AAC_EXT_SBR_DATA_CRC");
		return -ENOSYS;

	case AAC_EXT_TYPE_FILL:
	default:
		for (int i = 0; i < 8 * (count - 1) + align; i++)
			AAC_BITS(other_bits, 1);
		return count;
	}
	return 0;
}


/**
 * Table 4.11 – Syntax of fill_element()
 */
static int AAC_SYNTAX_FCT(fill_element)(struct aac_bitstream *bs,
					struct aac_ctx *ctx,
					struct aac_fill_element *fil)
{
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_DUMP
	AAC_BITS(fil->count, 0);
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	int res;
	int cnt = 0, esc_count = 0;
	AAC_BITS(cnt, 4);
	if (cnt == 15) {
		AAC_BITS(esc_count, 8);
		cnt += esc_count - 1;
	}
	fil->count = cnt;
	while (cnt > 0) {
		res = AAC_SYNTAX_FCT(extension_payload)(
			bs, ctx, &fil->extension_payload, cnt);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
		cnt -= res;
	}
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_WRITE
	uint8_t count = 0;
	uint8_t esc_count = 0;
	if (fil->count >= 15) {
		count = 0xF;
		esc_count = (fil->count - 14);
	} else {
		count = fil->count;
	}
	AAC_BITS(count, 4);
	if (esc_count != 0)
		AAC_BITS(esc_count, 8);
	/* Fill with zero */
	size_t i;
	for (i = 0; i < fil->count; i++)
		aac_bs_write_bits(bs, 0, 8);
#endif
	return 0;
}


/**
 * Table 4.3 – Syntax of top level payload for audio object types AAC Main, SSR,
 * LC, and LTP
 */
static int AAC_SYNTAX_FCT(raw_data_block)(
	struct aac_bitstream *bs,
	struct aac_ctx *ctx,
	struct aac_raw_data_block *raw_data_block)
{
	int res;
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	memset(raw_data_block, 0, sizeof(*raw_data_block));
	while (raw_data_block->elements_count < AAC_MAX_SYN_ELE) {
		size_t i = raw_data_block->elements_count;
#else
	for (size_t i = 0; i < raw_data_block->elements_count; i++) {
#endif
		struct aac_syntactic_element *element =
			&raw_data_block->elements[i];
		AAC_BITS(element->id_syn_ele, 3);
		switch (element->id_syn_ele) {
		case AAC_SYN_ELE_ID_SCE:
			ULOGD("AAC_SYN_ELE_ID_SCE");
			AAC_BEGIN_STRUCT(aac_single_channel_element);
			res = AAC_SYNTAX_FCT(single_channel_element)(
				bs, ctx, &element->sce);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(aac_single_channel_element);
			raw_data_block->elements_count++;
			break;

		case AAC_SYN_ELE_ID_CPE:
			ULOGD("AAC_SYN_ELE_ID_CPE");
			AAC_BEGIN_STRUCT(channel_pair_element);
			res = AAC_SYNTAX_FCT(channel_pair_element)(
				bs, ctx, &element->cpe);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(channel_pair_element);
			raw_data_block->elements_count++;
			break;

		case AAC_SYN_ELE_ID_CCE:
			ULOGD("AAC_SYN_ELE_ID_CCE");
			AAC_BEGIN_STRUCT(coupling_channel_element);
			res = AAC_SYNTAX_FCT(coupling_channel_element)(
				bs, ctx, &element->cce);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(coupling_channel_element);
			raw_data_block->elements_count++;
			break;

		case AAC_SYN_ELE_ID_LFE:
			ULOGD("AAC_SYN_ELE_ID_LFE");
			return -ENOSYS;

		case AAC_SYN_ELE_ID_DSE:
			ULOGD("AAC_SYN_ELE_ID_DSE");
			AAC_BEGIN_STRUCT(data_stream_element);
			res = AAC_SYNTAX_FCT(data_stream_element)(
				bs, ctx, &element->dse);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(data_stream_element);
			raw_data_block->elements_count++;
			break;

		case AAC_SYN_ELE_ID_PCE:
			ULOGD("AAC_SYN_ELE_ID_PCE");
			AAC_BEGIN_STRUCT(program_config_element);
			res = AAC_SYNTAX_FCT(program_config_element)(
				bs, ctx, &element->pce);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(program_config_element);
			raw_data_block->elements_count++;
			break;

		case AAC_SYN_ELE_ID_FIL:
			ULOGD("AAC_SYN_ELE_ID_FIL");
			AAC_BEGIN_STRUCT(fill_element);
			res = AAC_SYNTAX_FCT(fill_element)(
				bs, ctx, &element->fil);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(fill_element);
			raw_data_block->elements_count++;
			break;

		case AAC_SYN_ELE_ID_END:
			ULOGD("AAC_SYN_ELE_ID_END");
			goto padding;

		default:
			ULOGE("unsupported code: %d", element->id_syn_ele);
			return -EINVAL;
		}
	}
padding:
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	res = aac_bs_read_trailing_bits(bs);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_WRITE
	res = aac_bs_write_trailing_bits(bs);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#endif

	return 0;
}


/**
 * Table 1.A.5 – Syntax of adts_frame()
 */
static int AAC_SYNTAX_FCT(adts_frame)(struct aac_bitstream *bs,
				      struct aac_ctx *ctx,
				      const struct aac_ctx_cbs *cbs,
				      void *userdata)
{
	int res = 0;
	const uint8_t *buf = NULL;
	size_t len = 0;
	size_t start_off = bs->off;
	size_t end_off = 0;

#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
	buf = bs->cdata + bs->off;
	len = bs->len;
	res = aac_ctx_clear_adts(ctx);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#endif

	AAC_BEGIN_STRUCT(aac_adts);
	res = AAC_SYNTAX_FCT(adts_fixed_header)(bs, &ctx->adts);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	res = AAC_SYNTAX_FCT(adts_variable_header)(bs, &ctx->adts);
	ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
	AAC_END_STRUCT(aac_adts);

	end_off = start_off + ctx->adts.aac_frame_length;
	AAC_CB(ctx,
	       cbs,
	       userdata,
	       adts_frame_begin,
	       buf,
	       ctx->adts.aac_frame_length,
	       &ctx->adts);

	if (ctx->adts.number_of_raw_data_blocks_in_frame == 0) {
		res = AAC_SYNTAX_FCT(adts_error_check)(bs, ctx);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
		if ((AAC_READ_FLAGS() & AAC_READER_FLAGS_FRAME_DATA) != 0) {
			AAC_BEGIN_STRUCT(raw_data_block);
			res = AAC_SYNTAX_FCT(raw_data_block)(
				bs, ctx, &ctx->adts_frame.raw_data_block[0]);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(raw_data_block);
		} else {
			/* Pad to the next byte */
			while (!aac_bs_byte_aligned(bs)) {
				uint8_t read;
				AAC_BITS(read, 1);
			}
			/* Read up to next frame */
			while (end_off > bs->off) {
				uint8_t read;
				AAC_BITS(read, 8);
			}
		}
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_DUMP
		if ((AAC_DUMP_FLAGS() & AAC_DUMP_FLAGS_FRAME_DATA) != 0) {
			AAC_BEGIN_STRUCT(raw_data_block);
			res = AAC_SYNTAX_FCT(raw_data_block)(
				bs, ctx, &ctx->adts_frame.raw_data_block[0]);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			AAC_END_STRUCT(raw_data_block);
		}
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_WRITE
		res = AAC_SYNTAX_FCT(raw_data_block)(
			bs, ctx, &ctx->adts_frame.raw_data_block[0]);
		ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#else
#	error "Unsupported AAC_SYNTAX_OP_KIND"
#endif
		AAC_CB(ctx,
		       cbs,
		       userdata,
		       adts_frame_end,
		       buf,
		       ctx->adts.aac_frame_length,
		       &ctx->adts);
		return 0;
	} else {
		res = AAC_SYNTAX_FCT(adts_error_check)(bs, ctx);
		for (int i = 0;
		     i <= ctx->adts.number_of_raw_data_blocks_in_frame;
		     i++) {
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
			if ((AAC_READ_FLAGS() & AAC_READER_FLAGS_FRAME_DATA) !=
			    0) {
				res = AAC_SYNTAX_FCT(raw_data_block)(
					bs,
					ctx,
					&ctx->adts_frame.raw_data_block[i]);
				ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			} else {
				/* Pad to the next byte */
				while (!aac_bs_byte_aligned(bs)) {
					uint8_t read;
					AAC_BITS(read, 1);
				}
				/* Read up to next frame */
				while (end_off > bs->off) {
					uint8_t read;
					AAC_BITS(read, 8);
				}
			}
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_DUMP
			if ((AAC_DUMP_FLAGS() & AAC_DUMP_FLAGS_FRAME_DATA) !=
			    0) {
				res = AAC_SYNTAX_FCT(raw_data_block)(
					bs,
					ctx,
					&ctx->adts_frame.raw_data_block[i]);
				ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
			}
#elif AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_WRITE
			res = AAC_SYNTAX_FCT(raw_data_block)(
				bs, ctx, &ctx->adts_frame.raw_data_block[i]);
			ULOG_ERRNO_RETURN_ERR_IF(res < 0, -res);
#else
#	error "Unsupported AAC_SYNTAX_OP_KIND"
#endif
		}
		AAC_CB(ctx,
		       cbs,
		       userdata,
		       adts_frame_end,
		       buf,
		       len,
		       &ctx->adts);
	}
	return 0;
}


#endif /* _AAC_SYNTAX_H_ */
