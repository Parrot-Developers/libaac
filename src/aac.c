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


ULOG_DECLARE_TAG(aac);


int aac_adts_to_adef_format(const struct aac_adts *adts,
			    struct adef_format *format)
{
	ULOG_ERRNO_RETURN_ERR_IF(adts == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(adts->profile_ObjectType + 1 != AAC_AOT_AAC_LC,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		adts->sampling_frequency_index >
			(int)ARRAY_SIZE(sampling_frequency_table),
		EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		adts->channel_configuration >
			(int)ARRAY_SIZE(channel_configuration_table),
		EINVAL);

	format->encoding = ADEF_ENCODING_AAC_LC;
	format->channel_count =
		channel_configuration_table[adts->channel_configuration];
	format->bit_depth = 16; /* TODO? */
	format->sample_rate =
		sampling_frequency_table[adts->sampling_frequency_index];
	format->aac.data_format = ADEF_AAC_DATA_FORMAT_ADTS;

	return 0;
}


int aac_adts_from_adef_format(const struct adef_format *format,
			      struct aac_adts *adts)
{
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(adts == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!adef_is_format_valid(format), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		format->aac.data_format != ADEF_AAC_DATA_FORMAT_ADTS, EINVAL);

	adts->syncword = 0xFFF;
	adts->ID = 0;
	adts->layer = 0;
	adts->protection_absent = 1;
	adts->profile_ObjectType = AAC_AOT_AAC_LC - 1;

	for (size_t i = 0; i < ARRAY_SIZE(sampling_frequency_table); i++) {
		if (sampling_frequency_table[i] == format->sample_rate) {
			adts->sampling_frequency_index = i;
			break;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(channel_configuration_table); i++) {
		if (channel_configuration_table[i] == format->channel_count) {
			adts->channel_configuration = i;
			break;
		}
	}

	adts->original_copy = 0;
	adts->home = 0;
	adts->copyright_identification_bit = 0;
	adts->copyright_identification_start = 0;
	adts->aac_frame_length = 7; /* + payload */
	adts->adts_buffer_fullness = 0x7FF; /* VBR */
	adts->number_of_raw_data_blocks_in_frame = 0;

	return 0;
}


int aac_asc_to_adef_format(const struct aac_asc *asc,
			   struct adef_format *format)
{
	ULOG_ERRNO_RETURN_ERR_IF(asc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(asc->audioObjectType != AAC_AOT_AAC_LC,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		asc->samplingFrequencyIndex >
			(int)ARRAY_SIZE(sampling_frequency_table),
		EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		asc->channelConfiguration >
			(int)ARRAY_SIZE(channel_configuration_table),
		EINVAL);

	format->encoding = ADEF_ENCODING_AAC_LC;
	format->channel_count =
		channel_configuration_table[asc->channelConfiguration];
	format->bit_depth = 16; /* TODO? */
	format->sample_rate =
		sampling_frequency_table[asc->samplingFrequencyIndex];
	format->aac.data_format = ADEF_AAC_DATA_FORMAT_RAW;

	return 0;
}


int aac_asc_from_adef_format(const struct adef_format *format,
			     struct aac_asc *asc)
{
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(asc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(!adef_is_format_valid(format), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		format->aac.data_format != ADEF_AAC_DATA_FORMAT_RAW, EINVAL);

	asc->audioObjectType = AAC_AOT_AAC_LC;

	for (size_t i = 0; i < ARRAY_SIZE(sampling_frequency_table); i++) {
		if (sampling_frequency_table[i] == format->sample_rate) {
			asc->samplingFrequencyIndex = i;
			break;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(channel_configuration_table); i++) {
		if (channel_configuration_table[i] == format->channel_count) {
			asc->channelConfiguration = i;
			break;
		}
	}

	return 0;
}
