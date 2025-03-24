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

#include "aac_test.h"


static void test_reader_parse_asc(void)
{
	int ret;
	struct aac_asc asc;
	struct adef_format fmt;

	ret = aac_parse_asc(NULL, 0, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_parse_asc(NULL, 0, &asc);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid encoding */
	uint8_t buf[] = {0x19, 0x90};

	/* Invalid size */
	ret = aac_parse_asc(&buf[0], 1, &asc);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* No format supplied */
	ret = aac_parse_asc(&buf[0], 1, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid ASC */
	ret = aac_parse_asc(&buf[0], sizeof(buf), &asc);
	CU_ASSERT_EQUAL(ret, 0);
	ret = aac_asc_to_adef_format(&asc, &fmt);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* AAC_LC, 48KHz, stereo */
	buf[0] = 0x11;
	buf[1] = 0x90;
	ret = aac_parse_asc(&buf[0], sizeof(buf), &asc);
	CU_ASSERT_EQUAL(ret, 0);
	ret = aac_asc_to_adef_format(&asc, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_48000hz_stereo_raw));

	/* AAC_LC, 44.1KHz, mono */
	buf[1] = 0x88;
	ret = aac_parse_asc(&buf[0], sizeof(buf), &asc);
	CU_ASSERT_EQUAL(ret, 0);
	ret = aac_asc_to_adef_format(&asc, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_48000hz_mono_raw));

	/* AAC_LC, 44.1KHz, stereo */
	buf[0] = 0x12;
	buf[1] = 0x10;
	ret = aac_parse_asc(&buf[0], sizeof(buf), &asc);
	CU_ASSERT_EQUAL(ret, 0);
	ret = aac_asc_to_adef_format(&asc, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_44100hz_stereo_raw));

	/* More complex ASC */
	uint8_t asc_2[] = {0x11, 0x90, 0x56, 0xe5, 0x00};
	ret = aac_parse_asc(&asc_2[0], sizeof(asc_2), &asc);
	CU_ASSERT_EQUAL(ret, 0);
	ret = aac_asc_to_adef_format(&asc, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_48000hz_stereo_raw));
}


static void test_write_asc(void)
{
	int ret;
	struct adef_format fmt = {0};
	struct aac_asc asc = {0};
	uint8_t *buf = NULL;
	size_t buf_len = 0;

	ret = aac_asc_from_adef_format(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_asc_from_adef_format(&fmt, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_asc_from_adef_format(NULL, &asc);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_asc(NULL, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_asc(&asc, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_asc(&asc, &buf, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_asc(NULL, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid format: PCM */
	fmt = adef_pcm_16b_48000hz_mono;
	ret = aac_asc_from_adef_format(&fmt, &asc);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid data format: ADTS */
	fmt = adef_aac_lc_16b_48000hz_stereo_adts;
	ret = aac_asc_from_adef_format(&fmt, &asc);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* AAC_LC, 48KHz, stereo */
	fmt = adef_aac_lc_16b_48000hz_stereo_raw;
	ret = aac_asc_from_adef_format(&fmt, &asc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(asc.audioObjectType, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(asc.samplingFrequencyIndex, 3);
	CU_ASSERT_EQUAL(asc.channelConfiguration, 2);
	ret = aac_write_asc(&asc, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf_len, 4);
	CU_ASSERT_EQUAL(buf[0], 0x11);
	CU_ASSERT_EQUAL(buf[1], 0x90);
	CU_ASSERT_EQUAL(buf[2], 0x00);
	CU_ASSERT_EQUAL(buf[3], 0x00);
	free(buf);

	/* AAC_LC, 48KHz, mono */
	fmt = adef_aac_lc_16b_48000hz_mono_raw;
	ret = aac_asc_from_adef_format(&fmt, &asc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(asc.audioObjectType, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(asc.samplingFrequencyIndex, 3);
	CU_ASSERT_EQUAL(asc.channelConfiguration, 1);
	ret = aac_write_asc(&asc, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf_len, 4);
	CU_ASSERT_EQUAL(buf[0], 0x11);
	CU_ASSERT_EQUAL(buf[1], 0x88);
	CU_ASSERT_EQUAL(buf[2], 0x00);
	CU_ASSERT_EQUAL(buf[3], 0x00);
	free(buf);

	/* AAC_LC, 44.1KHz, stereo */
	fmt = adef_aac_lc_16b_44100hz_stereo_raw;
	ret = aac_asc_from_adef_format(&fmt, &asc);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(asc.audioObjectType, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(asc.samplingFrequencyIndex, 4);
	CU_ASSERT_EQUAL(asc.channelConfiguration, 2);
	ret = aac_write_asc(&asc, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf_len, 4);
	CU_ASSERT_EQUAL(buf[0], 0x12);
	CU_ASSERT_EQUAL(buf[1], 0x10);
	CU_ASSERT_EQUAL(buf[2], 0x00);
	CU_ASSERT_EQUAL(buf[3], 0x00);
	free(buf);
}


static void test_reader_parse_adts(void)
{
	int ret;
	struct aac_adts adts;
	struct adef_format fmt;

	ret = aac_parse_adts(NULL, 0, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_parse_adts(NULL, 0, &adts);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	uint8_t bad_adts_short[] = {0x19, 0x90};
	uint8_t bad_adts[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	uint8_t adts_mono[] = {0xff, 0xf1, 0x4c, 0x40, 0x01, 0x7f, 0xfc};
	uint8_t adts_stereo[] = {0xff, 0xf1, 0x4c, 0x80, 0x01, 0xbf, 0xfc};

	/* Invalid size */
	ret = aac_parse_adts(
		&bad_adts_short[0], sizeof(bad_adts_short), &adts);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid data */
	ret = aac_parse_adts(&bad_adts[0], sizeof(bad_adts), &adts);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* AAC_LC, 48KHz, mono */
	ret = aac_parse_adts(&adts_mono[0], sizeof(adts_mono), &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 3);
	CU_ASSERT_EQUAL(adts.channel_configuration, 1);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 11);
	ret = aac_adts_to_adef_format(&adts, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_48000hz_mono_adts));

	/* AAC_LC, 48KHz, stereo */
	ret = aac_parse_adts(
		&adts_stereo[0], sizeof(adts_stereo), &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 3);
	CU_ASSERT_EQUAL(adts.channel_configuration, 2);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 13);
	ret = aac_adts_to_adef_format(&adts, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_48000hz_stereo_adts));

	/* AAC_LC, 44.1KHz, mono */
	adts_mono[2] = 0x50;
	ret = aac_parse_adts(&adts_mono[0], sizeof(adts_mono), &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 4);
	CU_ASSERT_EQUAL(adts.channel_configuration, 1);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 11);
	ret = aac_adts_to_adef_format(&adts, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_44100hz_mono_adts));

	/* AAC_LC, 44.1KHz, stereo */
	adts_stereo[2] = 0x50;
	ret = aac_parse_adts(
		&adts_stereo[0], sizeof(adts_stereo), &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 4);
	CU_ASSERT_EQUAL(adts.channel_configuration, 2);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 13);
	ret = aac_adts_to_adef_format(&adts, &fmt);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_TRUE(
		adef_format_cmp(&fmt, &adef_aac_lc_16b_44100hz_stereo_adts));
}


static void test_write_adts(void)
{
	int ret;
	struct adef_format fmt = {0};
	struct aac_adts adts = {0};
	uint8_t *buf = NULL;
	size_t buf_len = 0;

	ret = aac_adts_from_adef_format(NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_adts_from_adef_format(&fmt, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_adts_from_adef_format(NULL, &adts);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_adts(NULL, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_adts(&adts, NULL, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_adts(&adts, &buf, NULL);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = aac_write_adts(NULL, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid format: PCM */
	fmt = adef_pcm_16b_48000hz_mono;
	ret = aac_adts_from_adef_format(&fmt, &adts);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* Invalid data format: RAW */
	fmt = adef_aac_lc_16b_48000hz_stereo_raw;
	ret = aac_adts_from_adef_format(&fmt, &adts);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	/* AAC_LC, 48KHz, stereo */
	fmt = adef_aac_lc_16b_48000hz_stereo_adts;
	ret = aac_adts_from_adef_format(&fmt, &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 3);
	CU_ASSERT_EQUAL(adts.channel_configuration, 2);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 7);
	adts.aac_frame_length = 13;
	ret = aac_write_adts(&adts, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf_len, 7);
	CU_ASSERT_EQUAL(buf[0], 0xFF);
	CU_ASSERT_EQUAL(buf[1], 0xF1);
	CU_ASSERT_EQUAL(buf[2], 0x4c);
	CU_ASSERT_EQUAL(buf[3], 0x80);
	CU_ASSERT_EQUAL(buf[4], 0x01);
	CU_ASSERT_EQUAL(buf[5], 0xbf);
	CU_ASSERT_EQUAL(buf[6], 0xfc);
	free(buf);

	/* AAC_LC, 48KHz, mono */
	fmt = adef_aac_lc_16b_48000hz_mono_adts;
	ret = aac_adts_from_adef_format(&fmt, &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 3);
	CU_ASSERT_EQUAL(adts.channel_configuration, 1);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 7);
	adts.aac_frame_length = 11;
	ret = aac_write_adts(&adts, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf_len, 7);
	CU_ASSERT_EQUAL(buf[0], 0xFF);
	CU_ASSERT_EQUAL(buf[1], 0xF1);
	CU_ASSERT_EQUAL(buf[2], 0x4c);
	CU_ASSERT_EQUAL(buf[3], 0x40);
	CU_ASSERT_EQUAL(buf[4], 0x01);
	CU_ASSERT_EQUAL(buf[5], 0x7f);
	CU_ASSERT_EQUAL(buf[6], 0xfc);
	free(buf);

	/* AAC_LC, 44.1KHz, stereo */
	fmt = adef_aac_lc_16b_44100hz_stereo_adts;
	ret = aac_adts_from_adef_format(&fmt, &adts);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(adts.syncword, 0xFFF);
	CU_ASSERT_EQUAL(adts.ID, 0);
	CU_ASSERT_EQUAL(adts.layer, 0);
	CU_ASSERT_EQUAL(adts.protection_absent, 1);
	CU_ASSERT_EQUAL(adts.profile_ObjectType + 1, AAC_AOT_AAC_LC);
	CU_ASSERT_EQUAL(adts.sampling_frequency_index, 4);
	CU_ASSERT_EQUAL(adts.channel_configuration, 2);
	CU_ASSERT_EQUAL(adts.original_copy, 0);
	CU_ASSERT_EQUAL(adts.home, 0);
	CU_ASSERT_EQUAL(adts.aac_frame_length, 7);
	adts.aac_frame_length = 13;
	ret = aac_write_adts(&adts, &buf, &buf_len);
	CU_ASSERT_EQUAL(ret, 0);
	CU_ASSERT_EQUAL(buf_len, 7);
	CU_ASSERT_EQUAL(buf[0], 0xFF);
	CU_ASSERT_EQUAL(buf[1], 0xF1);
	CU_ASSERT_EQUAL(buf[2], 0x50);
	CU_ASSERT_EQUAL(buf[3], 0x80);
	CU_ASSERT_EQUAL(buf[4], 0x01);
	CU_ASSERT_EQUAL(buf[5], 0xbf);
	CU_ASSERT_EQUAL(buf[6], 0xfc);
	free(buf);
}


CU_TestInfo g_aac_test_asc_adts[] = {
	{FN("parse-asc"), &test_reader_parse_asc},
	{FN("write-asc"), &test_write_asc},
	{FN("parse-adts"), &test_reader_parse_adts},
	{FN("write-adts"), &test_write_adts},

	CU_TEST_INFO_NULL,
};
