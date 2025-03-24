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

#ifndef _AAC_PRIV_H_
#define _AAC_PRIV_H_

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#define ULOG_TAG aac
#include <ulog.h>

#include <aac/aac.h>


/* 5.7 Mathematical functions */
#define Abs(x) ((x) >= 0 ? (x) : -(x))
#define Min(x, y) ((x) < (y) ? (x) : (y))
#define Max(x, y) ((x) > (y) ? (x) : (y))


#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


/**
 * Table 1.18 – Sampling Frequency Index
 */
static const uint32_t sampling_frequency_table[16] = {
	96000,
	88200,
	64000,
	48000,
	44100,
	32000,
	24000,
	22050,
	16000,
	12000,
	11025,
	8000,
	7350,
	0, /* Reserved */
	0, /* Reserved */
	0, /* Escape value: frequency is written explicitely */
};


/**
 * Table 1.19 – Channel Configuration
 */
static const uint8_t channel_configuration_table[16] = {
	0, /* Defined in AOT Specifc Config */
	1, /* 1 channel: front-center */
	2, /* 2 channels: front-left, front-right */
	3, /* 3 channels: front-center, front-left, front-right */
	4, /* 4 channels: front-center, front-left, front-right, back-center */
	5, /* 5 channels: front-center, front-left, front-right, back-left,
	      back-right */
	6, /* 6 channels: front-center, front-left, front-right, back-left,
	      back-right, LFE-channel*/
	8, /* 8 channels: front-center, front-left, front-right, side-left,
	      side-right, back-left, back-right, LFE-channel */
	0, /* Reserved */
	0, /* Reserved */
	0, /* etc. */
	0,
	0,
	0,
	0,
	0,
};


struct aac_ctx {
	enum adef_aac_data_format data_format;
	struct aac_scalefactor_bands_and_grouping info;
	union {
		struct aac_adts adts;
		struct aac_asc asc;
	};
	union {
		struct aac_adts_frame adts_frame;
		struct aac_raw_data_block raw_data_block;
	};
};


#endif /* !_AAC_PRIV_H_ */
