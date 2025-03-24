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

/* codecheck_ignore[COMPLEX_MACRO] */
#define AAC_ENUM_CASE(_prefix, _name)                                          \
	case _prefix##_name:                                                   \
		return #_name


#define AAC_ENUM_MAP(_prefix, _name)                                           \
	{                                                                      \
		_prefix##_name, #_name                                         \
	}


static const struct {
	const enum aac_audioObjectType aot;
	const char *str;
} aot_map[] = {
	AAC_ENUM_MAP(AAC_AOT_, NULL),
	AAC_ENUM_MAP(AAC_AOT_, AAC_MAIN),
	AAC_ENUM_MAP(AAC_AOT_, AAC_LC),
	AAC_ENUM_MAP(AAC_AOT_, AAC_SSR),
	AAC_ENUM_MAP(AAC_AOT_, AAC_LTP),
	AAC_ENUM_MAP(AAC_AOT_, SBR),
	AAC_ENUM_MAP(AAC_AOT_, AAC_SCALABLE),
	AAC_ENUM_MAP(AAC_AOT_, TWINVQ),
	AAC_ENUM_MAP(AAC_AOT_, CELP),
	AAC_ENUM_MAP(AAC_AOT_, HVXC),
	AAC_ENUM_MAP(AAC_AOT_, TTSI),
	AAC_ENUM_MAP(AAC_AOT_, MAINSYNTH),
	AAC_ENUM_MAP(AAC_AOT_, WAVESYNTH),
	AAC_ENUM_MAP(AAC_AOT_, MIDI),
	AAC_ENUM_MAP(AAC_AOT_, SAFX),
	AAC_ENUM_MAP(AAC_AOT_, ER_AAC_LC),
	AAC_ENUM_MAP(AAC_AOT_, ER_AAC_LTP),
	AAC_ENUM_MAP(AAC_AOT_, ER_AAC_SCALABLE),
	AAC_ENUM_MAP(AAC_AOT_, ER_TWINVQ),
	AAC_ENUM_MAP(AAC_AOT_, ER_BSAC),
	AAC_ENUM_MAP(AAC_AOT_, ER_AAC_LD),
	AAC_ENUM_MAP(AAC_AOT_, ER_CELP),
	AAC_ENUM_MAP(AAC_AOT_, ER_HVXC),
	AAC_ENUM_MAP(AAC_AOT_, ER_HILN),
	AAC_ENUM_MAP(AAC_AOT_, ER_PARAM),
	AAC_ENUM_MAP(AAC_AOT_, SSC),
	AAC_ENUM_MAP(AAC_AOT_, PS),
	AAC_ENUM_MAP(AAC_AOT_, SURROUND),
	AAC_ENUM_MAP(AAC_AOT_, ESCAPE),
	AAC_ENUM_MAP(AAC_AOT_, L1),
	AAC_ENUM_MAP(AAC_AOT_, L2),
	AAC_ENUM_MAP(AAC_AOT_, L3),
	AAC_ENUM_MAP(AAC_AOT_, DST),
	AAC_ENUM_MAP(AAC_AOT_, ALS),
	AAC_ENUM_MAP(AAC_AOT_, SLS),
	AAC_ENUM_MAP(AAC_AOT_, SLS_NON_CORE),
	AAC_ENUM_MAP(AAC_AOT_, ER_AAC_ELD),
	AAC_ENUM_MAP(AAC_AOT_, SMR_SIMPLE),
	AAC_ENUM_MAP(AAC_AOT_, SMR_MAIN),
};


enum aac_audioObjectType aac_aot_from_str(const char *str)
{
	enum aac_audioObjectType ret = AAC_AOT_NULL;

	ULOG_ERRNO_RETURN_VAL_IF(str == NULL, EINVAL, ret);

	for (unsigned int i = 0; i < ARRAY_SIZE(aot_map); i++) {
		if (strcasecmp(str, aot_map[i].str) == 0)
			return aot_map[i].aot;
	}
	ULOGW("%s: unknown AOT '%s'", __func__, str);
	return ret;
}


const char *aac_aot_to_str(enum aac_audioObjectType aot)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(aot_map); i++) {
		if (aot == aot_map[i].aot)
			return aot_map[i].str;
	}
	return "UNKNOWN";
}
