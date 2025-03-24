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

#ifndef _AAC_H_
#define _AAC_H_

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef AAC_API_EXPORTS
#	ifdef _WIN32
#		define AAC_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define AAC_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !AAC_API_EXPORTS */
#	define AAC_API
#endif /* !AAC_API_EXPORTS */

#include "aac/aac_types.h"

#include "aac/aac_bitstream.h"

#include "aac/aac_ctx.h"

#include "aac/aac_dump.h"
#include "aac/aac_reader.h"
#include "aac/aac_writer.h"


AAC_API int aac_adts_to_adef_format(const struct aac_adts *adts,
				    struct adef_format *format);


AAC_API int aac_adts_from_adef_format(const struct adef_format *format,
				      struct aac_adts *adts);


AAC_API int aac_asc_to_adef_format(const struct aac_asc *asc,
				   struct adef_format *format);


AAC_API int aac_asc_from_adef_format(const struct adef_format *format,
				     struct aac_asc *asc);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_AAC_H_ */
