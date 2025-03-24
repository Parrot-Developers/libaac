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

#ifndef _AAC_READER_H_
#define _AAC_READER_H_


struct aac_reader;


/* Parse frame data */
#define AAC_READER_FLAGS_FRAME_DATA 0x01


AAC_API
int aac_reader_new(const struct aac_ctx_cbs *cbs,
		   void *userdata,
		   struct aac_reader **ret_obj);


AAC_API
int aac_reader_destroy(struct aac_reader *reader);


AAC_API
struct aac_ctx *aac_reader_get_ctx(struct aac_reader *reader);


AAC_API
int aac_reader_stop(struct aac_reader *reader);


AAC_API
int aac_reader_parse(struct aac_reader *reader,
		     uint32_t flags,
		     const uint8_t *buf,
		     size_t len,
		     size_t *off);


AAC_API
int aac_parse_asc(const uint8_t *buf, size_t len, struct aac_asc *asc);


AAC_API
int aac_parse_adts(const uint8_t *buf, size_t len, struct aac_adts *adts);


#endif /* !_AAC_READER_H_ */
