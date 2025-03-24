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

#ifndef _AAC_SYNTAX_OPS_
#define _AAC_SYNTAX_OPS_


/* Available operation kinds */
#define AAC_SYNTAX_OP_KIND_READ 0
#define AAC_SYNTAX_OP_KIND_WRITE 1
#define AAC_SYNTAX_OP_KIND_DUMP 2

#ifndef AAC_SYNTAX_OP_NAME
#	error "AAC_SYNTAX_OP_NAME shall be defined first"
#endif /* !AAC_SYNTAX_OP_NAME */

#ifndef AAC_SYNTAX_OP_KIND
#	error "AAC_SYNTAX_OP_KIND shall be defined first"
#endif /* !AAC_SYNTAX_OP_KIND */

#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_READ
#	define AAC_SYNTAX_CONST
#else
#	define AAC_SYNTAX_CONST const
#endif

#define _AAC_CAT2(a, b) a##b
#define _AAC_CAT3(a, b, c) a##b##c
#define _AAC_CAT4(a, b, c, d) a##b##c##d

#define _AAC_SYNTAX_FCT(_op, _name) _AAC_CAT4(_aac_, _op, _, _name)

#define AAC_SYNTAX_FCT(_name) _AAC_SYNTAX_FCT(AAC_SYNTAX_OP_NAME, _name)


#define _AAC_READ_BITS(_name, _type, _field, ...)                              \
	do {                                                                   \
		_type _v = 0;                                                  \
		int _res = aac_bs_read_bits_##_name(bs, &_v, ##__VA_ARGS__);   \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
		(_field) = _v;                                                 \
	} while (0)

#define AAC_READ_BITS(_f, _n) _AAC_READ_BITS(u, uint32_t, _f, _n)
#define AAC_READ_BITS_U(_f, _n) _AAC_READ_BITS(u, uint32_t, _f, _n)
#define AAC_READ_BITS_I(_f, _n) _AAC_READ_BITS(i, int32_t, _f, _n)

#define AAC_READ_FLAGS() (((struct aac_reader *)(bs->priv))->flags)


#define _AAC_WRITE_BITS(_name, _type, _field, ...)                             \
	do {                                                                   \
		_type _v = (_field);                                           \
		int _res = aac_bs_write_bits_##_name(bs, _v, ##__VA_ARGS__);   \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
	} while (0)

#define AAC_WRITE_BITS(_f, _n) _AAC_WRITE_BITS(u, uint32_t, _f, _n)
#define AAC_WRITE_BITS_U(_f, _n) _AAC_WRITE_BITS(u, uint32_t, _f, _n)
#define AAC_WRITE_BITS_I(_f, _n) _AAC_WRITE_BITS(i, int32_t, _f, _n)


#define _AAC_DUMP_CALL(_fct, ...)                                              \
	do {                                                                   \
		struct aac_dump *_dump = bs->priv;                             \
		int _res = (*_dump->cbs._fct)(_dump, ##__VA_ARGS__);           \
		ULOG_ERRNO_RETURN_ERR_IF(_res < 0, -_res);                     \
	} while (0)

#define AAC_DUMP_BITS(_f, _n) _AAC_DUMP_CALL(field, #_f, _f)
#define AAC_DUMP_BITS_U(_f, _n) _AAC_DUMP_CALL(field, #_f, _f)
#define AAC_DUMP_BITS_I(_f, _n) _AAC_DUMP_CALL(field, #_f, _f)

#define AAC_DUMP_FLAGS() (((struct aac_dump *)(bs->priv))->flags)


/* clang-format off */
#if AAC_SYNTAX_OP_KIND == AAC_SYNTAX_OP_KIND_DUMP
#  define AAC_BEGIN_STRUCT(_name)   _AAC_DUMP_CALL(begin_struct, #_name)
#  define AAC_END_STRUCT(_name)     _AAC_DUMP_CALL(end_struct, #_name)
#  define AAC_BEGIN_ARRAY(_name)    _AAC_DUMP_CALL(begin_array, #_name)
#  define AAC_END_ARRAY(_name)      _AAC_DUMP_CALL(end_array, #_name)
#  define AAC_BEGIN_ARRAY_ITEM()    _AAC_DUMP_CALL(begin_array_item)
#  define AAC_END_ARRAY_ITEM()      _AAC_DUMP_CALL(end_array_item)
#  define AAC_FIELD(_name, _val)    _AAC_DUMP_CALL(field, #_name, _val)
#  define AAC_FIELD_S(_name, _val)  _AAC_DUMP_CALL(field, _name, _val)
#else
#  define AAC_BEGIN_STRUCT(_name)   do {} while (0)
#  define AAC_END_STRUCT(_name)     do {} while (0)
#  define AAC_BEGIN_ARRAY(_name)    do {} while (0)
#  define AAC_END_ARRAY(_name)      do {} while (0)
#  define AAC_BEGIN_ARRAY_ITEM()    do {} while (0)
#  define AAC_END_ARRAY_ITEM()      do {} while (0)
#  define AAC_FIELD(_name, _val)    do {} while (0)
#  define AAC_FIELD_S(_name, _val)  do {} while (0)
#endif
/* clang-format on */


#define AAC_CB(_ctx, _cbs, _userdata, _name, ...)                              \
	do {                                                                   \
		if ((_cbs) != NULL && (_cbs)->_name != NULL) {                 \
			(*(_cbs)->_name)((_ctx), ##__VA_ARGS__, (_userdata));  \
		}                                                              \
	} while (0)


#endif /* !_AAC_SYNTAX_OPS_ */
