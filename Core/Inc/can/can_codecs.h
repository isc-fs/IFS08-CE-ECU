/* SPDX-License-Identifier: proprietary */
/*
 * Code-first CAN codec layer (C11 port of isc-fs/IFS08-CE-AMS
 * Core/Inc/can/can_codecs.hpp). The message registry
 * (Core/Inc/can/messages/all_messages.inc) is #included once per expansion
 * pass below under different macro definitions, producing for every message:
 *   pass 0  per-message ID / DLC constants  (Name##_ID, Name##_DLC)
 *   pass 1  a typed struct  (+ per-field width _Static_assert)
 *   pass 2  the firmware encoder  encode_##Name(const Name##_t*, uint8_t*)
 *   pass 3  the firmware decoder  decode_##Name(const uint8_t*, Name##_t*)
 *   pass 4  a runtime can_field_desc_t[] table + ALL_MSGS[]  (host dbc_dump)
 *
 * There is exactly ONE place each layout is written down -- its .def -- and
 * struct / encoder / decoder / DBC row all derive from it, so a field add,
 * width change or endian flip moves them together. No second source of
 * truth, no firmware<->DBC drift.
 *
 * Differences from the C++ original (forced by C / scoped down for the ECU):
 *  - No namespace; encoders are `encode_<Name>`, helpers `can_dsl_*`.
 *  - Encoders take `uint8_t *d` (not a sized array reference), so the
 *    compile-time DLC check is lost; pass 0's Name##_DLC + the parity test
 *    cover it instead. Callers pass the 8-byte can_msg_t.data buffer.
 *  - static_assert -> C11 _Static_assert (legal as a struct member).
 *  - pass 4 (descriptor tables) is guarded by ECU_CAN_DESCRIPTORS so only the
 *    host dbc_dump pulls the data in; firmware TUs get only the static-inline
 *    codecs (unused ones drop out, no per-TU table duplication).
 *  - The C++ pass-5 constexpr bit-overlap guard can't fold in C; that check
 *    moves to the runtime parity suite instead.
 *
 * CONVENTION -- the struct holds RAW WIRE INTEGERS. (factor, offset) are
 * DBC-display metadata ONLY; the encoder/decoder never apply them. Any
 * physical<->raw scaling happens in the adapter BEFORE the struct is filled.
 *
 * Field macros (byte-aligned position given as a BYTE index):
 *   FIELD_LE   little-endian unsigned     FIELD_LE_S  little-endian signed
 *   FIELD_BE   big-endian   unsigned      FIELD_BE_S  big-endian signed
 * Sub-byte / unaligned (position given as an absolute START BIT):
 *   FIELD_LE_BITS  little-endian unsigned   FIELD_BE_BITS  big-endian unsigned
 */

#ifndef ECU_CAN_CODECS_H_
#define ECU_CAN_CODECS_H_

#include "can_dsl.h"

/* Low `len` bits, guarding the len==64 UB of (1<<64). */
#define CAN_DSL_MASK(len) (((len) >= 64) ? ~0ull : ((1ull << (len)) - 1u))

/* ---- pass 0: per-message ID / DLC constants ------------------------------ */
#define CAN_MSG(Name, Id, Dlc, Sender, Period) enum { Name##_ID = (int)(Id), Name##_DLC = (int)(Dlc) };
#define CAN_MSG_END(Name)
#define FIELD_LE(name, ctype, byte, len, f, o, u)
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)
#define FIELD_BE(name, ctype, byte, len, f, o, u)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u)
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u)
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

/* ---- pass 1: typed structs (+ per-field width _Static_assert) ------------- */
#define CAN_MSG(Name, Id, Dlc, Sender, Period) typedef struct {
#define CAN_MSG_END(Name) } Name##_t;
#define ECU_DSL_W(name, ctype, len) ctype name; \
    _Static_assert((len) <= 8u*sizeof(ctype), "DSL " #name ": bit length exceeds " #ctype " width");
#define FIELD_LE(name, ctype, byte, len, f, o, u)        ECU_DSL_W(name, ctype, len)
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)      ECU_DSL_W(name, ctype, len)
#define FIELD_BE(name, ctype, byte, len, f, o, u)        ECU_DSL_W(name, ctype, len)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)      ECU_DSL_W(name, ctype, len)
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u)  ECU_DSL_W(name, ctype, len)
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u)  ECU_DSL_W(name, ctype, len)
#include "messages/all_messages.inc"
#undef ECU_DSL_W
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

/* ---- pass 2: encode (struct -> bytes) ------------------------------------ */
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    static inline void encode_##Name(const Name##_t *in, uint8_t *d) { \
        for (unsigned _i = 0u; _i < (unsigned)(Dlc); ++_i) d[_i] = 0u;
#define CAN_MSG_END(Name) }
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    can_dsl_set_le(d, 8u*(byte), len, (uint64_t)(in->name) & CAN_DSL_MASK(len));
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) \
    can_dsl_set_le(d, 8u*(byte), len, (uint64_t)(in->name) & CAN_DSL_MASK(len));
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    can_dsl_set_be(d, 8u*(byte)+7u, len, (uint64_t)(in->name) & CAN_DSL_MASK(len));
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    can_dsl_set_be(d, 8u*(byte)+7u, len, (uint64_t)(in->name) & CAN_DSL_MASK(len));
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u) \
    can_dsl_set_le(d, (start), len, (uint64_t)(in->name) & CAN_DSL_MASK(len));
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u) \
    can_dsl_set_be(d, (start), len, (uint64_t)(in->name) & CAN_DSL_MASK(len));
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

/* ---- pass 3: decode (bytes -> struct) ------------------------------------ */
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    static inline void decode_##Name(const uint8_t *d, Name##_t *out) {
#define CAN_MSG_END(Name) }
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    out->name = (ctype)(can_dsl_get_le(d, 8u*(byte), len));
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) \
    out->name = (ctype)(can_dsl_sign_extend(can_dsl_get_le(d, 8u*(byte), len), len));
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    out->name = (ctype)(can_dsl_get_be(d, 8u*(byte)+7u, len));
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    out->name = (ctype)(can_dsl_sign_extend(can_dsl_get_be(d, 8u*(byte)+7u, len), len));
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u) \
    out->name = (ctype)(can_dsl_get_le(d, (start), len));
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u) \
    out->name = (ctype)(can_dsl_get_be(d, (start), len));
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

/* ---- pass 4: runtime descriptors (host dbc_dump only) -------------------- */
#ifdef ECU_CAN_DESCRIPTORS
#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    static const can_field_desc_t Name##_fields[] = {
#define CAN_MSG_END(Name) };
#define FIELD_LE(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte),    len, false, false, (double)(f), (double)(o), u },
#define FIELD_LE_S(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte),    len, false, true,  (double)(f), (double)(o), u },
#define FIELD_BE(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte)+7u, len, true,  false, (double)(f), (double)(o), u },
#define FIELD_BE_S(name, ctype, byte, len, f, o, u) \
    { #name, 8u*(byte)+7u, len, true,  true,  (double)(f), (double)(o), u },
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u) \
    { #name, (start),      len, false, false, (double)(f), (double)(o), u },
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u) \
    { #name, (start),      len, true,  false, (double)(f), (double)(o), u },
#include "messages/all_messages.inc"
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS

#define CAN_MSG(Name, Id, Dlc, Sender, Period) \
    { #Name, (Id), (Dlc), (Sender), (Period), Name##_fields, \
      sizeof(Name##_fields)/sizeof(can_field_desc_t) },
#define CAN_MSG_END(Name)
#define FIELD_LE(name, ctype, byte, len, f, o, u)
#define FIELD_LE_S(name, ctype, byte, len, f, o, u)
#define FIELD_BE(name, ctype, byte, len, f, o, u)
#define FIELD_BE_S(name, ctype, byte, len, f, o, u)
#define FIELD_LE_BITS(name, ctype, start, len, f, o, u)
#define FIELD_BE_BITS(name, ctype, start, len, f, o, u)
static const can_msg_desc_t ALL_MSGS[] = {
#include "messages/all_messages.inc"
};
static const unsigned ALL_MSGS_COUNT = sizeof(ALL_MSGS) / sizeof(can_msg_desc_t);
#undef CAN_MSG
#undef CAN_MSG_END
#undef FIELD_LE
#undef FIELD_LE_S
#undef FIELD_BE
#undef FIELD_BE_S
#undef FIELD_LE_BITS
#undef FIELD_BE_BITS
#endif /* ECU_CAN_DESCRIPTORS */

#undef CAN_DSL_MASK

#endif /* ECU_CAN_CODECS_H_ */
