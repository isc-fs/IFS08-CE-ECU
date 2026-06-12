/* SPDX-License-Identifier: proprietary */
/*
 * Bit-level pack/unpack helpers + runtime descriptor types for the
 * code-first CAN DSL. Shared between the firmware (encode/decode) and the
 * host-side dbc_dump tool (which walks the FieldDesc tables to emit a .dbc).
 *
 * Nothing here is message-specific. Message layouts live in
 * Core/Inc/can/messages/ *.def and are #included by can_codecs.h.
 *
 * C11 PORT of isc-fs/IFS08-CE-AMS Core/Inc/can/can_dsl.hpp. The ECU is pure
 * C, so the C++ original (templates / references / namespaces / constexpr)
 * is reimplemented here in C: the bit traversals are byte-for-byte identical
 * (same LE linear / BE DBC-sawtooth walk), so frames stay wire-compatible
 * with the AMS decoder. The array-window/constexpr-overlap machinery the AMS
 * uses for its cell/temp grids is omitted -- the ECU has no array frames.
 */

#ifndef ECU_CAN_DSL_H_
#define ECU_CAN_DSL_H_

#include <stdint.h>
#include <stdbool.h>

/* ---- Bit traversals ------------------------------------------------------
 *
 * LE (Intel): value bit i lives at frame bit (start + i). Linear.
 *
 * BE (Motorola forward MSB, DBC sawtooth): the MSB of the value lives at
 * frame bit `start` (= 8*byte + 7 for byte-aligned fields); bits descend
 * within a byte toward bit 0, then jump to bit 7 of the next byte. Matches
 * the cantools / Vector CANdb++ convention.
 */

static inline uint64_t can_dsl_get_le(const uint8_t *d, unsigned start, unsigned len)
{
    uint64_t v = 0u;
    for (unsigned i = 0u; i < len; ++i) {
        const unsigned bit = start + i;
        v |= (uint64_t)((d[bit >> 3] >> (bit & 7u)) & 1u) << i;
    }
    return v;
}

static inline void can_dsl_set_le(uint8_t *d, unsigned start, unsigned len, uint64_t v)
{
    for (unsigned i = 0u; i < len; ++i) {
        const unsigned bit = start + i;
        const uint8_t b = (uint8_t)((v >> i) & 1u);
        d[bit >> 3] = (uint8_t)((d[bit >> 3] & (uint8_t)~(1u << (bit & 7u))) | (b << (bit & 7u)));
    }
}

static inline uint64_t can_dsl_get_be(const uint8_t *d, unsigned start, unsigned len)
{
    uint64_t v = 0u;
    unsigned bit = start;
    for (unsigned i = 0u; i < len; ++i) {
        v = (v << 1) | ((d[bit >> 3] >> (bit & 7u)) & 1u);
        if ((bit & 7u) == 0u) bit += 15u;
        else                  bit -= 1u;
    }
    return v;
}

static inline void can_dsl_set_be(uint8_t *d, unsigned start, unsigned len, uint64_t v)
{
    unsigned bit = start;
    for (unsigned i = 0u; i < len; ++i) {
        const uint8_t b = (uint8_t)((v >> (len - 1u - i)) & 1u);
        d[bit >> 3] = (uint8_t)((d[bit >> 3] & (uint8_t)~(1u << (bit & 7u))) | (b << (bit & 7u)));
        if ((bit & 7u) == 0u) bit += 15u;
        else                  bit -= 1u;
    }
}

static inline int64_t can_dsl_sign_extend(uint64_t v, unsigned len)
{
    const uint64_t m = (uint64_t)1 << (len - 1u);
    return (int64_t)((v ^ m) - m);
}

/* ---- Runtime descriptors (read by the host dbc_dump tool) ---------------- */

typedef struct {
    const char *name;
    unsigned    start_bit;   /* already in DBC convention (LE: 8*byte; BE: 8*byte+7) */
    unsigned    len;
    bool        big_endian;
    bool        is_signed;
    double      factor;
    double      offset;
    const char *unit;
} can_field_desc_t;

typedef struct {
    const char             *name;
    uint32_t                id;
    unsigned                dlc;
    const char             *sender;
    unsigned                period_ms;
    const can_field_desc_t *fields;
    unsigned                n_fields;
} can_msg_desc_t;

#endif /* ECU_CAN_DSL_H_ */
