/* SPDX-License-Identifier: proprietary */
/*
 * dbc_dump -- emit DBC BO_/SG_ rows straight from the code-first CAN DSL
 * descriptors (ALL_MSGS). The committed docs/dbc/ecu.dbc IS this tool's
 * output; CI regenerates and diffs, so the DBC cannot drift from the .def
 * the firmware encoders are generated from. One source of truth.
 *
 * C port of isc-fs/IFS08-CE-AMS tools/dbc_dump.cpp (the ECU has no
 * array-of-frames families, so that section is dropped). Defining
 * ECU_CAN_DESCRIPTORS pulls in the pass-4 descriptor tables.
 *
 * Build + run (host):
 *   cc -std=c11 -I Core/Inc tools/dbc_dump.c -o /tmp/dbc_dump && /tmp/dbc_dump
 */

#define ECU_CAN_DESCRIPTORS
#include "can/can_codecs.h"

#include <stdio.h>

int main(void)
{
    /* DBC header boilerplate (cantools-parseable). Lean by design (no CM_ /
     * VAL_ tables); DBCinator re-enriches downstream. */
    printf("VERSION \"\"\n\n");
    printf("NS_ :\n");
    static const char *ns[] = {
        "NS_DESC_", "CM_", "BA_DEF_", "BA_", "VAL_", "CAT_DEF_", "CAT_", "FILTER",
        "BA_DEF_DEF_", "EV_DATA_", "ENVVAR_DATA_", "SGTYPE_", "SGTYPE_VAL_",
        "BA_DEF_SGTYPE_", "BA_SGTYPE_", "SIG_TYPE_REF_", "VAL_TABLE_", "SIG_GROUP_",
        "SIG_VALTYPE_", "SIGTYPE_VALTYPE_", "BO_TX_BU_", "BA_DEF_REL_", "BA_REL_",
        "BA_DEF_DEF_REL_", "BU_SG_REL_", "BU_EV_REL_", "BU_BO_REL_", "SG_MUL_VAL_",
    };
    for (unsigned i = 0u; i < sizeof(ns) / sizeof(ns[0]); ++i) printf("\t%s\n", ns[i]);
    printf("\nBS_:\n\n");
    printf("BU_: AMS VCU ECU UDV Pit_Tool\n\n");

    for (unsigned mi = 0u; mi < ALL_MSGS_COUNT; ++mi) {
        const can_msg_desc_t *m = &ALL_MSGS[mi];
        printf("BO_ %u %s: %u %s\n", m->id, m->name, m->dlc, m->sender);
        for (unsigned fi = 0u; fi < m->n_fields; ++fi) {
            const can_field_desc_t *f = &m->fields[fi];
            /* The [min|max] range is required for cantools to parse the SG_
             * line; emit [0|0] ("unspecified", as gen_dbc.py does). Real
             * ranges are display polish DBCinator can enrich downstream. */
            printf(" SG_ %s : %u|%u@%c%c (%g,%g) [0|0] \"%s\" Vector__XXX\n",
                   f->name, f->start_bit, f->len,
                   f->big_endian ? '0' : '1', f->is_signed ? '-' : '+',
                   f->factor, f->offset, f->unit);
        }
        printf("\n");
    }
    return 0;
}
