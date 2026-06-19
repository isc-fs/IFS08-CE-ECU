/* SPDX-License-Identifier: proprietary */
/*
 * The ECU CAN message registry. can_codecs.hpp #includes THIS file once per
 * expansion pass (struct / encode / decode / descriptor / overlap-guard), so
 * adding a frame to the DSL is a single line here plus its .def.
 *
 * Scope: every frame the ECU participates in -- the frames it OWNS (TX) and
 * the frames it CONSUMES (RX) -- so the generated DBC documents the whole
 * contract and the encoders/decoders derive from one source. The `Sender`
 * field in each CAN_MSG tags ownership (VCU = this ECU, AMS, Pit_Tool, EMC).
 */

/* ---- ECU-owned TX ------------------------------------------------------- */
#include "vcu_heartbeat.def"        /* 0x100  DC-bus heartbeat -> ACU         */

/* pit-diag stream (gated by 0x7E0) */
#include "pit_diag_ack.def"         /* 0x7E1                                  */
#include "pit_diag_status.def"      /* 0x700                                  */
#include "pit_diag_pedals.def"      /* 0x701                                  */
#include "pit_diag_brake.def"       /* 0x705                                  */
#include "pit_diag_inverter.def"    /* 0x702                                  */
#include "pit_diag_fwinfo.def"      /* 0x703                                  */
#include "pit_diag_health.def"      /* 0x704                                  */

/* ---- RX consumed (sender owns the layout) ------------------------------- */
#include "acu_ok_precharge.def"     /* 0x020  AMS precharge-OK                */
#include "acu_v_cell_min.def"       /* 0x12C  AMS min cell voltage            */
#include "ams_status.def"           /* 0x4A0  AMS FSM state / ams_ok          */
#include "bl_boot_trigger.def"      /* 0x002  jump-to-bootloader magic        */
#include "pit_diag_cmd.def"         /* 0x7E0  pit-diag enable/disable magic   */

/*
 * TODO(inverter): the NX/EMC inverter frames (RX 0x461 state / 0x463 speed /
 * 0x466 DC-bus; TX 0x360 setpoint-1 / 0x362 setpoint-3) are AUTOSAR E2E
 * Profile 1 protected (CRC8 in byte 0 + 4-bit rolling counter). They land in
 * a follow-up once the E2E question is settled (does the inverter ENFORCE E2E
 * on RX? -- the old VCU sent plain zeros there). The E2E CRC/counter is
 * computed in the inverter adapter before encode, per the raw-struct/adapter
 * convention.
 */
