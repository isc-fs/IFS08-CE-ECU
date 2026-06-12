/* SPDX-License-Identifier: proprietary */
/*
 * The ECU TX message registry. can_codecs.h #includes THIS file once per
 * expansion pass (constants / struct / encode / decode / descriptor), so
 * adding a frame to the DSL is a single line here plus its .def.
 *
 * Scope: every frame this repo OWNS the wire contract for. RX frames the ECU
 * consumes from another node (AMS 0x020/0x12C/0x131..0x137/0x4A0, the NX
 * inverter 0x461..0x466) are NOT here -- the sender owns those; we consume
 * via the sender's released DBC.
 *
 * Foundation scope (this PR): the 0x100 VCU heartbeat only -- the end-to-end
 * proof of the pipeline. The ECU-owned pit-diag frames (0x700..0x703, 0x7E1)
 * and the uDV/FDCAN2 contract (#17) land in follow-up PRs.
 */

#include "vcu_heartbeat.def"
