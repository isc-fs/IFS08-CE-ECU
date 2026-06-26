# IFS08-CE-ECU — HIL Bench Test Plan

Authoritative behavioural spec for the **HIL team** to implement against in `IFS08_HIL`.
Every case is pinned to the firmware source on `dev` (file:symbol noted); the HIL team
owns the rig wiring + harness — this document owns *what the firmware must do, the exact
bytes that drive it, and how to observe the result*.

- **Firmware under test:** `dev` (lean FreeRTOS C++17 app, links over the CAN bootloader at `0x08020000`).
- **ECU node id:** `0x01`. Buses: **FDCAN1 = INV** (NX/EMC inverter, standard IDs), **FDCAN2 = ACU** (AMS + pit-tool, shared), both **500 kbit/s classic**.
- **SIL relationship:** the `ecu::Controller` core (FSM + every plausibility cut + the RX decoders) is unit-tested host-side (`tests/sil`). HIL is **not** a re-run — it verifies the same behaviour through the real CAN wire, real RTOS timing, and the real/simulated peers. Cases with a SIL sibling are tagged `[SIL: test_x]`.
- **NEW this session** (no HIL coverage yet) are tagged **🆕** — prioritise: inverter fault-recovery (E-003..E-007), real-AMS integration (F-*), rpm/temps telemetry (J-001/J-002).

> **Notation.** Frame payloads are written `{b0 b1 b2 …}` in **hex**, byte0 first. `··` = don't-care byte. "≥N Hz" means the rig must keep re-sending at that rate or the input goes stale (see §1.2). Multi-byte fields note **LE/BE** explicitly — the bus mixes both.

---

## 1. Test interface

### 1.1 Inject / observe boundaries
| Boundary | Mechanism | Detail |
|---|---|---|
| Inverter feedback | CAN TX on **FDCAN1** | `0x461` state, `0x463` rpm, `0x464` temps, `0x466` DC-bus (§2.2) |
| AMS / ACU | CAN TX on **FDCAN2** | `0x020`, `0x12C`, `0x4A0` (§2.2) |
| Pedals / brake | analog → ADC3 | brake **PF7** (IN3), APPS1 **PF8** (IN7), APPS2 **PF9** (IN2); raw 12-bit (0–4095) |
| Start button | GPIO **PB5**, active-high | debounced 5×10 ms = **50 ms hold** min (`StartDebounceSamples`). If PB5 is unwired on the rig, the bench `ECU_HIL_STUB_START_BTN` image takes start from `0x7E0` byte4 — see #75 |
| Pit-diag arm | CAN `0x7E0` (§2.2) | `DE AD BE EF` enable / `00 00 00 00` disable |
| Bootloader entry | CAN `0x002` (§2.2) | `B0 07 AD 12` |
| Observe | the ECU's own TX | `0x100`, `0x360/0x362`, pit-diag `0x700`-`0x706` (§2.1) |

### 1.2 Input derivation & freshness — **read this before writing any case** (`control_task.cpp:48-83`)
The FSM does **not** see raw frames; `ControlTask` derives its inputs from the RX snapshot every 10 ms. The non-obvious rules:

- **`inv_vconfig_ready`** latches on the **first** `0x466` ever received and **never clears** (`last_vconfig_tick != 0`). One `0x466` arms WaitInvVdcConfig→Precharge for the rest of the boot.
- **`inv_state`** = `0x461` byte4 `& 0x7F`. **`inv_present`** = an inverter frame within **`InvStaleMs` = 200 ms**.
- **`ok_precharge`** = `0x020[0]≠0` **AND** AMS-fresh. **`ams_error`** = `0x4A0[0]==5` **AND** AMS-fresh. **AMS-fresh** = any of `0x020/0x12C/0x4A0` within **`AmsStaleMs` = 200 ms**.
  → **Stream AMS frames ≥5–10 Hz** for the whole test, or `ok_precharge` fail-safes to false and the FSM drops out of Active. A *stale* AMS reads as **not-ok** (re-arm), **not** as error.
- **`v_cell_min_mV`** (derate input) comes from `0x12C` / `0x4A0`; if AMS is stale the core substitutes `CellVDefaultMv = 3600` (no derate).
- **Torque cap:** `out.torque_pct` is clamped to **`config::TorqueCap`** (default 100) *after* the core. Keep it 100 for representative runs.
- The **gated pit-diag stream** (`0x700/0x701/0x702/0x703/0x705/0x706`) fires @100 ms only while armed. **`0x704` is separate** — `DiagTask`, ungated, @1 s.

### 1.3 Reusable setup recipes
**`Setup-Active`** — walk the FSM to Active (referenced by E/F/G/J). Keep all streams running for the whole case:
1. **FDCAN1 @≥10 Hz:** `0x466` DC-bus (e.g. 300 V) → latches `vconfig_ready`; `0x461` byte4 = `04` (Ready).
2. **FDCAN2 @≥10 Hz:** `0x020` = `{01}` (ok_precharge); `0x12C` = `{0E 10}` (3600 mV, healthy → no derate).
3. Arm R2D: brake raw → **1500** (>`BrakeArmRaw` 900, <`BrakePressedRaw` 3000) **and** START held ≥50 ms.
4. After **`R2dSoundMs` = 2000 ms** RTDS, with `0x461`=Ready, FSM reaches **Active** (`0x700.fsm_state` = 5).
5. **Release brake → 0** before applying APPS (else EV.2.3 latches).

**`Setup-Precharge`** — as above through step 1 only (FSM sits at Precharge waiting on `0x020`).

### 1.4 FSM reference (`control.cpp`, enum `ecu::CtrlState`)
`0 WaitInvVdcConfig → 1 Precharge → 2 WaitStartBrake → 3 R2dDelay → 4 WaitInvStandby → 5 Active`, plus `6 AmsError` (entered from any state when `ams_error`, latched). Inverter mode commanded per state: **Off `0x01`** pre-R2D, **Ready `0x04`** at WaitInvStandby, **TorqueEnable `0x06`** in Active; recovery overrides (E-003/4).

---

## 2. Frame appendix (exact wire layouts)

### 2.1 ECU TX — observe these
| ID | Name | Bus | DLC | Layout | Notes |
|----|------|-----|-----|--------|-------|
| `0x100` | VCU_heartbeat | ACU | 2 | `dc_bus_voltage` u16 **LE** @0 | every state, every 10 ms; = last `0x466` DC-bus |
| `0x360` | INV setpoint mode | INV | 3 | `{00 00 MODE}` — mode @ byte2 | Off`01` Ready`04` TorqueEnable`06` Fault`13` HardFaultReset`0D` |
| `0x362` | INV setpoint torque | INV | 4 | `{00 00 lo hi}` — `Torque_Nm_Req` s16 **LE** @2, **NEGATED** | 10%→`00 00`, 50%→`95 FF` (−107), 90%→`2A FF` (−214), 100%→`10 FF` (−240) |
| `0x700` | PitDiag_status | ACU | 8 | b0 `fsm_state`, b1 `inv_state`, **b2 flags**, b3 `torque_pct`, b4-5 `v_cell_min_mV` u16 **BE**, b6-7 `torque_cmd` s16 BE | flags b2: bit0 ev_2_3, bit1 t11_8_9, bit2 rtds_active, bit3 ok_precharge, bit4 start_button. **`torque_cmd` hardwired 0** (deferred — don't assert) |
| `0x701` | PitDiag_pedals | ACU | 8 | `apps1_raw` BE@0, `apps2_raw` BE@2, `brake_raw` BE@4, `apps1_pct`@6, `apps2_pct`@7 | u16 BE raws, u8 % |
| `0x702` | PitDiag_inverter | ACU | 7 | `dc_bus_voltage` u16 **BE**@0, `inv_rpm` **s32 BE**@2-5, `inv_error`@6 | rpm signed |
| `0x703` | PitDiag_fwinfo | ACU | 7 | `major`@0 `minor`@1 `patch`@2, `git_hash` u32 BE@3-6 | |
| `0x704` | PitDiag_health | ACU | 8 | `free_heap` BE@0, `min_free_heap` BE@2, **b4 task-bits**, `reset_cause`@5, `uptime_s`@6, `last_fault`@7 | **ungated, @1 s.** b4: bit0 control, bit1 can_rx, bit2 can_tx, bit3 diag. reset_cause 1 POR/2 PIN/3 SOFT/4 IWDG. last_fault 0/`F5`StackOvf/`F6`Malloc/`F1`HardFault |
| `0x705` | PitDiag_brake | ACU | 3 | `brake_pressure` u16 BE@0 (×0.1 bar), `brake_pct`@2 | pressure **0 until cal**; pct = raw·100/4095 |
| `0x706` | PitDiag_inverter_temps | ACU | 4 | b0 board, b1 pwrstg, b2 motor1, b3 motor2 | each **raw − 50 = °C**; raw `FF` = 205 °C = disconnect |
| `0x7E1` | PitDiag_ack | ACU | 1 | `enabled`@0 | reply to `0x7E0` |

### 2.2 ECU RX — inject these
| ID | Name | Bus | DLC | Layout | Example inject |
|----|------|-----|-----|--------|----------------|
| `0x461` | INV state | INV | ≥5 | `inv_state` = b4 `&0x7F`; `inv_error`(DEM) = b2 | Ready `{·· ·· 00 ·· 04 ·· ·· ··}`; hard-fault `{·· ·· 01 ·· 0B ·· ·· ··}` (state 11, DEM 1) |
| `0x463` | INV rpm | INV | 8 | `EMachine_Speed_erpm` 20-bit **signed LE @ bit44** = `(b5>>4)\|(b6<<4)\|(b7<<12)`, sign bit19 | 0:`b5=00`; +1:`b5=10`; +32767:`b5=F0 b6=FF b7=07`; −1:`b5=F0 b6=FF b7=FF`; +74560:`b5=00 b6=34 b7=12` |
| `0x464` | INV temps | INV | ≥4 | b0..b3 raw = °C+50 | 25/40/60 °C + open: `{4B 5A 6E FF}` |
| `0x466` | INV DC-bus | INV | ≥4 | `DCBus_Voltage_V` 10-bit **LE @ bit16** = `b2 \| (b3&3)<<8` | 300 V: `{·· ·· 2C 01 ·· ·· ·· ··}`; 125 V: `{·· ·· 7D 00 …}`. **Any `0x466` latches vconfig_ready** |
| `0x020` | ACU ok_precharge | ACU | 1 | `ok_precharge` bool @0 | ok `{01}`, not `{00}` |
| `0x12C` | ACU v_cell_min | ACU | 2 | `min_cell_mV` u16 **BE** @0 | 3600:`{0E 10}`, 3000:`{0B B8}`, 2800:`{0A F0}` |
| `0x4A0` | AMS_status | ACU | 8 | `fsm_state`@0 (**5 = Error**), `ams_ok`@1, `min_cell_mV` BE@4-5 | error `{05 00 00 00 0E 10 ·· ··}`; normal `{04 01 00 00 0E 10 ·· ··}` |
| `0x002` | BL_boot_trigger | ACU | 4 | `magic` u32 BE | `{B0 07 AD 12}` |
| `0x7E0` | PitDiag_cmd | ACU | 4 | `magic` u32 BE | enable `{DE AD BE EF}`, disable `{00 00 00 00}` |

---

## 3. Test blocks

Per case: **Setup → Inject → Expect → Pass**. Default precondition: cold-booted, pit-diag armed (`0x7E0=DEADBEEF`), all peer streams at ≥10 Hz unless a case says otherwise.

### A — Boot & bootloader recovery
- **A-001 — cold boot comes alive.** *Inject:* power-cycle the ECU. *Expect:* `0x100` and `0x704` appear on FDCAN2 with no other stimulus. *Pass:* both seen **≤ 1.5 s** after power; `0x704` byte5 `reset_cause = 1` (POR).
- **A-002 — pit-diag arms.** *Inject:* `0x7E0 {DE AD BE EF}`. *Expect:* `0x7E1 {01}` once, then `0x700/0x701/0x702/0x703/0x705/0x706` @100 ms. *Pass:* ACK + all six IDs present, period 100 ± 20 ms.
- **A-003 — pit-diag disarms.** *Inject:* `0x7E0 {00 00 00 00}`. *Expect:* `0x7E1 {00}`; the six stream IDs stop; **`0x704` keeps coming** @1 s. *Pass:* stream stops ≤200 ms; `0x704` continues.
- **A-004 — BL entry over CAN.** *Inject:* `0x002 {B0 07 AD 12}`. *Expect:* app TX stops; board enumerates in the bootloader. *Pass:* a `can-flasher` connect succeeds; re-flash + jump returns to the app.
- **A-005 — cold-boot reliability (#48 fix, `main.c` IRQ handoff).** *Inject:* power-cycle **6×**. *Expect:* every boot reaches the app. *Pass:* **6/6** show `0x100` ≤1.5 s + a live pit-diag stream when armed.

### B — FDCAN resilience *(the #48 regression gate — keep permanently)*
- **B-001 — TX live on both buses.** *Setup:* idle, no peers. *Expect:* `0x100` on FDCAN2 **and** `0x360`+`0x362` on FDCAN1, continuous. *Pass:* all three present ≥ their cadence for 30 s; no TX-dead.
- **B-002 — recover from bus-off.** *Inject:* force a fault on one bus (short / heavy error injection) for ~1 s, then clear. *Expect:* TX resumes on that bus after recovery. *Pass:* frames return ≤1 s after the bus clears.
- **B-003 — RX under load.** *Inject:* flood FDCAN1 with `0x461` at high rate, byte4 stepping 3→4. *Expect:* `0x700.inv_state` tracks the latest injected value. *Pass:* `inv_state` follows within 100 ms, no stream gap.
- **B-004 — 3-instance no-wedge (#48 root).** *Setup:* both buses loaded (inverter + AMS streams) ≥60 s. *Expect:* neither bus goes TX-dead. *Pass:* `0x100` and `0x360/0x362` continuous throughout.

### C — Heartbeat & AMS-stale contract
- **C-001 — `0x100` in every state.** *Setup:* walk the FSM slowly through 0→5 (`Setup-Active`, paced). *Expect:* `0x100` never gaps. *Pass:* max inter-frame gap **< 200 ms** in all six states (the `VcuStale` window).
- **C-002 — `0x100` carries DC-bus.** *Inject:* `0x466` DC-bus = 250 V (`{·· ·· FA 00 …}`). *Expect:* `0x100.dc_bus_voltage` = 250. *Pass:* LE u16 == 250 (passthrough, ± 0).

### D — Start / R2D FSM sequence *(`control.cpp:90-121`)*
- **D-001 — vconfig gate.** *Setup:* cold, no inverter. *Expect:* `0x700.fsm_state = 0`. *Inject:* one `0x466`. *Pass:* `fsm_state → 1` (Precharge). `[SIL: test_boot_sequence]`
- **D-002 — precharge gate.** *Setup:* `Setup-Precharge`. *Inject:* `0x020 {01}` (kept fresh). *Pass:* `fsm_state → 2`; with `0x020 {00}` it holds at 1.
- **D-003 — precharge timeout/retry.** *Setup:* Precharge, **withhold** `0x020`. *Expect:* after **`PrechargeTimeoutMs` = 10 s** it re-enters Precharge (no hang, no advance). *Pass:* `fsm_state` stays 1 across the 10 s boundary. `[SIL: test_precharge_no_ack]`
- **D-004 — R2D arms.** *Setup:* at WaitStartBrake (2). *Inject:* brake raw 1500 **+** START ≥50 ms. *Pass:* `fsm_state → 3`, `0x700.rtds_active = 1`, RTDS GPIO high.
- **D-005 — R2D needs both.** *Inject:* (a) START with brake 0; (b) brake 1500 with no START. *Pass:* both hold at `fsm_state = 2`.
- **D-006 — RTDS duration.** *Setup:* entered R2dDelay (3). *Expect:* after **2000 ms**, `fsm_state → 4`, `rtds_active = 0`. *Pass:* dwell = 2.0 s ± 50 ms.
- **D-007 — inverter-ready gate.** *Setup:* WaitInvStandby (4). *Inject:* `0x461` byte4 = 3 (Standby) → holds; = 4 (Ready) → advances. *Pass:* `fsm_state → 5` only on `inv_state == 4`.
- **D-008 — Active re-arm on contactor open.** *Setup:* `Setup-Active`. *Inject:* `0x020 {00}`. *Pass:* `fsm_state → 1` (Precharge) within one cycle. `[SIL: test_dynamic_states]`

### E — Inverter setpoints & fault recovery *(`control.cpp:128-168`, `inverter.cpp`)*
- **E-001 — mode word per state.** *Setup:* walk 0→5. *Expect:* `0x360` byte2 = `01` (states 0-3), `04` (state 4), `06` (state 5). *Pass:* matches at each state. `[SIL: test_inverter]`
- **E-002 — torque tracks APPS.** *Setup:* `Setup-Active`, `0x461`=Ready. *Inject:* APPS both channels to 50% (apps1 raw ≈ 2920, apps2 raw ≈ 2685). *Expect:* `0x700.torque_pct ≈ 50`; `0x362 = {00 00 95 FF}` (−107). *Pass:* `torque_pct` 50 ± 2, `0x362` matches the negated map ± 1 LSB. `[SIL: test_active_torque_and_deadband]`
- **E-003 🆕 — hard-fault recovery.** *Setup:* any non-AmsError state, streams live. *Inject:* `0x461` byte4 = `0B` (11). *Expect:* `0x360` byte2 = **`0D`**. *Pass:* `0x360[2] == 0x0D` for as long as `inv_state == 11`. `[SIL: test_inverter_fault_recovery]`
- **E-004 🆕 — soft-fault recovery.** *Inject:* `0x461` byte4 = `0A` (10). *Expect:* `0x360` byte2 = **`13`**. *Pass:* `0x360[2] == 0x13`. `[SIL: test_inverter_fault_recovery]`
- **E-005 🆕 — recovery clears & FSM advances.** *Setup:* at WaitInvStandby (4). *Inject:* `0x461`=11 (→`0x360`=`0D`), then step byte4 → 3 → 4. *Expect:* mode follows then Ready handling; FSM reaches Active. *Pass:* `fsm_state` reaches 5 (does **not** stall at 4).
- **E-006 🆕 — fault cuts torque.** *Setup:* `Setup-Active`, APPS at 50%. *Inject:* `0x461` byte4 = `0B` (11). *Expect:* `0x362 = {00 00 00 00}` **and** `0x360[2] = 0D`. *Pass:* torque request 0 within one cycle.
- **E-007 🆕 — AmsError suppresses recovery.** *Setup:* drive to AmsError (F-002), keep it fresh. *Inject:* `0x461` byte4 = `0B`. *Expect:* `0x360` byte2 = **`01`** (Off), **not** `0D`. *Pass:* `0x360[2] == 0x01`.

### F — AMS integration *(real AMS, `ECU_STUB_NO_AMS` OFF)*
- **F-001 🆕 — real precharge gate.** *Setup:* `Setup-Precharge` with the real AMS. *Expect:* FSM holds at Precharge until the AMS asserts `0x020 {01}`, then advances. *Pass:* `fsm_state` 1→2 only on real ok_precharge.
- **F-002 🆕 — AMS-error inhibit.** *Setup:* any state. *Inject:* `0x4A0 {05 00 …}` @≥10 Hz. *Expect:* `fsm_state → 6` (AmsError), `0x360[2] = 01`, `0x362 = 0`, ERR LED on. *Then inject:* `0x4A0 {04 01 …}`. *Expect:* `fsm_state → 0`. *Pass:* both transitions; **note** error needs fresh `0x4A0` (stale → re-arm, not error). `[SIL: test_ams_error]`
- **F-003 🆕 — low-cell derate.** *Setup:* `Setup-Active`, APPS 100% (raws ≥ AdcMax). *Inject:* `0x12C {0B B8}` (3000 mV), kept fresh. *Expect:* derate ≈ 0.32 → `0x700.torque_pct ≈ 32`, `0x362 ≈ {00 00 C5 FF}` (−59). *Pass:* `torque_pct` 32 ± 3 (vs 100 at 3600 mV). `[SIL: test_cell_v_derate]`
- **F-004 🆕 — AMS-stale fail-safe.** *Setup:* `Setup-Active`. *Inject:* **stop all** `0x020/0x12C/0x4A0` for > 200 ms. *Expect:* `ok_precharge` derives false → `fsm_state → 1`. *Pass:* re-arm within ~210 ms of the last AMS frame; `0x700.ok_precharge` bit clears.
- **F-005 ⚠ — `0x4A0` actually emitted.** *Action:* scope the real car AMS for `0x4A0`. The ECU reads it for the error-inhibit; the bench saw `0x4A2/0x4A4` but **not** `0x4A0`. *Pass:* confirm presence + that byte0 carries the FSM state (5 = Error) before relying on F-002 in-car.

### G — FSAE plausibility cuts *(`control.cpp:49-78`)*
- **G-001 — EV.2.3 brake+throttle latch.** *Setup:* `Setup-Active`. *Inject:* brake raw > 3000 (e.g. 3500) **and** APPS → `torque_pct > 25`. *Expect:* `0x700.ev_2_3 = 1`, `torque_pct = 0`. *Clear:* brake < 3000 **and** `torque_pct < 5`. *Pass:* latches, and clears only on both conditions. `[SIL: test_ev_2_3]`
- **G-002 — T.11.8.9 APPS disagreement.** *Setup:* `Setup-Active`. *Inject:* apps1 ≈ 60%, apps2 ≈ 40% (|Δ| > 10%) held **≥ 100 ms**. *Expect:* `0x700.t11_8_9 = 1`, `torque_pct = 0`. *Inject (b):* same Δ for < 100 ms → no cut. *Pass:* cut after the 100 ms window, not before. `[SIL: test_t11_8_9_window]`
- **G-003 — APPS agreement floor.** *Inject:* both APPS ≤ 8% (`AppsAgreementPct`). *Pass:* `torque_pct = 0`. `[SIL: test_apps_pct]`
- **G-004 — deadband.** *Inject:* APPS → 8% then → 95%. *Pass:* `torque_pct = 0` below 10%, `= 100` above 90%. `[SIL: test_active_torque_and_deadband]`

### H — Pit-diag stream fidelity
- **H-001 — `0x700` status mirror.** *Setup:* known FSM/inverter/flags. *Pass:* `fsm_state`, `inv_state`, the 5 flag bits, and `torque_pct` all match the commanded state. (Do **not** assert `torque_cmd` — hardwired 0.)
- **H-002 — `0x701` pedals mirror.** *Inject:* apps1 raw 2920, apps2 raw 2685, brake 1500. *Expect:* `0x701` raws BE-match; `apps1_pct ≈ 50`, `apps2_pct ≈ 50` per `Apps*AdcMin/Max`. *Pass:* raws exact, % within ± 2.
- **H-003 — cadence + gate.** *Pass:* armed → six IDs @100 ± 20 ms; `0x7E0 {0}` stops them ≤200 ms while `0x704` continues.

### I — Health & liveness (`0x704`, `diag_task.cpp`)
- **I-001 — ungated @1 s.** *Setup:* never arm pit-diag. *Pass:* `0x704` still streams @1 s ± 100 ms from boot.
- **I-002 — reset_cause.** *Inject:* POR (power-cycle), pin reset, and a software / IWDG reset. *Pass:* `0x704` byte5 = 1 / 2 / 3-4 respectively.
- **I-003 — task liveness.** *Setup:* continuous RX on both buses. *Expect:* `0x704` byte4 bits0-3 all set. *Pass:* all four set under load. **Known:** the `can_rx` bit (bit1) clears on a *quiescent* bus (a healthy blocked RX task reads idle) — drive continuous RX to model the car, don't flag idle-bus as a fault.
- **I-004 — uptime climbs.** *Pass:* `0x704` byte6 `uptime_s` increments monotonically (wraps at 255 — it's a u8).
- **I-005 — no heap leak.** *Setup:* 5-min armed soak. *Pass:* `0x704` `min_free_heap` (bytes 2-3) stops falling and holds.

### J — Telemetry accuracy & soak
- **J-001 🆕 — rpm decode (bit-44 signed).** *Inject `0x463`* per §2.2 for {0, +1, +32767, −1, +74560}. *Expect:* `0x702.inv_rpm` (s32 BE @2-5) = each value. *Pass:* all five match, **especially −1** (`FF FF FF FF`) and +74560. `[SIL: test_inverter_rx]`
- **J-002 🆕 — temps decode (−50 offset).** *Inject `0x464` {4B 5A 6E FF}`.* *Expect:* `0x706` = board 25, pwrstg 40, motor1 60, motor2 **205** (=`FF`, disconnect). *Pass:* each = raw − 50.
- **J-003 — DC-bus mirror.** *Inject:* `0x466` = 300 V. *Pass:* `0x702.dc_bus_voltage` (BE) = 300 **and** `0x100` (LE) = 300.
- **J-004 — drive soak.** *Setup:* `Setup-Active`, APPS ~50%, 60 s. *Pass:* `0x100` alive throughout, `0x700.fsm_state` stays 5, `0x704.uptime_s` climbs, no reset (`reset_cause` unchanged).
- **J-005 — reflash robustness.** *Inject:* `0x002` → flash → jump, **2×**. *Pass:* each cycle boots clean; a bus-wedge mid-write recovers via `ip link` re-up + retry (BL stays alive — "no app" ≠ brick).

---

## 4. Implementation notes
- **Accuracy is the contract.** Every value above is from `ecu_config.hpp` / `control.cpp` / the `.def` files on `dev`. If a constant moves, the case moves with it; the generated `docs/dbc/ecu.dbc` (dbcinator) is the authoritative field-layout source the rig should load.
- **Bring-up stubs stay off-by-default compile flags** (`ECU_STUB_*`, `ECU_DEBUG_INV_BRIDGE`). Drive the real gates via the rig's CAN + DAC + GPIO inject so the FSM exercises its real freshness/gating. `ECU_DEBUG_INV_BRIDGE` mirrors the inverter setpoints onto FDCAN2 (`0x560/0x562`) if your sniffer only sees the ACU bus. `TorqueCap`/`StubBrakeRaw` at defaults (100/0).
- **Cases needing a firmware hook** (raise to firmware, don't work around on the rig): `I-002` IWDG / software-reset injection, `F-004`/`I-005` if a deterministic stop-stream / leak probe is wanted, and surfacing `0x700.torque_cmd` (currently hardwired 0 — deferred inverter unit-map).
- **The bench harness lives in `IFS08_HIL`;** this spec + the DBC live here (the firmware owns its wire contract). Tracking: #94.
