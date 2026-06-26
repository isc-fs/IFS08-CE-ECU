# IFS08-CE-ECU — HIL Bench Test Plan

Authoritative behavioural spec for the **HIL team** to implement against in `IFS08_HIL`.
Every case below is pinned to the firmware source on `dev` (file:symbol noted); the
HIL team owns the rig wiring and the test harness — this document owns *what the
firmware must do* and *how to observe it*.

- **Firmware under test:** `dev` (lean FreeRTOS C++17 app, links over the CAN bootloader at `0x08020000`).
- **ECU node id:** `0x01`. Buses: **FDCAN1 = INV** (NX/EMC inverter), **FDCAN2 = ACU** (AMS + pit-tool), both 500 kbit/s classic.
- **Relationship to SIL:** the `ecu::Controller` core (FSM + every plausibility cut + the RX decoders) is already unit-tested host-side (`tests/sil`, 135 checks). **HIL is not a re-run of the SIL** — it verifies the same behaviour *through the real CAN wire, real timing, the real RTOS, and the real/ simulated peers*: encode/decode parity, cadence, gate latencies, boot/BL, and resilience. Cases that have a SIL sibling are tagged `[SIL: test_x]`.
- **NEW this session** (no HIL coverage yet) are tagged **🆕** — prioritise these: inverter fault-recovery (E-003..E-007), real-AMS integration (F-*), and rpm/temps telemetry (J-001/J-002).

---

## 1. Test interface

**Inject (stimulus):**
| Boundary | How | Notes |
|---|---|---|
| Inverter feedback | CAN frames on **FDCAN1**: `0x461` App_State, `0x463` rpm, `0x464` temps, `0x466` DC-bus | the inverter sim |
| AMS / ACU | CAN frames on **FDCAN2**: `0x020` ok_precharge, `0x12C` v_cell_min, `0x4A0` AMS status | the acu inject |
| Pedals / brake | analog → ADC3 (brake PF7, APPS1 PF8, APPS2 PF9) | raw 12-bit |
| Start button | GPIO PB5 (active-high) | |
| Pit-diag arm | CAN `0x7E0` payload `DE AD BE EF` (enable) / `0` (disable) on FDCAN2 | ACK on `0x7E1` |
| Bootloader entry | CAN `0x002` payload `B0 07 AD 12` on FDCAN2 | app → BL reboot |

**Observe (response):** the ECU's own TX —
- **`0x100`** VCU_heartbeat (FDCAN2, every state, every 10 ms cycle) — DC-bus voltage.
- **`0x360`/`0x362`** inverter setpoints (FDCAN1) — `App_State_Req` mode word @ byte2; `Torque_Nm_Req` s16 LE @ bytes 2-3 (**negated**).
- **pit-diag stream `0x700`-`0x706`** (FDCAN2, @100 ms, gated by `0x7E0`) — the primary observability.
- **`0x704`** health — streamed **ungated** @1 s (visible from boot, survives a ControlTask hang).

### Pit-diag observable map
| ID | Frame | Fields used by this plan |
|----|-------|--------------------------|
| `0x700` | status | `fsm_state`, `inv_state`, `ev_2_3`, `t11_8_9`, `rtds_active`, `ok_precharge`, `start_button`, `torque_pct`, `v_cell_min_mV` |
| `0x701` | pedals | `apps1_raw/pct`, `apps2_raw/pct`, `brake_raw` |
| `0x702` | inverter | `dc_bus_voltage`, `inv_rpm` (signed), `inv_error` |
| `0x704` | health | `free_heap`, task-liveness bits (control/can_rx/can_tx/diag), `reset_cause`, `uptime_s`, `last_fault` |
| `0x705` | brake | `brake_pressure` (bar, cal pending), `brake_pct` |
| `0x706` | temps 🆕 | `temp_board/pwrstg/motor1/motor2_degC` (raw byte − 50 = °C) |

### FSM reference (`control.cpp`, `ecu::CtrlState`)
`WaitInvVdcConfig → Precharge → WaitStartBrake → R2dDelay → WaitInvStandby → Active`, plus `AmsError` (entered from any state, latched). Gates: vconfig=`0x466` seen · precharge=`0x020` · R2D=`start && brake>BrakeArmRaw(900)` · RTDS=`R2dSoundMs(2000)` · ready=`inv_state==InvReadyState(4)`. Inverter mode per state: Off `0x01` pre-R2D · Ready `0x04` at WaitInvStandby · TorqueEnable `0x06` in Active.

---

## 2. Test blocks

> Convention per case: **Inject** (stimulus) → **Expect** (observable) → **Pass** (criterion). Default precondition unless stated: cold-booted, pit-diag armed, no AMS error, inverter feeding `inv_state=Standby(3)`.

### A — Boot & bootloader recovery
| ID | Intent | Inject → Expect → Pass |
|----|--------|------------------------|
| A-001 | Cold boot comes alive | power-cycle → `0x100` + `0x704` appear on FDCAN2 → both seen ≤ 1.5 s; `0x704.reset_cause` = power-on |
| A-002 | Pit-diag arms | `0x7E0=DEADBEEF` → `0x7E1(enabled=1)` + `0x700`-`0x706` @100 ms → ACK + stream present |
| A-003 | Pit-diag disarms | `0x7E0=0` → `0x7E1(0)`; `0x700`-`0x703/705/706` stop, **`0x704` keeps streaming** |
| A-004 | BL entry over CAN | `0x002 = B0 07 AD 12` → app streams stop, board re-flashable over CAN → flash succeeds |
| A-005 | Cold-boot reliability (#48 fix, `main.c` `__enable_irq` handoff) | 6× power-cycle → every boot reaches the app (`0x100`+pit-diag) → **6/6** |

### B — FDCAN resilience  *(the #48 regression gate — keep as a permanent gate)*
| ID | Intent | Inject → Expect → Pass |
|----|--------|------------------------|
| B-001 | TX live on both buses | idle → `0x100` on FDCAN2 **and** `0x360`/`0x362` on FDCAN1, continuous → both present, no TX-dead |
| B-002 | Recover from bus-off | force a bus fault on a bus, then clear → TX resumes after recovery → frames return |
| B-003 | RX under heavy load | flood FDCAN1 with inverter traffic → `0x700.inv_state` still tracks the injected `0x461` → no decode stalls |
| B-004 | 3 concurrent FDCAN instances don't wedge TX (#48 root cause) | run FDCAN1+FDCAN2 both loaded for ≥ 60 s → no TX-dead on either → continuous |

### C — Heartbeat & AMS-stale contract
| ID | Intent | Inject → Expect → Pass |
|----|--------|------------------------|
| C-001 | `0x100` in **every** FSM state | walk FSM through each state → `0x100` never gaps > 200 ms (the AMS `VcuStale` window, `AmsStaleMs`) → gap < 200 ms throughout |
| C-002 | `0x100` carries DC-bus | inject `0x466 dc_bus=X` → `0x100.dc_bus_voltage == X` |

### D — Start / R2D FSM sequence  *(`control.cpp:90-121`)*
| ID | Intent | Inject → Expect → Pass | SIL |
|----|--------|------------------------|-----|
| D-001 | vconfig gate | no `0x466` → `fsm=WaitInvVdcConfig`; inject `0x466` → `Precharge` | `test_boot_sequence` |
| D-002 | precharge gate | at Precharge, no `0x020` → holds; inject `0x020(ok=1)` → `WaitStartBrake` | |
| D-003 | precharge timeout/retry | withhold `0x020` ≥ 10 s (`PrechargeTimeoutMs`) → re-enters Precharge (retry, not stuck) | `test_precharge_no_ack` |
| D-004 | R2D arms | at WaitStartBrake, `start=1` + `brake_raw>900` → `R2dDelay`, `rtds_active=1` | |
| D-005 | R2D needs **both** | start alone (brake<900) → holds; brake alone (no start) → holds | |
| D-006 | RTDS duration | at R2dDelay → after `R2dSoundMs=2000 ms` → `WaitInvStandby`, `rtds_active=0` → 2.0 s ± tick | |
| D-007 | inverter-ready gate | at WaitInvStandby, `inv_state≠4` → holds; `inv_state=4` → `Active` | |
| D-008 | Active re-arm on contactor open | at Active, `0x020(ok=0)` → back to `Precharge` | `test_dynamic_states` |

### E — Inverter setpoints & fault recovery  *(`control.cpp:128-168`, `inverter.cpp`)*
| ID | Intent | Inject → Expect → Pass | SIL |
|----|--------|------------------------|-----|
| E-001 | mode word per state | walk FSM → `0x360.byte2` = `0x01` pre-R2D, `0x04` at WaitInvStandby, `0x06` in Active | `test_inverter` |
| E-002 | torque tracks APPS | at Active (`inv_state=4/6`), sweep APPS → `0x362` `Torque_Nm_Req` follows, **negated** (forward = negative) | `test_active_torque_and_deadband` |
| **E-003** 🆕 | **hard-fault recovery** | inject `inv_state=11` (any non-AmsError state) → `0x360.byte2 = 0x0D` | `test_inverter_fault_recovery` |
| **E-004** 🆕 | **soft-fault recovery** | inject `inv_state=10` → `0x360.byte2 = 0x13` | `test_inverter_fault_recovery` |
| **E-005** 🆕 | **recovery clears & FSM advances** | inv_state `11→0x0D`, then feed `3`(standby)→`4`(ready) → FSM walks WaitInvStandby→Active → reaches Active (does **not** stall) | |
| **E-006** 🆕 | **fault cuts torque** | at Active, inject `inv_state=11` → `0x362`=0 **and** `0x360=0x0D` | |
| **E-007** 🆕 | **AmsError suppresses recovery** | in AmsError + `inv_state=11` → `0x360 = Off(0x01)`, **not** `0x0D` | |

### F — AMS integration  *(real AMS on the bus, `ECU_STUB_NO_AMS` OFF)*
| ID | Intent | Inject → Expect → Pass | SIL |
|----|--------|------------------------|-----|
| **F-001** 🆕 | real precharge gate | with real AMS, no `0x020(ok=1)` → FSM holds at Precharge; on real precharge-OK → advances | |
| **F-002** 🆕 | AMS-error inhibit | inject `0x4A0 fsm_state=5` (`AmsFsmError`) → `fsm=AmsError` from any state, `0x360=Off`, no torque; clear (`≠5`) → exits to WaitInvVdcConfig | `test_ams_error` |
| **F-003** 🆕 | low-cell derate | inject `0x12C v_cell_min` below knee (`3500`→`2800`) → in Active, `0x362` torque scaled down vs full | `test_cell_v_derate` |
| **F-004** 🆕 | AMS stale → fail-safe | stop `0x020`/`0x4A0` > 200 ms (`AmsStaleMs`) → `ok_precharge` goes stale (false) → Active re-arms to Precharge | |
| F-005 | `0x4A0` actually emitted | confirm the car AMS sends `0x4A0` (the ECU reads it for the error latch) — **open item flagged at bench**: seen `0x4A2/0x4A4` but not `0x4A0` | |

### G — FSAE plausibility cuts  *(`control.cpp:49-78`)*
| ID | Intent | Inject → Expect → Pass | SIL |
|----|--------|------------------------|-----|
| G-001 | EV.2.3 brake+throttle latch | `brake_raw>3000` + APPS→`torque>25%`(`Ev23SetPct`) → `0x700.ev_2_3=1`, torque 0; clear only when `brake<3000` + `torque<5%`(`Ev23ResetPct`) | `test_ev_2_3` |
| G-002 | T.11.8.9 APPS disagree | `|apps1−apps2|>10%`(`AppsDisagreePct`) held ≥ 100 ms(`AppsDisagreePersistMs`) → `0x700.t11_8_9=1`, torque 0; < 100 ms → no cut | `test_t11_8_9_window` |
| G-003 | APPS agreement floor | both APPS ≤ 8%(`AppsAgreementPct`) → torque 0 | `test_apps_pct` |
| G-004 | deadband | APPS < 10% → torque 0; > 90% → torque 100% | `test_active_torque_and_deadband` |

### H — Pit-diag stream fidelity
| ID | Intent | Inject → Expect → Pass |
|----|--------|------------------------|
| H-001 | `0x700` status mirror | drive known FSM/inverter/flag states → `0x700` fields match |
| H-002 | `0x701` pedals mirror | inject APPS/brake raw → `0x701` raw + computed % match the cal (`Apps*AdcMin/Max`) |
| H-003 | cadence | armed → `0x700`-`0x706` at 100 ms ± tick |
| H-004 | gate | `0x7E0=0` stops `0x700`-`0x703/705/706`; `0x704` continues |

### I — Health & liveness (`0x704`, `diag_task.cpp`)
| ID | Intent | Inject → Expect → Pass |
|----|--------|------------------------|
| I-001 | ungated @1 s | never arm pit-diag → `0x704` still streams @1 s from boot |
| I-002 | reset_cause | power-on vs IWDG-reset vs pin-reset → `0x704.reset_cause` distinguishes |
| I-003 | task liveness | under continuous RX, all 4 task bits set. **Note:** the `can_rx` bit clears on a *quiescent* bus (a healthy blocked RX task reads idle) — drive continuous RX to model the car |
| I-004 | uptime climbs | `0x704.uptime_s` increments monotonically |
| I-005 | no heap leak | 5-min soak → `0x704.free_heap` / `min_free_heap` stable |

### J — Telemetry accuracy & soak
| ID | Intent | Inject → Expect → Pass | SIL |
|----|--------|------------------------|-----|
| **J-001** 🆕 | rpm decode (bit-44, signed) | inject `0x463` `EMachine_Speed_erpm` = {0, +1, +32767, −1, +74560} → `0x702.inv_rpm` matches each (esp. **negative**) | `test_inverter_rx` |
| **J-002** 🆕 | temps decode (−50 offset) | inject `0x464` bytes → `0x706` temps = byte − 50 °C; raw `0xFF` → 205 °C (disconnect sentinel) | |
| J-003 | DC-bus mirror | inject `0x466` → `0x702.dc_bus_voltage` + `0x100.dc_bus_voltage` match | |
| J-004 | drive soak | 60 s held in Active driving → `0x100` alive, `fsm` stays Active, `uptime` climbs, no reset | |
| J-005 | reflash robustness | 2× BL reflash (`0x002` → flash → jump) → each boots clean; bus-wedge mid-write recovers via re-up + retry | |

---

## 3. Notes for implementation

- **Accuracy over coverage:** every threshold/ID above is from `ecu_config.hpp` / `control.cpp` on `dev`. If a constant changes, this doc + the cases move with it (the `dbcinator` DBC is the wire source-of-truth for field layouts).
- **Bring-up stubs are off-by-default compile flags** (`ECU_STUB_*`, `ECU_DEBUG_INV_BRIDGE`) — a flight image has none. For HIL you'll likely build with the rig's own inject path (CAN + DAC + GPIO) rather than the stubs, so the FSM exercises its real gates. `ECU_DEBUG_INV_BRIDGE` is handy to mirror FDCAN1 onto FDCAN2 if your sniffer only sees the ACU bus.
- **Two values now live in `ecu_config.hpp`** for on-stands work: `TorqueCap` (default 100 = no cap) and `StubBrakeRaw` (default 0). Both must be at their defaults for a representative test unless a case explicitly needs the clamp.
- **Prioritise the 🆕 cases** — they're the behaviours brought online this session (inverter recovery, real-AMS gating, rpm/temps) and have **no** HIL coverage yet.
- **Deferred / needs a firmware hook:** fault-injection for some I/F cases and a stop-stream hook for staleness may need a small firmware affordance — flag back to the firmware team rather than working around it on the rig.
