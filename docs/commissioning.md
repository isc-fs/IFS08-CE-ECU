# Commissioning — on-car ECU bring-up

The firmware ships with `COMMISSION`-tagged placeholders in
`Core/Inc/app/ecu_config.hpp` (carried from the legacy VCU) that **must be
re-measured on the assembled car** before any drive. Everything below is done
over CAN through the pit-diag stream — no UART, no SWD.

## Prerequisites

- ECU flashed and booted; `0x100` streaming on **FDCAN2 / ACU @ 68.75 %**.
- A CAN tool on the ACU bus (the pit tool, or `candump`/`cansend` decoding against
  [`docs/dbc/ecu.dbc`](dbc/ecu.dbc)).
- Pit-diag enabled: send **`0x7E0`** with payload **`DE AD BE EF`** (bytes 0–3).
  The ECU acks `0x7E1 = 1` and streams `0x700`–`0x708`. Disable with `0x7E0 = 0`.
  (`0x706` = inverter temps, `0x707` = DV diagnostics, `0x708` = inverter L1/L2
  fault layers. `0x704` health is **ungated** — it streams from `DiagTask` even
  with pit-diag off.)

---

## 1. Pedal calibration (APPS + brake)

Calibration is **runtime data** (#169). It lives in the bootloader NVM sector,
survives a power cycle **and a firmware reflash**, and can be set over CAN
without rebuilding anything.

Two ways to do it. Use the wizard when it exists; the manual route is the
fallback and is what to use today.

### Is a calibration actually in force?

Read **`0x704` byte 5 bits 4–5** (`cal_status`) — **ungated**, so this works
without arming pit-diag:

| Value | Meaning |
|---|---|
| `Defaults` | nothing stored; running the compile-time values |
| `Loaded` | a stored calibration is in force |
| `InvalidFellBack` | a record was found and **rejected**; running defaults |
| `BadVersionFellBack` | record from an unknown layout; running defaults |

`InvalidFellBack` is the one that matters: the ECU is deliberately ignoring a
stored calibration. Do not drive on it assuming your numbers applied.

### 1a. With MingoCAN (preferred — requires isc-fs/can-flasher#534)

> **Not available yet.** The ECU side is complete; the MingoCAN client is not
> built. Until #534 ships, use 1b.

Car on stands, **wheels off the ground**, LV on, **TS off**. The ECU refuses to
open a session unless the tractive system is down, the FSM is not in `Active`,
commanded torque is zero and the motor is stopped — and it says which of those
failed.

The wizard walks five captures: accelerator released, accelerator at the
mechanical stop, **accelerator at mid travel**, brake released, brake pressed
firmly. It then shows old-vs-new for every value, you confirm, and it commits.

**The mid-travel capture is not optional padding.** Rest and full endpoints
cannot detect what T.11.8.9 guards against — both channels normalise to 0–100 %
by construction, so they agree at the endpoints however badly they diverge in
between. Mid travel is the only place channel agreement is really measured.

A valid commit applies **immediately** — no reset. Sweep both pedals afterwards
and confirm `apps1_pct`/`apps2_pct` on `0x701` read 0 at rest and 100 at full.

### 1b. Manually (works today)

**Read:** with pit-diag armed, watch **`0x701 PitDiag_pedals`** — `apps1_raw`,
`apps2_raw`, `brake_raw` (12-bit).

**Measure**, letting each reading settle:

| Pedal position | Record | Into |
|---|---|---|
| Accelerator released | `apps1_raw`, `apps2_raw` | `Apps1AdcMin`, `Apps2AdcMin` |
| Accelerator at the stop | `apps1_raw`, `apps2_raw` | `Apps1AdcMax`, `Apps2AdcMax` |
| Brake released | `brake_raw` | `BrakeRestRaw` |
| Brake pressed firmly | `brake_raw` | `BrakePressedRaw` |

Then set the two intermediate brake thresholds from the measured span — this is
what the wizard does automatically, and it is what finally retires the IFS06
placeholders:

```
span          = BrakePressedRaw - BrakeRestRaw
BrakeArmRaw   = BrakeRestRaw + span * 10 / 100    # brake-to-arm for R2D
BrakeDvHardRaw= BrakeRestRaw + span * 60 / 100    # DV R2D gate, and the 0x505
                                                  # verdict sent to the uDV
```

> ⚠️ `BrakeDvHardRaw` is broadcast to the uDV as this ECU's verdict on whether
> the EBS is really braking. Sanity-check 60 % against what the EBS actually
> produces before any driverless running.

**Margin:** set each `*AdcMin` a little **above** the true rest reading so a
released pedal lands at exactly 0 %, and leave headroom below `*AdcMax`. Note
this costs pedal travel — the current +14/+13 count insets are about 1.6 % of
span, which is part of why torque onset sits above the nominal deadband.

**Sanity:** both APPS channels must track each other across the whole sweep, not
just at the ends. Watch `apps1_pct` and `apps2_pct` together through a slow
sweep; more than a few percent apart at mid travel means one channel is
mis-calibrated or a sensor is non-linear, and T.11.8.9 will cut torque in
service.

**Apply + verify:**
1. Set the constants in `Core/Inc/app/ecu_config.hpp`.
2. Rebuild and flash over CAN. **Confirm BL recovery first**
   (`0x002`/`0xB007AD12` round-trip); never power-cut mid-write.
3. Re-arm pit-diag and sweep: `apps1_pct`/`apps2_pct` read **0 % at rest, 100 %
   at full**, tracking within 10 % throughout, and `brake_pct` on `0x705` reads
   **0 with the brake released** (it reads ~13 % until `BrakeRestRaw` is set —
   the percentage falls back to full-ADC-range scaling while the span is
   unknown).

### What is NOT calibratable

The deadbands, the FSAE plausibility percentages (`AppsDisagree*`) and
every timing stay compile-time. They are design decisions, not per-car
measurements, and an operator must not be able to move them.

`brake_pressure` on `0x705` reports real **bar** (0.1 bar units). The sensor is a
Variohm EPT1400, 40 bar, ratiometric 0.5–4.5 V, behind a 2/3 divider (R8 1k /
R9 2k). Because both the divider and Vref are known this is an **absolute map**
— it needs no calibration at all:

```
0 bar  -> 414 counts       40 bar -> 3723 counts      82.7 counts/bar
```

That also makes the three brake thresholds reviewable in physical units:

| Constant | Counts | Pressure | Gates |
|---|---|---|---|
| `BrakeArmRaw` | 750 | 4.1 bar | R2D arm (light press) |
| `BrakeDvHardRaw` | 2500 | 25.2 bar | DV R2D + the `0x505` verdict to the uDV |
| `BrakePressedRaw` | 3000 | 31.3 bar | brake full travel — top of the `brake_pct` scale |

> ⚠️ A released-pedal reading of ~560 counts is **1.8 bar**, not 0. That is either
> genuine residual pressure in the system or sensor offset, and it shifts every
> reported pressure by the same amount. Worth resolving on the car.

---

## 1c. Pack internal resistance (`CellIrMilliOhm`) — needs one acceleration run

The low-cell derate does **not** run on the voltage the AMS reports. It runs on
an estimated open-circuit voltage:

```
v_ocv = v_cell_min + I_pack × R_cell
```

Why: a cell's terminal voltage under load mostly measures how hard you are
accelerating, not how empty the pack is. At 300 A and 1 mΩ each series element
sags 300 mV, so a healthy cell resting at 3.0 V reads 2.7 V at the end of the
straight and recovers the instant the driver lifts. Derating on that makes the
derate a function of throttle, and closes an undamped loop — cut torque, current
falls, voltage recovers, derate lifts, sag returns. Adding the ohmic drop back
cancels it by construction.

`CellIrMilliOhm` is the only number in that expression that has to come from the
car. **No dyno required** — one acceleration run is enough.

1. Enable the pit-diag stream (`0x7E0` = `0xDEADBEEF`).
2. Log `0x709` `PitDiag_cell` through one full-throttle acceleration.
3. Plot `raw_mV` and `est_ocv_mV` on the same axes.

| What the trace does | Meaning | Action |
|---|---|---|
| `raw_mV` dips hard, `est_ocv_mV` **flat** | correct | done |
| `est_ocv_mV` still dips under load | under-compensating | **raise** `CellIrMilliOhm` |
| `est_ocv_mV` humps upward under load | over-compensating | **lower** it |

`comp_mV` on the same frame shows how much correction is being applied, so the
two effects can be read apart rather than inferred from the sum.
`tools/packlog.py` fits the same number off a log if you would rather not eyeball it.

> ⚠️ **Shipped at 1 mΩ, deliberately low.** Under-compensating leaves some sag in
> the estimate and derates *earlier* — the safe direction. Over-compensating
> hides a genuinely empty pack. Do not raise it above what the trace supports.

Three guards exist because compensation is the one correction that makes the
pack look *healthier*, and all three fail towards derating sooner:

- `0x135` stale (>200 ms) → no compensation at all, derate on the raw voltage.
  Visible as `compensated = 0` on `0x709`.
- The correction is clamped to `CellIrCompMaxMv` (500 mV).
- A **raw** cell at/below `CellVRawFloorMv` (2400 mV) goes straight to the floor
  derate whatever the estimate says. Visible as `raw_floor = 1`.

### The curve itself

A **cap**, not a scale factor: `torque = min(torque, cap)`. At a 68 % cap, half
pedal still gives exactly 50 % — only the top of the range is clipped. (It
multiplied demand until #177, which rescaled the whole pedal and cost the driver
resolution across the entire travel in order to limit a peak.)

| min cell (estimated OCV) | torque cap |
|---|---|
| ≥ 2800 mV | 100% |
| 2700 | 68% |
| 2600 | 36% |
| ≤ 2500 | 5% (limp-home) |

> ⚠️ `CellVDerateFloorMv` (2500 mV) is **`COMMISSION`**: it must sit at or above
> the AMS undervoltage cut, which lives in the AMS firmware and is not visible
> from this repo. If the AMS opens above 2500 mV the AIRs drop with the car still
> commanding 5% torque instead of coasting down. Confirm it against the AMS
> config and the cell datasheet before any endurance run.

---

## 1d. Motor thermal cap — check the sensors before you trust the limit

The motor limit is **80 °C**. The cap starts backing torque off at **70 °C** and
reaches its **20 % floor at 80 °C** — the limit is where the floor *is*, not
where the cap starts, because a limiter that waits for the limit has already let
the winding get there.

| motor temp | torque cap |
|---|---|
| ≤ 70 °C | 100% |
| 72 | 84% |
| 75 | 60% |
| 78 | 36% |
| ≥ 80 | 20% |

It is a **cap, not a gain**: anything below the cap passes through untouched, so
at 75 °C half pedal still gives exactly 50%. Only the top of the range is clipped.

Heat goes as torque squared, so the 20% floor is ~4% of the heating — the motor
cools under any realistic load while the car can still drive off track.

### The part that actually needs checking

`0x464` carries two motor sensors and **both failure modes are silent**:

| raw byte | decodes to | what it really means |
|---|---|---|
| `0xFF` | 205 °C | **sensor disconnected** — would slam the cap to the floor if read literally |
| `0` | −50 °C | **no `0x464` ever received** — reads as a very cold, very healthy motor |

The second is the dangerous one, and it is the *default state at boot*. Both are
rejected, and losing every sensor does **not** mean "no limit" — the cap holds at
`MotorTempUnknownCapPct` (60%), enough to drive, not enough to cook a motor
nobody is watching.

**Before any run, confirm on `0x706`:**

1. `temp_s1_valid` and `temp_s2_valid` are both **1**
2. `temp_unknown` is **0**
3. `motor_temp_used_degC` tracks the two raw temps and is plausible (ambient at
   key-on, not −50 and not 205)

If `temp_unknown = 1` the car will run at 60% and feel flat. That is the
protection working, not a fault in the derate — go and find the temperature
sensor.

> `motor_temp_used_degC` is the **hottest valid** sensor, filtered (~2.5 s). It
> is deliberately not simply `max(motor1, motor2)`: a disconnected sensor is
> excluded, so with one sensor dead this tracks the surviving one rather than
> the 205 °C sentinel.

---

## 2. R2D / RTDS test without brake pressure / start wiring (`StubBrakeRaw` / `StubStart`)

R2D arms on `start_button && brake_raw > BrakeArmRaw`, and DV R2D on
`dv_r2d_req && brake_raw > BrakeDvHardRaw` (`control.cpp`). During bring-up the brake
line may be unpressurized and/or the start button unwired, leaving the FSM stuck in
`WaitStartBrake`. Make the app **assume** those inputs so the **real** R2D/RTDS sequence
runs — nothing to inject, no CAN traffic, no globals. **Both are config values in
`ecu_config.hpp`, NOT build flags** — a false/zero toggle folds away at compile time
(`if constexpr` / `!= 0`), so a flight `dev` build carries no stub code:

> **The stubs are not `-D` flags, but two real CMake options do exist** in
> `firmware/CMakeLists.txt`, both `OFF` by default and both **never a flight default**:
> `-DECU_HIL_CLEAR_ERROR_LATCH=ON` (wipes the RTC fault latch at boot — HIL only) and
> `-DECU_DEBUG_INV_BRIDGE=ON` (mirrors FDCAN1 inverter frames onto FDCAN2 for a pit
> sniffer, emitting extra IDs `0x560`/`0x562`/`0x57F`).


- **`config::StubBrakeRaw`** — a nonzero value is injected as `brake_raw` instead of
  reading the ADC. Set it ABOVE the threshold you need: `BrakeArmRaw` (750) for manual
  R2D, or **`BrakeDvHardRaw` (2500) for DV R2D** (e.g. 2700). `0` = read the real ADC
  (flight).
  > There is no longer an upper bound to respect. The EV.2.3 brake+throttle cut that
  > used to trip above `BrakePressedRaw` was deleted along with the rule in FS-Rules
  > 2024 (#177), so brake pressure no longer gates torque in either mode.
- **`config::StubStart`** — `start_button` is taken as pressed (PB5 isn't read).
  ⚠ **Do NOT set this for a DV (uDV-driven) R2D test** — it takes the manual branch first
  (`control.cpp`), preempting the `dv_r2d_req` path. `false` = read PB5 (flight).

**Build the bring-up image** (never a flight build) — set the toggles in `ecu_config.hpp`
first, then the ordinary firmware build. The **stubs** are source constants, not `-D` flags:
```bash
#   ecu_config.hpp: StubBrakeRaw = 2700;  StubStart = true;  // MANUAL R2D only — leave
#                                                            // StubStart false for DV/uDV
cmake -S firmware -B build-fw-brake \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake
cmake --build build-fw-brake
```

Flash it. Once precharge completes the FSM walks straight through:
`WaitStartBrake → R2dDelay` → **RTDS sounds for `R2dSoundMs` (2 s)** → `WaitInvStandby`.
Watch `0x700` `fsm_state` and the `rtds_active` bit (PB4). (Manual test: `StubBrakeRaw`
alone, press the real start button; `StubStart` alone, press the real brake.)

**⚠ Never flash an image with `StubBrakeRaw != 0` / `StubStart = true` to drive.**

`BrakeArmRaw` was calibrated on the car (2026-06-27, `750`). **`BrakePressedRaw` (3000,
brake full travel) and `BrakeDvHardRaw` (2500, the DV R2D gate) are still `COMMISSION`** —
recalibrate both once the line is purged, reading `0x701 brake_raw` via pit-diag.

> ⚠ Read `brake_raw` on **`0x701`**, not `0x705`: `PitDiag_brake`'s `brake_pressure` is
> currently **hardcoded to 0** in `pit_diag.cpp` (it depends on the brake calibration that
> does not exist yet), so `0x705` is a placeholder frame.

---

## 3. R2D + capped torque on stands, no AMS (`StubNoAms` / `TorqueCap`)

First powered freewheel: **no AMS on the bus**, inverter on **bench PSUs**, car **on
stands**. Two more `ecu_config.hpp` values on top of §2 (again, config, **not** build flags):

- **`config::StubNoAms`** — assume precharge-complete + AMS-healthy, so the FSM leaves
  `Precharge` without `0x020`. It still gates on the inverter's `0x466` DC-bus report
  first, so it won't arm into a dead bus. Consumed as `if constexpr (config::StubNoAms)`,
  so `false` discards it entirely. **This disables the AMS safety gate.**
  (A companion `config::StubNoInverter` fakes the inverter present/vconfig/Ready for a
  bench with no inverter either — likewise `if constexpr`, false on `dev`.)
- **`config::TorqueCap`** — clamps the commanded torque to N % right after the control
  step, so `0x362` to the inverter is capped (`100` = no cap).

> **⚠ SAFETY — read before flashing.** This image has **no AMS protection**: no
> over/under-voltage, over-current, or cell-temperature cutoff. The **PSU current limits**,
> the **torque cap**, and the **car on stands** are your entire safety envelope. **Never**
> flash it to a car on the ground or with a real HV pack. BL-recovery-check first
> (`0x002` / `0xB007AD12`); never power-cut mid-write.

**Build** (20 % cap, freewheeling) — set the toggles in `ecu_config.hpp`, then the ordinary
firmware build (no `-D` stub flags):
```bash
#   ecu_config.hpp: StubBrakeRaw = 2700;  StubNoAms = true;  TorqueCap = 20;
#                   StubStart = true;   // MANUAL only — leave false for a DV/uDV torque test
cmake -S firmware -B build-fw-bringup \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake
cmake --build build-fw-bringup
arm-none-eabi-objcopy -O binary build-fw-bringup/ECU08.elf build-fw-bringup/ECU08.bin
```

**Sequence**
1. **Calibrate APPS first** (§1) — so 20 % is 20 % of a *correct* pedal map.
2. Power the inverter from the current-limited PSUs, then the ECU.
3. The FSM walks: `WaitInvVdcConfig` (waits `0x466`) → `Precharge` → *(no-AMS)* →
   `WaitStartBrake` → *(brake + start)* → `R2dDelay` (RTDS 2 s) → `WaitInvStandby` →
   *(inverter Ready, `0x461` state 4)* → **`Active`**. Watch `0x700 fsm_state`.
4. **Ease the throttle in.** `0x362` torque tracks the APPS, clamped to 20 %; the wheels
   freewheel up. Watch `0x702` rpm and `0x700`.

**Bonus — settles the open E2E question.** If the inverter reaches Ready and the wheels
spin from the plain `0x362` frames, the NX/EMC inverter **doesn't need E2E** (close E-004).
If it never leaves `WaitInvStandby`, or ignores torque, E2E (or a config word) is required.

---

## 4. DV (driverless) R2D + torque without TS (`StubNoAms` + `StubNoInverter`)

Exercise the **autonomous** drive path — the uDV requests R2D and streams torque — with
**no tractive system energised**, a **real uDV on the FDCAN2 (ACU) bus**. The DV trigger is
latched once at `WaitStartBrake`: `dv_r2d_req && brake_raw > BrakeDvHardRaw` — no start
button, the EBS holds the hard braking, and the ECU verifies it on its own brake sensor
(see [uDV integration #17](https://github.com/isc-fs/IFS08-CE-ECU/issues/17)).

Two `ecu_config.hpp` toggles open the ladder to `Active` without HV; **`StubStart` must stay
`false`** — manual wins the manual/DV precedence tie, so a stubbed start button would preempt
the DV trigger:

- **`config::StubNoAms`** — clears `Precharge` (as §3). **Disables the AMS safety gate.**
- **`config::StubNoInverter`** — assumes the inverter vconfig + `Ready` state, clearing both
  inverter gates (`WaitInvVdcConfig` + `WaitInvStandby`) so the FSM reaches `Active` with no
  inverter on FDCAN1.

**Brake** — the DV R2D gate needs `brake_raw > BrakeDvHardRaw` (2500):
- If the **real EBS presses** the brake above 2500, leave `StubBrakeRaw = 0` (real ADC).
- Otherwise set `StubBrakeRaw` above 2500 — e.g. `2700`.

> **⚠ SAFETY.** No AMS, no inverter handshake — the same envelope as §3 (PSU limits + torque
> cap + car on stands). Keep `TorqueCap` low for on-stands work; **`100` for flight only**.
> BL-recovery-check first (`0x002` / `0xB007AD12`); never power-cut mid-write.

**Build** — set the toggles in `ecu_config.hpp`, then the ordinary firmware build
(no `-D` stub flags):
```bash
#   ecu_config.hpp: StubNoAms = true;  StubNoInverter = true;  StubStart = false;
#                   StubBrakeRaw = 2700;   // only if the EBS isn't pressing the brake
cmake -S firmware -B build-fw-dv \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake
cmake --build build-fw-dv
```

**Sequence**
1. Flash it (`StubStart` **false**). Power the ECU; the real uDV joins FDCAN2.
2. FSM walks `Precharge → WaitStartBrake` on the stubs. The ECU streams the uDV feed the
   whole time: `0x506` motor rpm @10 ms, `0x504` TS-active / `0x505` brake-over-limit /
   `0x511` R2D-confirm @100 ms.
3. **uDV sends `0x510`** (R2D request) with the brake over 2500 → the ECU latches DV,
   RTDS sounds (2 s), walks to `Active`. Confirm on pit-diag: `0x700` `dv_mode` bit set,
   and `0x707` shows `dv_r2d_req` / `brake_over_limit` / `r2d_confirm`.
4. **uDV streams `0x507`** torque % → conditioned to negative mechanical torque on `0x362`;
   a stale command (> `UdvCmdStaleMs` 100 ms) commands **0**, never an APPS fallback.
