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
  The ECU acks `0x7E1 = 1` and streams `0x700`–`0x705`. Disable with `0x7E0 = 0`.

---

## 1. APPS calibration (`Apps1/2AdcMin/Max`)

The pedal travel maps raw ADC → torque %. The endpoints are board- and
pedal-specific.

**Read:** with pit-diag on, watch **`0x701` PitDiag_pedals** — `apps1_raw`,
`apps2_raw` (12-bit ADC). `apps1_pct`/`apps2_pct` show the *current* (placeholder)
mapping.

**Measure:**
1. **Rest (0 %):** pedal fully released → record steady `apps1_raw → Apps1AdcMin`,
   `apps2_raw → Apps2AdcMin`.
2. **Full (100 %):** pedal to the mechanical stop → record `apps1_raw → Apps1AdcMax`,
   `apps2_raw → Apps2AdcMax`.
3. **Sanity:** both channels move together (the dual-APPS / T.11.8.9 redundancy).
   Keep the convention `*AdcMin` = rest, `*AdcMax` = full — the firmware handles the
   span direction.
4. **Margin:** set `*AdcMin` a hair *above* the true rest reading so a released
   pedal lands at exactly 0 %, not a value that clamps; leave headroom below `*AdcMax`.

**Apply + verify:**
1. Set the four constants in `Core/Inc/app/ecu_config.hpp`.
2. Rebuild (`cmake --build build-fw`) and flash over CAN
   (`can-flasher … --address 0x08020000 --verify-after --jump`).
   **Confirm BL recovery first** (`0x002`/`0xB007AD12` round-trip); never power-cut
   mid-write.
3. Re-enable pit-diag, watch `0x701`: `apps1_pct`/`apps2_pct` read **0 % at rest,
   100 % at full**, tracking within 10 % across the sweep (no T.11.8.9 false-trip).

---

## 2. R2D / RTDS test without brake pressure / start wiring (`StubBrakeRaw` / `StubStart`)

R2D arms on `start_button && brake_raw > BrakeArmRaw`, and DV R2D on
`dv_r2d_req && brake_raw > BrakeDvHardRaw` (`control.cpp`). During bring-up the brake
line may be unpressurized and/or the start button unwired, leaving the FSM stuck in
`WaitStartBrake`. Make the app **assume** those inputs so the **real** R2D/RTDS sequence
runs — nothing to inject, no CAN traffic, no globals. **Both are config values in
`ecu_config.hpp`, NOT build flags** — a false/zero toggle folds away at compile time
(`if constexpr` / `!= 0`), so a flight `dev` build carries no stub code:

- **`config::StubBrakeRaw`** — a nonzero value is injected as `brake_raw` instead of
  reading the ADC. Set it ABOVE the threshold you need: `BrakeArmRaw` (900) for manual
  R2D, or **`BrakeDvHardRaw` (2500) for DV R2D** (e.g. 2700), and BELOW `BrakePressedRaw`
  (3000) to dodge the EV.2.3 cut. `0` = read the real ADC (flight).
- **`config::StubStart`** — `start_button` is taken as pressed (PB5 isn't read).
  ⚠ **Do NOT set this for a DV (uDV-driven) R2D test** — it takes the manual branch first
  (`control.cpp`), preempting the `dv_r2d_req` path. `false` = read PB5 (flight).

**Build the bring-up image** (never a flight build) — set the toggles in `ecu_config.hpp`
first (the `bench/car-stubs` branch already has `StubBrakeRaw = 2700`), then the ordinary
firmware build; there are no `-D` flags:
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

`BrakeArmRaw` / `BrakePressedRaw` are `COMMISSION` placeholders — recalibrate the real
brake once the line is purged (read `0x701 brake_raw` / `0x705` via pit-diag).

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
