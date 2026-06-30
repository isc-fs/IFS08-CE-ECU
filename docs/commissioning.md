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

## 2. R2D / RTDS test without brake pressure / start wiring (`ECU_STUB_BRAKE` / `ECU_STUB_START`)

R2D arms on `start_button && brake_raw > BrakeArmRaw` (`control.cpp`). During bring-up
the brake line may be unpressurized and/or the start button unwired, leaving the FSM
stuck in `WaitStartBrake`. Two independent **compile-time** flags make the app simply
**assume** those inputs, so the **real** R2D/RTDS sequence runs — nothing to inject,
no CAN traffic, no globals:

- **`ECU_STUB_BRAKE`** — `brake_raw` is taken as fully pressed (the ADC isn't read).
- **`ECU_STUB_START`** — `start_button` is taken as pressed (PB5 isn't read).

Each is its own flag, OFF by default. Both are separate from the bench HIL start stub.

**Build the bring-up image** (never a flight build):
```bash
cmake -S firmware -B build-fw-brake \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake \
  -DECU_STUB_BRAKE=ON -DECU_STUB_START=ON      # drop either flag you don't need
cmake --build build-fw-brake
```

Flash it. With both flags on, once precharge completes the FSM walks straight through:
`WaitStartBrake → R2dDelay` → **RTDS sounds for `R2dSoundMs` (2 s)** → `WaitInvStandby`.
Watch `0x700` `fsm_state` and the `rtds_active` bit (PB4). (With only `ECU_STUB_BRAKE`,
press the real start button; with only `ECU_STUB_START`, press the real brake.)

**⚠ Never flash a `-DECU_STUB_BRAKE=ON` / `-DECU_STUB_START=ON` image to drive.**

`BrakeArmRaw` / `BrakePressedRaw` are `COMMISSION` placeholders — recalibrate the real
brake once the line is purged (read `0x701 brake_raw` / `0x705` via pit-diag).

---

## 3. R2D + capped torque on stands, no AMS (`ECU_STUB_NO_AMS` / `ECU_BRINGUP_TORQUE_CAP_PCT`)

First powered freewheel: **no AMS on the bus**, inverter on **bench PSUs**, car **on
stands**. Two more compile-time flags on top of §2:

- **`ECU_STUB_NO_AMS`** — assume precharge-complete + AMS-healthy, so the FSM leaves
  `Precharge` without `0x020`. It still gates on the inverter's `0x466` DC-bus report
  first, so it won't arm into a dead bus. **This disables the AMS safety gate.**
- **`ECU_BRINGUP_TORQUE_CAP_PCT=N`** — clamps the commanded torque to N % right after the
  control step, so `0x362` to the inverter is capped (empty = no cap).

> **⚠ SAFETY — read before flashing.** This image has **no AMS protection**: no
> over/under-voltage, over-current, or cell-temperature cutoff. The **PSU current limits**,
> the **torque cap**, and the **car on stands** are your entire safety envelope. **Never**
> flash it to a car on the ground or with a real HV pack. BL-recovery-check first
> (`0x002` / `0xB007AD12`); never power-cut mid-write.

**Build** (20 % cap, freewheeling):
```bash
cmake -S firmware -B build-fw-bringup \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake \
  -DECU_STUB_BRAKE=ON -DECU_STUB_NO_AMS=ON -DECU_BRINGUP_TORQUE_CAP_PCT=20 \
  -DECU_STUB_START=ON          # drop this and press the real start button if it's wired
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
