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
