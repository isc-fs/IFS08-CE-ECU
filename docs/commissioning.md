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

## 2. R2D / RTDS test with an unpurged brake (`ECU_HIL_STUB_BRAKE`)

R2D arms on `start_button && brake_raw > BrakeArmRaw` (`control.cpp`). With an
unpurged brake line the pressure sensor can't reach `BrakeArmRaw`, so the FSM is
stuck in `WaitStartBrake`. The **`ECU_HIL_STUB_BRAKE`** bring-up flag injects the
brake over CAN so R2D/RTDS can be exercised anyway. It is **independent of the
start-button stub** — the car's real start button still works.

**Build the bring-up image** (OFF by default — never a flight build; `nm` confirms
`g_hil_force_brake` is absent from the default image):
```bash
cmake -S firmware -B build-fw-brake \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake \
  -DECU_HIL_STUB_BRAKE=ON
cmake --build build-fw-brake
```

**Inject + arm:** with the stub image flashed, `brake_raw = (0x7E0 byte5) << 4`
(the ADC is ignored). Enable pit-diag and set the brake in one frame:
`0x7E0 = DE AD BE EF <byte4> <byte5>`.

| `byte5` | `brake_raw` | effect |
|---|---|---|
| `0x00` | 0 | released |
| `0x39` (57) | 912 | minimum to arm R2D (`> BrakeArmRaw` 900) |
| `0xBC` (188) | 3008 | registers EV.2.3 "pressed" (`> BrakePressedRaw` 3000) |
| `0xC0` (192) | 3072 | firmly pressed (arms R2D + EV.2.3) |

With the brake injected above `BrakeArmRaw` and the start button pressed, the FSM
runs `WaitStartBrake → R2dDelay` → **RTDS sounds for `R2dSoundMs` (2 s)** →
`WaitInvStandby`. Watch `0x700` `fsm_state` and the `rtds_active` bit (PB4).

**⚠ Never flash a `-DECU_HIL_STUB_BRAKE=ON` image to drive.**

`BrakeArmRaw` / `BrakePressedRaw` are also `COMMISSION` placeholders — recalibrate
the real brake sensor (same `0x701 brake_raw` / `0x705` read approach as APPS) once
the line is purged.
