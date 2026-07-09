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

- **`ECU_STUB_BRAKE`** — `brake_raw` is taken from the `config::StubBrakeRaw` constant
  instead of the ADC. **It defaults to `0` (released)**, which does *not* arm R2D — you
  **must** raise `StubBrakeRaw` in `ecu_config.hpp` above the gate you want to clear
  (`> BrakeArmRaw` to arm R2D; keep it `< BrakePressedRaw` to dodge the EV.2.3 cut).
- **`ECU_STUB_START`** — `start_button` is taken as pressed (PB5 isn't read).

Each is its own flag, OFF by default. Both are separate from the bench HIL start stub.

**Build the bring-up image** (never a flight build) — set `StubBrakeRaw` first:
```bash
# in Core/Inc/app/ecu_config.hpp, e.g.:  StubBrakeRaw = 1000;  // > BrakeArmRaw, < BrakePressedRaw
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

## 3. R2D + capped torque on stands, no AMS (`ECU_STUB_NO_AMS` + `config::TorqueCap`)

First powered freewheel: **no AMS on the bus**, inverter on **bench PSUs**, car **on
stands**. One more compile-time flag on top of §2, plus a source constant:

- **`ECU_STUB_NO_AMS`** — assume precharge-complete + AMS-healthy, so the FSM leaves
  `Precharge` without `0x020`. It still gates on the inverter's `0x466` DC-bus report
  first, so it won't arm into a dead bus. **This disables the AMS safety gate.**
- **`config::TorqueCap`** (in `ecu_config.hpp`) — clamps the commanded torque to N %
  right after the control step, so `0x362` to the inverter is capped. Unlike the old
  `ECU_BRINGUP_TORQUE_CAP_PCT` *build flag* (removed), this is an **always-applied**
  constant: `100` = no cap, and it **MUST be `100` for any flight/drive build**. Lower
  it (e.g. `20`) only for on-stands work.

> **⚠ SAFETY — read before flashing.** This image has **no AMS protection**: no
> over/under-voltage, over-current, or cell-temperature cutoff. The **PSU current limits**,
> the **torque cap**, and the **car on stands** are your entire safety envelope. **Never**
> flash it to a car on the ground or with a real HV pack. BL-recovery-check first
> (`0x002` / `0xB007AD12`); never power-cut mid-write.

**Build** (20 % cap, freewheeling) — set `TorqueCap = 20` and `StubBrakeRaw` (> `BrakeArmRaw`)
in `ecu_config.hpp` first:
```bash
# ecu_config.hpp:  TorqueCap = 20;  StubBrakeRaw = 1000;
cmake -S firmware -B build-fw-bringup \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake \
  -DECU_STUB_BRAKE=ON -DECU_STUB_NO_AMS=ON \
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

---

## 4. DV (driverless) R2D + torque without TS (`ECU_STUB_NO_AMS` + `ECU_STUB_NO_INVERTER`)

Exercise the **autonomous** drive path — the uDV requests R2D and streams torque — with
**no tractive system energised**, a **real uDV on the FDCAN2 (ACU) bus**. The DV trigger is
latched once at `WaitStartBrake`: `dv_r2d_req && brake_raw > BrakeDvHardRaw` — no start
button, the EBS holds the hard braking, and the ECU verifies it on its own brake sensor
(see [uDV integration #17](https://github.com/isc-fs/IFS08-CE-ECU/issues/17)).

Two stub flags open the ladder to `Active` without HV; **`ECU_STUB_START` must stay OFF** —
manual wins the manual/DV precedence tie, so a stubbed start button would preempt the DV
trigger:

- **`ECU_STUB_NO_AMS`** — clears `Precharge` (as §3). **Disables the AMS safety gate.**
- **`ECU_STUB_NO_INVERTER`** — assumes the inverter vconfig + `Ready` state, clearing both
  inverter gates (`WaitInvVdcConfig` + `WaitInvStandby`) so the FSM reaches `Active` with no
  inverter on FDCAN1.

**Brake** — the DV R2D gate needs `brake_raw > BrakeDvHardRaw` (2500):
- If the **real EBS presses** the brake above 2500, build **without** `ECU_STUB_BRAKE`.
- Otherwise add `-DECU_STUB_BRAKE=ON` and set `StubBrakeRaw > 2500` (and `< BrakePressedRaw`
  3000, to dodge EV.2.3) in `ecu_config.hpp` — e.g. `2700`.

> **⚠ SAFETY.** No AMS, no inverter handshake — the same envelope as §3 (PSU limits + torque
> cap + car on stands). Keep `TorqueCap` low for on-stands work; **`100` for flight only**.
> BL-recovery-check first (`0x002` / `0xB007AD12`); never power-cut mid-write.

**Build** (real EBS pressing the brake shown; add `ECU_STUB_BRAKE` + `StubBrakeRaw` if not):
```bash
cmake -S firmware -B build-fw-dv \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake \
  -DECU_STUB_NO_AMS=ON -DECU_STUB_NO_INVERTER=ON
  #  + -DECU_STUB_BRAKE=ON   (with StubBrakeRaw > 2500) if the EBS isn't pressing
cmake --build build-fw-dv
```

**Sequence**
1. Flash it (`ECU_STUB_START` **off**). Power the ECU; the real uDV joins FDCAN2.
2. FSM walks `Precharge → WaitStartBrake` on the stubs. The ECU streams the uDV feed the
   whole time: `0x506` motor rpm @10 ms, `0x504` TS-active / `0x505` brake-over-limit /
   `0x511` R2D-confirm @100 ms.
3. **uDV sends `0x510`** (R2D request) with the brake over 2500 → the ECU latches DV,
   RTDS sounds (2 s), walks to `Active`. Confirm on pit-diag: `0x700` `dv_mode` bit set,
   and `0x707` shows `dv_r2d_req` / `brake_over_limit` / `r2d_confirm`.
4. **uDV streams `0x507`** torque % → conditioned to negative mechanical torque on `0x362`;
   a stale command (> `UdvCmdStaleMs` 100 ms) commands **0**, never an APPS fallback.

The on-stands DV image can be built ad-hoc as above, or from the parked `bench/car-stubs`
overrides — whichever is current.
