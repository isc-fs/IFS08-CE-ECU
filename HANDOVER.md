# IFS08-CE-ECU — Session Handover

> Point-in-time handover written 2026-07-24 to resume work on a different computer.
> Delete this file once you've picked the thread back up. Everything here was verified
> against live `git`/`gh` state at write time, but **issue/PR numbers and `dev`'s tip
> move** — re-check with the commands in [§1](#1-first-five-minutes-on-the-new-machine)
> before trusting a number below.

---

## 0. One-paragraph status

The ECU firmware is healthy on `dev` (tip `225f460`, v1.0.0 released). This session
shipped a pile of merged work (GPS, inverter observability, SPI1 removal, two CI
guards, docs). **The one unresolved engineering problem is [#148]: after a tractive-
system (TS) deactivation the inverter will not come back to torque without a power
cycle.** Everything needed to diagnose it is now on the bus — the next step is a
**pit-diag capture on stands**, not more code. Two doc/CI PRs are open and green
([#156], [#158]); merge them and you're at a clean baseline.

---

## 1. First five minutes on the new machine

```bash
# Repos (all under ~/Documents/Github on the old machine)
git clone https://github.com/isc-fs/IFS08-CE-ECU        # THIS repo (origin = isc-fs)
git clone https://github.com/isc-fs/IFS08_HIL           # HIL rig (separate workstream)
git clone https://github.com/isc-fs/IFS08-TE            # ground station / telemetry receiver
git clone https://github.com/isc-fs/can-flasher         # MingoCAN pit tool (Rust)
# IFS08_PRIVATE/GPS_TEST  -> the bench-proven GPS project the GPS driver was ported from
# IFS08-CE/"VCU pre-jarama" -> last year's IFS07 VCU (reference for the inverter ladder)

cd IFS08-CE-ECU
git checkout dev && git pull

# Toolchain (macOS / Homebrew on the old box):
#   arm-none-eabi-gcc (14.2.Rel1 in CI), cmake, ninja/make, python3, gh
brew install --cask gcc-arm-embedded    # or the carlosperate action's 14.2.Rel1
brew install cmake gh
gh auth login                            # needed for the issue/PR workflow below
```

Re-verify the live state (numbers below WILL have drifted):

```bash
git log --oneline -1                                     # dev tip
gh pr  list --state open  --json number,title,headRefName,mergeStateStatus
gh issue list --state open --json number,title
```

`origin` must be `https://github.com/isc-fs/IFS08-CE-ECU`. Note there is **also** a
`mrandy` remote (`MrAndy5/IFS08-CE-ECU_FORK`) — that's the fork the nRF24 bit-bang
driver was salvaged from; don't push there.

---

## 2. THE ACTIVE TASK — #148, inverter won't return to torque after TS-off

This is where we stopped. **Read [#148] in full** — it has the whole history. Summary:

### What happens
On the car (385 V pack, on stands), after the tractive system is deactivated
mid-drive, doing the R2D re-arm does **not** bring the inverter back to torque. Only a
power cycle recovers it. The driver re-doing R2D after every TS-off is **required and
correct** (FSAE); what's missing is the inverter climbing back to TorqueEnable without
a power cycle — a capability last year's IFS07 VCU had.

### What we tried and REVERTED (do not repeat)
- **[#144]** made `WaitInvStandby` reactive: it commanded `Off(0x01)` instead of
  `Ready(0x04)` when the inverter reported `OFF(0)`/`SHUTDOWN(13)`, on the theory
  (from the IFS07 switch) that an off inverter needs an "on" word before it accepts
  Ready. **It did not work on the car**, and the **W90 manual §9.1 state diagram
  contradicts the premise**: `OFF --(App_State_Req = READY)--> READY` is a *single
  direct transition*. So commanding `Off` to an already-off inverter just holds it
  there and Ready is never sent. **[#155] reverted it.** `dev` now commands
  unconditional `Ready` in `WaitInvStandby` (matches the manual).
  - `Core/Src/app/control.cpp`, the output-side `case CtrlState::WaitInvStandby:` —
    `mode = InvMode::Ready;` unconditionally, with a big comment block explaining why
    it must NOT be made reactive again.
  - `InvOffState(0)`/`InvShutdownState(13)` in `ecu_config.hpp` are tagged
    **UNVERIFIED** — `3/4/10/11` are bench-proven, `0/13` are IFS07-derived guesses.

### The leading hypothesis (unproven)
The FSM may never reach `WaitInvStandby`. In `control.cpp`:
```
if (in.ams_error && state_ != AmsError) enter_(AmsError);   // from ANY state
case AmsError: mode = InvMode::Off;                          // only ever 0x01
if (state_ != AmsError) { ...inverter fault recovery... }    // <- SUPPRESSED in AmsError
case AmsError: if (!in.ams_error) -> WaitInvVdcConfig;       // only exit
```
with `ams_error = (ams_fsm_state == AmsFsmError(5)) && ams_fresh` (`control_task.cpp`).
**If the AMS latches Error on TS-off**, the ECU parks in `AmsError`, commands only
`Off`, and inverter fault recovery is *deliberately inhibited*. A power cycle "fixes"
it by clearing the **AMS** latch, not the inverter. That would explain why #144 (which
only touched `WaitInvStandby`) changed nothing. **This is not confirmed** — it needs
the capture below.

### THE NEXT STEP — a pit-diag capture on stands (no code needed first)

`dev` already exposes everything. **Flash current `dev`** (confirm via `0x703` fwinfo
git hash), then:

1. **Arm the pit-diag stream** — send one CAN frame on the ACU bus (FDCAN2):
   ```
   ID 0x7E0   DLC 4   data: DE AD BE EF        (magic 0xDEADBEEF, big-endian)
   ```
   ECU acks on `0x7E1`; `0x700`/`0x702` then stream at 100 ms. (Send `00 00 00 00` to
   disable.)
2. **Reproduce the stall** (TS off → R2D re-arm → stuck).
3. **Read these while stuck** (MingoCAN + current `ecu.dbc` names every one — no
   hand-decoding):
   | Signal | Frame | Note |
   |---|---|---|
   | `fsm_state` | `0x700` | the decisive one |
   | `inv_state` | `0x700` | #150 named 0/6/10/11/13, not just 3/4 |
   | `ok_precharge` | `0x700` (bit) | did HV/precharge return? |
   | `inv_mode_cmd` | `0x702` | **what the ECU is COMMANDING** (App_State_Req) — added by #150 |
   | `inv_error` + `dem_present` | `0x702` | W90 DEM name + active-vs-latched |

4. **Decision table** — one capture localises it:
   | `fsm_state` | `inv_state` | `inv_mode_cmd` | Conclusion |
   |---|---|---|---|
   | **AmsError** | any | Off | AMS latched Error → recovery inhibited **by design**. Safety-policy decision, not a bug |
   | **WaitInvStandby** | Off/Shutdown | **Ready** | ECU commanding correctly, inverter refusing → inverter/wiring |
   | WaitInvStandby | HardFault(11) | HardFaultReset | fault won't clear → inverter-side |
   | **Precharge** | any | Off | `ok_precharge` never returned → AMS-side, inverter irrelevant |
   | Active | TorqueEnable | TorqueEnable | recovered → torque path issue instead |

   Before #150 you couldn't see `inv_mode_cmd`, so rows 1–3 were indistinguishable.
   Now they aren't.

**A MingoCAN log / trace export of a few seconds of `0x700`+`0x702` while stuck is the
single most useful artifact.** Hand it to the next session and it can read the branch
directly.

### The inverter (context you'll need)
EPowerLabs **W90 (EMC150)** = NX0001-STS04 "A16" config. FDCAN1 @ 500 kbit/s classic.
Manual `EPowerLabs W90 UserManual.pdf` is in the repo root (merged via #126). Bench-
proven command words: `0x01` off, `0x04` ready, `0x06` torque, `0x0D` hard-fault reset,
`0x13` soft-fault reset (these are `InvMode` in `control.hpp`). Torque is **NEGATED**
(mechanical mounting — forward = negative `Torque_Nm_Req`), don't "fix" it. The
manual's `App_State_Req` *enum numbering* (1/2/3/4/5) does NOT match our A16 config, and
its annex uses different CAN IDs — so **trust the bench for numbers, the §9.1 diagram
for topology**.

---

## 3. Open PRs (all green at handover — merge these)

Merge order matters only because `dev` is protected (each merge pushes the others
`BEHIND`; run `gh pr update-branch <n>`, wait for CI, then merge).

| PR | Branch | What | Notes |
|----|--------|------|-------|
| [#158] | `feat/137-restore-stub-brake` | Restores `stub_brake` on `0x704` (`Closes #137`) | Narrows `reset_cause` to 3 bits to free a bit; verified disjoint. Closes tracker #157 on merge |
| [#156] | `ci/autoclose-negation-guard` | Stops the autoclose workflow closing an issue on a *negated* keyword ("does not fix #N") | See the war story in [§5](#5-gotchas-that-bit-us-this-session-read-before-repeating) |

There is also an **unmerged prepared revert already merged** — ignore, #155 is in.

New branch not ours: `feat/2` (tracker **#159**, "Publish inverter telemetry on CAN3")
— someone else's WIP, 1 commit ahead of `dev`. Leave it.

---

## 4. Open issues (grouped by what unblocks them)

**Needs the car / stands:**
- **[#148]** — the active task above. TS-off inverter recovery. **Top priority.**

**Design decision for you (codeable once you decide):**
- **[#132]** — priority TX lane so safety cyclics (`0x100`/`0x504`/`0x505`/`0x511`)
  can't be starved by the pit-diag/telemetry flood. Real design choice (priority queue
  vs separate FIFO) — decide before building. *This is the main substantial firmware
  task that can be done off-car.*
- **[#137]** — being closed by #158; will auto-close on merge.

**Blocked on another team (uDV):**
- **[#108]** — ECU consumes uDV IMU broadcast `0x512` (FDCAN2, DLC 8, 50 Hz). Waiting
  on uDV confirming they want the ECU to decode it. #112 was deduped into this.
- **[#127]** — `0x504 VCU_ts_active` semantics + liveness; uDV moving TS off the TSMS
  pin onto the frame.
- **[#142]** — AS-Emergency acoustic on the RTDS buzzer; consume new uDV `0x513`
  AS-status frame, sound 10 s / 1–5 Hz on `RTDS_Pin`.

**Auto-trackers (close themselves on merge):**
- **#157** (feat/137-restore-stub-brake), **#159** (feat/2).

---

## 5. Cross-repo threads (don't drop these)

- **isc-fs/IFS08-TE#1** — the ground station receives GPS now (`0x508`/`0x509`, and in
  the radio snapshot bytes `[82..95]`) but **won't display it** until
  `ISC_REAL_TIME_25/ISC_RTT_serial.py` `_decode_snapshot()` is extended (branch
  `feat/receptor_08`). Full byte map is in the issue.
- **isc-fs/can-flasher#505** — MingoCAN "flight-vs-bench badge" surfacing all four ECU
  stubs uniformly. Updated this session to note `stub_brake` (0x704 b43) now exists.
  Basic decode is automatic once MingoCAN loads the regenerated `ecu.dbc`.

---

## 6. Gotchas that bit us this session (READ before repeating)

1. **CubeMX regen zeroes the FDCAN MessageRAM offsets.** There is NO GUI field for
   `MessageRAMOffset`, so *every* `.ioc` regen resets FDCAN2 (387) and FDCAN3 (582) to
   0 → overlapping windows → **CAN TX dies** (this is defect #48). It has happened 3+
   times. **After any regen**, restore `387`/`582` in `Core/Src/fdcan.c`. CI now guards
   this (`scripts/check_fdcan_ram.py`, runs in build-tests.yml + release.yml) — it
   computes the real footprint and fails on overlap. Snapshot the fragile values before
   a regen and diff after: FDCAN offsets, `AutoRetransmission=ENABLE ×3`, USART10
   `BaudRate=9600` (GPS), `main.c __enable_irq()` (#48 boot handoff), and the `GpsTask`
   registration in `freertos.c` USER CODE.
2. **Bench edits keep getting left on `dev`'s working tree.** `StubBrakeRaw` (→2700)
   and `TorqueCap` (→35/50/65) get set for on-stands testing and left uncommitted.
   Before branching, `git checkout -- Core/Inc/app/ecu_config.hpp` to discard them.
   `dev` must always have `StubBrakeRaw=0`, `TorqueCap=100`, all `StubX=false`.
3. **The autoclose workflow is negation-blind** (being fixed in #156). Writing "does
   not fix #N" in a PR body *closes* #N on merge. It closed #148 this way; had to
   reopen by hand. Until #156 merges, don't reference an issue number after a negation
   in a PR body/title.
4. **`gh ... --body "..."` with backticks/apostrophes gets mangled by the shell** —
   the shell runs backticked text as commands. **Always use `gh ... --body-file`** for
   any comment/PR body containing `` ` `` or `'`.
5. **Don't build unless asked.** Standing rule: Raúl owns local builds; CI is the gate.
   When authorized, build in `build-sil/` / `build-fw/` (both gitignored) so VS Code /
   MingoCAN / CMake settings are never touched.
6. **No Claude attribution** on commits or PRs (no `Co-Authored-By`, no "Generated
   with").
7. **`dev` and `main` are protected** — PR + green CI required; merges are Raúl's. Every
   merge pushes other open PRs `BEHIND`; update-branch + re-wait-for-CI each time.

---

## 7. Build / test / DBC quick reference

```bash
# Host SIL (pure control core — no HW, no RTOS). This is the fast regression gate.
cmake -S . -B build-sil -DBUILD_SIL_TESTS=ON -DBUILD_UNIT_TESTS=OFF
cmake --build build-sil -j
./build-sil/tests/sil/ecu08_sil --test-all          # 273 checks at handover
./build-sil/tests/sil/ecu08_sil --test-inverter-ts-off   # the #148-relevant suite
./build-sil/tests/sil/ecu08_sil --dump-radio         # emit a known-value radio snapshot (hex)

# ARM firmware cross-compile (needs arm-none-eabi + the toolchain file)
cmake -S firmware -B build-fw -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/gcc-arm-none-eabi.cmake"
cmake --build build-fw -j                            # ~92.5 KB FLASH at handover

# Regenerate the DBC from the code-first DSL (the dbcinator bot also does this per-PR)
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump > docs/dbc/ecu.dbc

# FDCAN RAM guard (the #48 check)
python3 scripts/check_fdcan_ram.py Core/Src/fdcan.c
```

Flash is over **CAN bootloader** (app @ `0x08020000`, node id `0x01`). See
`docs/commissioning.md` and the memory recipes for the exact PCAN/flasher invocation.

---

## 8. Architecture in ten lines (see `CLAUDE.md` for the full version)

STM32H733ZG, lean FreeRTOS C++17 app linking over the stm32-can-bootloader. Pure
host-testable control core (`ecu::Controller::step()`) surrounded by a thin RTOS/HAL
layer. Tasks: `ControlTask` (10 ms realtime, the only safety actor, sole IWDG kicker),
`CanRxTask` (sole `VehicleService` writer), `CanTxTask` (sole FDCAN TX), `DiagTask`
(1 Hz `0x704` health), `TelemetryTask` (200 ms FDCAN3 dash + nRF24 radio), `GpsTask`
(20 ms NMEA drain, `0x508`/`0x509`). Buses: **FDCAN1 = inverter**, **FDCAN2 = AMS +
pit-tool + uDV**. CAN map is a **code-first X-macro DSL** (`Core/Inc/can/messages/*.def`
+ `all_messages.inc`) → structs/codecs/descriptors/DBC, one source of truth. nRF24 is
**bit-bang** on PA5/6/7 (the SPI1 peripheral reads MISO stuck-high on this board; SPI1
was removed this session). GPS = MTK3339 on **USART10 (PG11/PG12) @ 9600**.

---

## 9. What this session shipped (for context, all merged to `dev`)

- **#147** GPS position + speed (MTK3339/USART10 → `0x508`/`0x509` + radio + uDV)
- **#150** inverter observability: `0x702 inv_mode_cmd` (commanded mode) + `inv_state`
  VAL names — **the tooling that makes #148's capture decisive**
- **#152** removed inert SPI1 (nRF24 is bit-bang)
- **#153** CI guard: FDCAN MessageRAM overlap (#48 defense)
- **#149** CI: auto-close branch trackers on merge
- **#155** reverted #144 (the failed TS-off climb)
- **#144** the failed TS-off attempt (kept in history, reverted)
- **#126** W90 manual in-tree · **#133** vestigial constant · **#145** docs accuracy
  pass · **#154** GPS in the radio snapshot map
- Issue hygiene: 21 → ~8 open; dedup, superseded-branch closes.

---

*Written by the assistant at session end, 2026-07-24. When you resume: re-run the §1
commands, then go to §2. The capture is the whole game for #148.*

[#108]: https://github.com/isc-fs/IFS08-CE-ECU/issues/108
[#127]: https://github.com/isc-fs/IFS08-CE-ECU/issues/127
[#132]: https://github.com/isc-fs/IFS08-CE-ECU/issues/132
[#137]: https://github.com/isc-fs/IFS08-CE-ECU/issues/137
[#142]: https://github.com/isc-fs/IFS08-CE-ECU/issues/142
[#144]: https://github.com/isc-fs/IFS08-CE-ECU/pull/144
[#148]: https://github.com/isc-fs/IFS08-CE-ECU/issues/148
[#155]: https://github.com/isc-fs/IFS08-CE-ECU/pull/155
[#156]: https://github.com/isc-fs/IFS08-CE-ECU/pull/156
[#158]: https://github.com/isc-fs/IFS08-CE-ECU/pull/158
