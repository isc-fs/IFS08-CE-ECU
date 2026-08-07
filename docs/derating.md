# Torque limiting: the four caps

Every limiter in this ECU is a **`min()` ceiling** on commanded torque. This document
explains how they compose and which one to suspect when the car is slow. The thresholds
themselves live in [`Core/Inc/app/ecu_config.hpp`](../Core/Inc/app/ecu_config.hpp) and are
**not repeated here** — this file goes stale the moment it copies a number.

The reasoning behind each individual cap is in its own header, at length. Start here to
know *which* header to open.

## The chain

```
pedals -> deadband -> T.11.8.9 cut -> DV source override
       -> cell voltage cap        cell_derate.hpp
       -> motor thermal cap       motor_thermal.hpp
       -> pack thermal cap        pack_thermal.hpp
       -> EV 2.2.1 power envelope power_limit.hpp
       -> TorqueCap (bench stub, control_task.cpp)
       -> Nm, NEGATED, onto 0x362 inverter.cpp
```

**Order is immaterial** among the four caps: they are all ceilings, so the lowest wins
whatever sequence they run in. That was *not* true until #177 — the cell derate used to
multiply demand, and a gain has to come first or it scales the caps themselves. Do not
reintroduce a multiplier.

## Why caps and not gains

A gain rescales the whole pedal: at a 68 % factor a 30 % request becomes 20 %, even though
30 % was never the problem. A cap leaves everything below the ceiling untouched, so the
driver keeps full resolution over the range still allowed. This distinction cost a rewrite
(#177) — it is the single most important idea in this subsystem.

## Which cap is which

| cap | driven by | annunciated on | header |
|---|---|---|---|
| Cell voltage | AMS `0x12C` min cell, **IR-compensated to OCV** using `0x135` current | `0x709` `capped` | `cell_derate.hpp` |
| Motor thermal | inverter `0x464` motor temps (EMRAX 228) | `0x706` `thermal_capped` | `motor_thermal.hpp` |
| Pack thermal | AMS `0x136`/`0x137` per-module maxima | `0x70A` `pack_capped` | `pack_thermal.hpp` |
| Power envelope | `0x463` rpm, feed-forward | `0x700` `power_capped` | `power_limit.hpp` |
| `TorqueCap` | nothing — a compile-time bench stub | `0x704` `stub_torque_cap` (**ungated**) | — |

**Check `0x704` first.** `TorqueCap` is applied *after* the pure core, so it trips none of
the `capped` flags — a bench image looks exactly like a derate that no derate explains.

## Three ideas that recur in all four

**1. A cap must never RELAX when a sensor is lost.** Losing sight of the pack must not hand
the car more torque than it had while the pack was visible. Both thermal caps ratchet down
only. This was a real bug (a hot pack jumped 78 Nm → 139 Nm when its frames went quiet).

**2. Per-signal freshness, never bus-level.** `last_ams_tick` and `last_inv_tick` are
stamped by *every* frame on their bus, so they cannot tell you that one frame died — and a
frozen reading always looks healthy. Five separate bugs of this exact shape have been fixed
(`0x135`, `0x464`, `0x136`/`0x137`, the FOC frames, `0x463`). **If you add a consumer of a
CAN signal, give it its own timestamp.**

**3. A floor must command real torque.** The percentage scale is *not* linear to 240 Nm —
`InvTorqueMap*` is re-based so `DeadbandLowPct` maps to exactly 0 Nm. A floor written as
"5 %" commanded *nothing*, while reading as a limp-home. Assert on the **Nm**, not the
percent; `ecu_config.hpp` has the `static_assert`.

## Constants that were never measured

Several thresholds are placeholders. `ecu_config.hpp` tags each one and gives the procedure;
the ones whose absence changes behaviour most:

- **`CellIrMilliOhm`** — one acceleration run, plot `est_ocv_mV` against `raw_mV` on `0x709`. Flat is correct.
- **`DrivetrainEffPct`** — the *entire* EV 2.2.1 compliance margin. `0x70D` puts commanded shaft power next to the inverter's own AC measurement and the DC bus, so one capture at steady full throttle gives the real number.
- **`PackTempLimitDegC`, `CellVDerateFloorMv`** — must be checked against what the **AMS** actually trips on. See below.

## ⚠️ Cross-check the AMS, always

The AMS opens the AIRs on its own thresholds. **An ECU cap set below the AMS trip point can
never engage** — the AIRs open first and the derate is decorative. This is a live example
worth understanding before you touch any threshold:

- AMS `CellUnderVoltageMv` trips on **raw loaded** min cell
- The ECU cell cap ramps on **IR-compensated OCV**, which under load is always *higher*

So the ECU must sit **above** the AMS trip with enough margin to cover the IR drop, or the
protection inverts. Both values are `COMMISSION` on both sides — agree one number across the
two repos rather than tuning either in isolation.

`IFS08-CE-AMS/Core/Inc/app/ams_config.hpp` is a sibling checkout. Read it.
