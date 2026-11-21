# F427 isolated bring-up configs

These are **single-purpose, minimal** Klipper configs used during Phase-2 hardware
bring-up to exercise one subsystem at a time (SPI comms, motion, sensorless homing,
extruder driver) without the full machine. They were written as throwaway snapshots,
so treat them as **scratch tools, not the source of truth**.

> **Canonical hardware = `../boards/xbuddy.cfg`** (consumed via `../printer.cfg`).
> If a value here disagrees with `xbuddy.cfg` — currents, SGT, rotation_distance,
> position_endstop, chopper timing, the ADC fix — `xbuddy.cfg` is right. Do **not**
> copy values out of these test files into the real config. The normal bring-up
> runbook (`../../docs/klipper_f427_bringup.md`) uses `printer.cfg`, not these.

## Files

| File | Purpose | Notes |
|---|---|---|
| `tmc-spi-test.cfg` | Prove SPI3a + CS wiring — `DUMP_TMC` each driver. No motion. | low current, dummy `^PG2/3/4` endstops |
| `motion.cfg` | Basic per-axis motion via `FORCE_MOVE`/`G1`. | near-duplicate of `corexy.cfg` |
| `corexy.cfg` | CoreXY direction check (an 'X' move turns both motors). | + `[force_move]` |
| `etmc.cfg` | Extruder TMC driver — `DUMP_TMC STEPPER=extruder` + jog. | |
| `estall.cfg` | **Loadcell clog detector + HX717 channel interleave.** Arms `[estall_detect]`, verifies ch A (e-stall) + ch B (presence) stay live together, trips the FIR on a hand-induced force edge. | **placeholder HX717 pins** — fill from `xbuddy.cfg`; in-file bench procedure |
| `homing.cfg` | **Sensorless X/Y homing + live SGT tuning.** Has the **`diag1_pushpull` fix** and the verified `SGT X=1/Y=0`. | the one to trust for homing |
| `sensorless.cfg` | **SUPERSEDED** — broken early snapshot; redirects to `homing.cfg`. | do not use |
| `puppybus.cfg` | RS-485 extension first-contact probe. | abandoned approach (`../../docs/klipper_puppybus.md`) |
| `dump_tmc.py` | Helper to dump TMC2130 registers. | |

## Known staleness (why xbuddy.cfg wins)
- **Currents** vary (0.5 / 0.70) — these were torque experiments; verified run
  currents are X 0.55 / Y 0.45 / Z 0.6 / E 0.45.
- **No chopper timing** (`CHOPPER_PRUSAMK3_24V` → TOFF3/HSTRT5/HEND1) and **no ADC
  sample-time fix** — both live only in the real build. Sensorless SGT tuned on a
  test config may shift once those are in.
- Several **headers were copy-pasted** ("SPI comms test"); the table above is
  authoritative over any in-file header.
</content>
</invoke>
