# CHANGES — modifications to Klipper (GPLv3 §5 disclosure)

`coreone-firmware` is a **modified version of Klipper**
([github.com/Klipper3d/klipper](https://github.com/Klipper3d/klipper), GPLv3), the open-firmware
port for the **Prusa Core One+** (STM32F427 xBuddy + STM32H503 xBuddy-extension). It is based on
upstream Klipper **v0.13.0-699** (`c707dd19214709dc23684b254a68e3bf69e4cfb3`).

Per GPLv3 §5, the significant modifications this fork carries on top of upstream are disclosed
below, with the dates they were made. The full **git commit history** in this repository is the
authoritative, dated record; the summary below is the human-readable index. Upstream Klipper's
copyright notices and per-file license headers are **retained unchanged**; the license
([COPYING](COPYING), GPLv3) is unchanged.

## Platform / HAL
- STM32F427 (xBuddy) + STM32H503 (xBuddy-extension) board support; vendored STM32H5 HAL/CMSIS. — 2026-06
- APB2 full-speed (84 MHz) clock tree, for Prusa hardware parity. — 2026-06-27
- BASEPRI critical sections; a priority-0 IRQ tier reserved for phase-stepping (serial/CAN bumped to 1). — 2026-06/07

## Motion / homing
- StallGuard (sensorless) homing. — 2026-06
- MSCNT rotor-phase, temperature-robust *validated* homing (Prusa-style: coarse StallGuard →
  travel-validate/retry → phase-snap), and native `G28.1` home under a `[gcode_macro G28]` override. — 2026-06 … 2026-06-27
  - **`CALIBRATE_HOMING_SGT` reporting rewritten — the old `MSCNT spread` column could not rank SGT and nearly
    drove two bad config changes (host-side, `mscnt_home.py`; REPORTING ONLY, no homing-behaviour change, NO reflash).**
    `spread = max|offset from the sample's mode|` is an **extreme-value** statistic: one bad landing of N sets the
    whole figure, and since the rotor settles on a quarter-period lattice at a background bad-site rate, the column
    mostly reported whether an outlier happened to appear — it ranked SGT **-2 worst at n=5 and SGT 0 worst at n=15**,
    same axis, same afternoon, cold both times (the ±512 slot just moved). It also measured against the sample's *own*
    mode, not the **persisted cal**, so it answered "do these homes agree" rather than "can live homing snap this" and
    hid an off-lattice cal. Now reports the **unsnappable rate** (`|off| > phase_snap_limit` vs the persisted cal) with
    a 95% interval (rule-of-three at both tails), a **site histogram** (makes the lattice visible), max|offset| demoted
    to labelled raw data, and an explicit **RANKING verdict** — the homes-per-SGT needed to separate the observed best
    rate from the worst (~250 for this machine's ~5% vs ~12%, ~40 min each), so it now says *"NOT RANKABLE at n=N —
    treat the ordering as NOISE"* instead of implying a winner. Recommendation logic (one notch off the premature edge
    = hot margin) deliberately unchanged. Added a second warning from an operator observation neither number can catch:
    **SGT +1 rammed the rail audibly while scoring joint-best** — a dead stall is mechanically repeatable, so a ramming
    SGT can score a *better* landing rate than a clean trip; keep MAX ≤ 0 and never sweep above the premature edge
    unattended. Also corrected a stale header claim ("retry-slower" — never implemented; every retry is byte-identical,
    and `coolstep_threshold:45` gates StallGuard above ~54 mm/s vs `homing_speed` 70). New module self-test pins both
    tail boundaries (rule of three at p=0 and p=1), the ~250-homes figure, and the equal/inverted-rate guards
    (`python3 klippy/extras/mscnt_home.py` → PASS). — 2026-10-18
- Phase-stepping executor: **open-loop** TMC2130 XDIRECT commutation on the F427, plus cogging
  compensation. — 2026-06
- Phase-stepping **per-layer registration fix**: round-robin commutation, position-continuous
  segment chaining (POS-FIT), forward-cursor shaped-trajectory build, D1 host back-pressure,
  D7 teleport tripwire (JUMP counter), firmware min-duration merge. — 2026-07-05
- Phase-stepping **MCU-side input shaping**: the input-shaping convolution moved from the host onto
  the F427 (`phase_shaper.c`); the host now streams raw un-shaped axis trapezoids and the MCU
  convolves + projects to CoreXY in the 40 kHz ISR (`MCU_SHAPE`, default-on/auto, host-shaped path
  retained as fallback). Requires the F427 built **hard-float** (`-mfpu`/`-mfloat-abi=hard`,
  `MACH_STM32F427` only; CPACR enabled early in `armcm_boot.c`; `STACK_SIZE`→2048). Offline
  equivalence gate in `coreone/test/shaper_golden/`. — 2026-08
  - ⚠ **On the gate counts below ("N gates green").** The suite includes `stale_arrival_test.py`, which
    is a *reproduction*, not a fix-check: it passes by **confirming** that an already-elapsed motion window
    still loses its motion (on-time +40.000 mm vs late 0.000 mm) when driven straight at the MCU evaluator,
    which genuinely cannot play a window whose time has passed. That loss is **irreducible** — the host no
    longer *acts on* a dead window (a past-window segment is now neutralised to a hold and the next live
    segment re-anchors — see the entry below, gated by `stale_neutralise_test.py`), which removes the LURCH
    and keeps the loss visible (`xy_trusted=0`, which gates `PRINT_END`'s park), but it cannot retro-play the
    elapsed motion. So `stale_arrival_test.py` stays green as a standing characterization of that limit. The
    takeaway for every "N gates green" count below: it means **N characterization checks hold, not N defects
    fixed** — one of them pins an unrecoverable physical limit, not a bug awaiting a patch. — 2026-10-19
  - Added `coreone/test/shaper_golden/run_all.sh` — the canonical gate runner: it enumerates `golden.py` **plus**
    every `*_test.py`, prints the count it intends to run before running, and fails if it finds none. **Prefer it over
    an ad-hoc `for t in *_test.py` glob**, which silently omits the `golden.py` oracle gate (the MCU-evaluator
    equivalence check — it *does* assert, `sys.exit(main())`), so a glob reports "all green" while never running the
    most important gate. `./run_all.sh -q` prints only failures and the summary. — 2026-10-31
    - **`run_all.sh` now gates the README against the filesystem, and the 16 missing gate rows were written (docs +
      tooling, no functional change).** The README gate table had silently drifted to documenting 6 of 22 gates — a map
      wrong about two-thirds of what it maps. `check_readme_coverage()` fails (exit 3, after the gates themselves pass)
      when a `*_test.py` gate has no backticked row in `README.md` **and**, in the other direction, when a row names a
      `*_test.py` that does not exist (so a future gate deletion drifts the same way and is caught too). `./run_all.sh
      --docs` runs the coverage check alone (instant, no harness build). No layout tolerance: the check keys on gate
      **filenames**, identical in both the overlay (flat `src/`) and fork (`src/stm32/`) layouts, so it is
      layout-agnostic by construction — and the README now carries no path, so it is byte-identical overlay↔fork (the
      expected-seam set drops from three files to two: only `golden.py`'s `ROOT` and `harness.c`'s include still differ
      because those encode where the tree sits). The 16 rows were sourced from each gate's own docstring and final
      PASS line, not recollection; the single table was split into five sections because four gates (`gc_counters`,
      `gc_refreeze`, `gc_watchdog`, `estall_telemetry`) exercise host-side `extras` and never touch the C harness, so
      the old "every sibling drives the same harness" lead-in was false for them. All 22 gates + the coverage check
      green via `run_all.sh` from the fork tree (and the check reproduces the real drift it was built for: the pre-fix
      README fails at 16 missing, the fork's old asymmetric copy at 15). Authored in the overlay (`pfb-klipper`
      `4bbf03771`) per the single-fork inversion rule and folded overlay→fork here. — 2026-11-02
- Phase-stepping engage: **clear the TIM8 NVIC pending latch on re-arm** (Prusa BFW-8383). Clearing
  `TIM8->SR` at re-arm kills the peripheral flag but not the NVIC pending bit (and `armcm_enable_irq` is
  SetPriority+EnableIRQ only, no ClearPending), so a `UIF` latched just before disengage could fire one
  tick early at the next engage → a single mistimed commutation frame. Two `NVIC_ClearPendingIRQ` in
  `phase_exec.c` (`pe_tim8_start`/`pe_tim8_stop`); justified by code inspection, re-engage exercised
  clean on hardware. — 2026-08-30
- Phase-stepping slave-stats diagnostic (host-side, `phase_exec.py`): dropped `seg_dry` from the
  starvation warn condition. An engaged-but-idle ring is dry by definition, so it false-alarmed
  ("position ratchet risk") on healthy manual engages; the honest starved signals
  (`overflow`/`jump`/`refills>0`/`minrep==0`) remain and `dry` is still printed as data. — 2026-08-30
- Phase-stepping trapq **gap-skip / pause-resume XY correctness** (host-side, `phase_exec.py`), after a
  bench move under phase-stepping finished ~65 mm off its mark with every slip counter reading zero:
  - **gap-skip fix** — on a queue-drain resume, find the first *real* move in the drained span and resume
    there instead of past it, so the move is kept rather than dropped. *Hardware-proven* (the head now
    returns on the mark); also offline-gated. — 2026-09-25
  - **`_bridge_gap`** — a multi-minute engaged idle left a ring hole longer than the signed-32 clock window
    (`phase_shaper.c:122`), so the evaluator raised `PE_EVAL_BEFORE` continuously (`lateBEF=65045`, a field
    doc'd "Must be 0"); the hole is now bridged with bounded hold segments. Offline regression gate
    `coreone/test/shaper_golden/gap_bridge_test.py` (180 s hole: 3.15M BEFORE → 0). Fixes the BEFORE storm,
    **not** the motion loss above. — 2026-09-25
  - **STALE late-arrival detector** + a `get_status()` (`gap_skips`/`stale_segs`/`xy_trusted`) that surfaces
    a segment arriving after its clock window as a loud stop-stat rather than a silent XY offset;
    reviewed-and-reasoned (has not yet fired). The `[gcode_macro]` park guards (PRINT_END / PAUSE) now gate
    on `stale_segs` — a deliberate relaxation of the prior unconditional refuse, on operator instruction. — 2026-09-26
  - **park-guard fail-open closed (`PHASE_XY_CHECK`).** Gating the park guard on `stale_segs` alone left a
    hole: a real M600 pause lost XY the *other* way — the MCU-side shaper ran out of history and held while
    the host position advanced (`lateBEF` high, `stale_segs` still 0), so `xy_trusted` read 1 and PRINT_END
    drove an absolute park to a position the machine did not have. Now a `PHASE_XY_CHECK STEPPER=<n>` mux
    command refreshes an on-demand `late_before` cache from the MCU (get_status is Moonraker-polled, too hot
    for a per-read round trip) and `xy_trusted = 0 if (stale_segs or late_before)` — both loss paths feed one
    verdict. printer.cfg calls `PHASE_XY_CHECK` then reads the verdict in the `_PRINT_END_PARK`/`_PAUSE_PARK`
    sub-macros (a gcode_macro's Jinja renders once, so the refresh must precede the read). The failure it
    closes is hardware-observed; the fix itself is reviewed-and-reasoned, not yet exercised by a real paused
    print. — 2026-09-26
  - **re-anchor-during-PAUSE rail strike closed (host-side, `phase_exec.py`).** A gap span longer than
    `GAP_MAX` (1 s) unconditionally set `_reanchor_next`, and a re-anchor rewrites the MCU position origin to
    the host's *commanded* position. When an estall PAUSE parked commanded XY to the park target while the
    gantry physically sat elsewhere (phase-stepping left engaged), each flush re-anchored toward the park
    target — driving the gantry into the rail repeatedly, for minutes, with no gcode and nothing in
    `klippy.log` (only `gap_skips` rising on a live query). Re-anchor is now gated on motion *actually
    crossing* the skipped span (`_span_has_move()`); a pure-idle span is still bridged (telescoping-clock
    invariant) but not re-anchored. Hardware-observed failure. ⚠ Two caveats: (1) where the MCU genuinely
    held a move (the open `lateBEF` case) physical position is no longer snapped to commanded, so that
    desync now **persists** rather than being silently papered over — `xy_trusted` is what must catch it (a
    visible desync over a hidden one plus a rail strike, but a trade); (2) **`lateBEF` is still unfixed** —
    short pauses (M600) should now behave, but a long pause still wants `PHASE_STEP=0`. Not "the phase-step
    pause bug is fixed". — 2026-10-09
  - **idle keep-alive — second rail strike closed (host-side, `phase_exec.py`).** The 2026-10-09 re-anchor fix
    bridged a pure-idle span but still relied on a *flush* to refresh the MCU segment ring; once the toolhead
    went fully idle under an engaged PAUSE nothing refreshed it, so the newest segment aged past the signed-32
    clock window (`phase_shaper.c:122`, 2^31 ticks = 12.783 s) and the evaluator began commanding that
    segment's *start* position — driving Y into the left rail once every ~25 s with the host silent
    (`print_time` frozen, `buffer_time=0`, five lines in `klippy.log`). A reactor timer (`_KEEPALIVE_S` = 4 s)
    now emits a contiguous zero-velocity hold from the ring head to the present whenever ENGAGED and idle,
    keeping the head within 8 s of now; `ra=0` on every hold (re-anchoring here was the *first* strike,
    2026-10-09) and it bridges only to the present, never future-dating (the 65 mm-offset failure, 2026-09-25).
    Offline C-evaluator gate `coreone/test/shaper_golden/idle_keepalive_test.py` (host-silent IDLE drifts
    −9.823 mm → KEEPALIVE +0.000 mm, zero bursts) + `keepalive_unit_test.py` (21 emitter checks); the three
    existing gap/stale/golden gates still pass. ⚠ Deployed to the Pi but **not yet live-validated** on the
    machine (needs G28 + `PHASE_STEP_ON` + a sustained idle). ⚠ The MCU evaluator itself is still defective —
    the 2^32 replay arrives through the in-segment branch, so this host keep-alive is a mitigation, not the
    cure; an MCU-side staleness latch (a separate build+flash, host chat's lane) is still owed. — 2026-10-11
  - **MCU-side staleness latch — second, independent defence (⚠ REFLASH-REQUIRED, F427 firmware + host).**
    The keep-alive above is host-side and cannot cover the host *itself* failing (klippy dies, USB drops, the
    reactor starves past the keep-alive period). `pe_axis_eval` (`src/stm32/phase_shaper.c`) now refuses to
    evaluate once the segment ring has been *uncovered* longer than `PE_STALE_S` (8 s) — returns
    `PE_EVAL_STALE`, and `pe_motor_step` holds. Sticky: only a newly arrived segment clears it, and the clearing
    tick raises `PE_EVAL_REANCHOR` so the commutation anchors to the angle already being driven instead of
    snapping to the new segment's `start_pos` (a slipped pole — what the 2026-10-11 strike sounded like). The
    measured quantity is the length of the current uncovered *episode* (timed from the clock it began),
    deliberately **not** the age of the newest segment — segments are routinely future-stamped (a bridged
    3-minute pause spans 180 s, overflowing 2^32 seven times), so a newest-segment-age metric reads garbage and
    would false-latch a healthy slow move. A new command pair `phase_exec_stale_query`/`phase_exec_stale_state`
    (`src/stm32/phase_exec.c`) surfaces events/latched/age; the host (`klippy/extras/phase_exec.py`) queries it
    in `PHASE_XY_CHECK`, folds `mcu_stale` into `xy_trusted`, and reports a dedicated MCU-STALE status line. The
    host lookup sits in its **own** try/except so a host newer than the firmware loses only the latch's
    diagnostics (`mcuSTALE=n/a(no latch in fw)`), never the shaping. Offline gate
    `coreone/test/shaper_golden/stale_latch_test.py` (10 checks: no-latch drifts −9.823 mm, latch holds
    +0.000 mm and latches at 8.008 s, latch+keep-alive never fires); golden equivalence unchanged (latch never
    reached on a healthy stream). ✅ **HW-VALIDATED 2026-10-12** (reference machine, operator present): an induced
    ring-starvation run (`SET_PHASE_KEEPALIVE PERIOD=30` + ~80 s host silence, klippy alive and observable)
    confirmed engage at ~8 s, position held unmoved through the episode, and a **clean clear with no pole slip** —
    the `PE_EVAL_REANCHOR` edge re-seeded the commutation with zero gantry motion (`snapMAX=0.000 mm`), machine
    still and quiet throughout *including the clear*, `xy_trusted` staying 0 after recovery (a stale episode does
    not auto-forgive — a re-home is required, the intended fail-safe). ⚠ **Scope:** validated for
    engage/hold/clear/no-snap via an induced starvation *with the host alive*; a genuine host **death** (USB drop,
    klippy crash — which would likely also trip Klipper's own "Lost communication with MCU") is a different path
    and stays UNTESTED. The host keep-alive remains HW-validated. ⚠ **Does NOT fix stale late-arrival** — an
    in-band 15–20 s late segment still loses its full travel; the latch only makes that loss *reported*
    (`xy_trusted=0`) rather than silent. Stale-arrival stays OPEN. — 2026-10-11 (HW-validated 2026-10-12)
  - **keep-alive period made configurable — so the MCU latch can be tested at all (host-side,
    `phase_exec.py`; firmware-inert, NO reflash).** The latch above only engages once the ring is
    uncovered past `PE_STALE_S` (8 s), and the host keep-alive exists precisely to stop that ever
    happening — so with both defences live the backstop is unreachable by idling and could only be
    bench-tested. A `keepalive_period` config option (default **4.0**, unchanged behaviour) plus a
    `SET_PHASE_KEEPALIVE PERIOD=` live mux command (mirrored onto the CoreXY slave like
    `SET_PHASE_IDLE`, re-arming immediately so a shortened period takes effect now) let an operator
    lengthen the period past 8 s to deliberately starve the ring and exercise the latch's engage →
    hold → clear. **≥8 s is a TEST-only setting** — the command warns loudly (≥8 s: the latch *will*
    engage and hold, XY commanded during the hold will not happen, re-home after; >5 s: false-latch
    margin) and the period is deliberately **not** clamped below 8 s, since clamping would make the
    latch permanently untestable. ⚠ The load-bearing part: the hold-chunk cap is now **decoupled**
    from the period (`_ka_chunk() = min(period, _BRIDGE_MAX)`), where it used to *be* the period —
    correct only by the coincidence that 4.0 < 8.0. A configurable period breaks that coincidence; a
    naïve 30 s chunk is 5.04e9 ticks (past 2^32) and would corrupt `trel` outright — re-creating the
    rail-strike fault from inside the fix. The timer *cadence* follows `_ka_period`; the segment
    *size* stays capped at `_BRIDGE_MAX` (8 s = 1.344e9 ticks < 2^31). Regression-pinned in
    `keepalive_unit_test.py` (a 30 s period still emits ≤8 s chunks). Also a diagnostic fix:
    `PHASE_SHAPE_STATS` no longer prints `thresh=0.0s | dry=covered` before the first engage (the
    `pe_axes[]` fields are only populated at engage) — it now gates on `bound` and says MCU-STALE is
    unbound, the same reporting-provenance class as `mcuSTALE=0` on a firmware with no latch. This
    does **not** change default behaviour and does **not** make the latch validated — it only makes
    validating it possible; the latch remains bench-only on hardware. — 2026-10-12
  - **XY-trust guard — `xy_trusted` now propagates to the CoreXY slave on every path, including
    fail-closed (host-side, `phase_exec.py`; SAFETY, firmware-inert, NO reflash).** `xy_trusted` is a
    per-object host cache (`_late_before`/`_mcu_stale`/`_stale_segs`) of what is largely **global** MCU
    state — `phase_exec_shape_state`/`_stale_state` report the same `pe_axes[]`/`pe_shape_*` statics
    whichever oid asks. But the caches were refreshed only on the object `PHASE_XY_CHECK` actually
    queried, so the CoreXY **slave** kept reporting `xy_trusted=1` straight through a real staleness
    episode, and an unrefreshed `1` is indistinguishable from a verified one. Observed live 2026-10-12
    right after the latch validation: `stepper_x` read `mcu_stale=2`/`xy_trusted=0` while `stepper_y`
    read `0`/`1` at the same instant. Nothing was at risk only because the live park guards happen to
    name `stepper_x` — the macro's choice of axis, not a property of the guard — so a guard naming the
    slave (or any new one) would read "trusted" through the fault. Same false-trusted-feeds-absolute-park
    family as the 2026-09-26 near-miss. Fix: a `_mirror_trust_to_slave()` helper pushes `_late_before`
    and `_mcu_stale` onto the slave, called on **all four** exits of `cmd_XY_CHECK` — including the two
    fail-closed returns (after the `-1` sentinel is stored) and the no-MCU-shaper return — so the
    invariant is unconditional: after `PHASE_XY_CHECK` returns, both axes carry the same verdict. The
    fail-closed half is the part with teeth (a master that knows it cannot tell, beside a slave still
    claiming `1`, is the same false-trusted guard with extra steps). `_stale_segs` is deliberately **not**
    mirrored — only the master streams, so copying it would invent a fault on the slave. No behaviour
    change on the master. Gates unchanged (`cmd_XY_CHECK` is a gcode handler no golden/gap/stale path
    exercises; full 6-suite re-run green). — 2026-10-12
  - **Saturating 16-bit diagnostic counters — so `lateBEF==0` cannot be a wrapped residue (⚠ REFLASH-REQUIRED,
    F427 firmware + host).** Every diagnostic counter on the shaping path (`src/stm32/phase_shaper.c`,
    `phase_exec.c`) is `uint16_t` and increments *per tick while its own exceptional condition holds* — under a
    40 kHz ISR a sustained episode advances it ~40000/s, so a bare `++` wraps back **through 0 every
    65536/40000 = 1.638 s**. That is a safety defect for exactly one of them: `PHASE_XY_CHECK` grants
    `xy_trusted` on `lateBEF == 0`, and `PRINT_END` then drives an **absolute park to X242 Y211** on that
    verdict. A counter that returns to its own "healthy" value every 1.6 s can therefore authorise a park on a
    position the machine may not have — the 2026-10-09 rail-strike consequence, reached by a route the
    2026-10-11 staleness latch does not cover. Seen for real: on the completed 4 h 10 m print 0001CD
    (2026-10-12) `PHASE_XY_CHECK` logged `lateBEF=0 → xy_trusted=1` and the disengage seconds later logged
    `lateBEF=11631` from the **same** MCU field — it had wrapped between the two reads; and two logged
    **near-misses** came within a hair of a false grant — `lateBEF=57157` on an M600 runout and `lateBEF=65045`
    on a magnet-insert pause, the latter just **490 counts (~12 ms) short of the 65535 ceiling** that reads as
    healthy (`phase_exec.c` lines 642, 1229, 1572). Fix: `pe_cnt_inc`/`pe_cnt_add` inline helpers in
    `src/stm32/phase_shaper.h` saturate at `PE_CNT_MAX` (0xFFFF); all 15 bare `++` sites converted (6 in
    `phase_shaper.c`, 9 in `phase_exec.c`), and the host (`klippy/extras/phase_exec.py`) renders the ceiling as
    `>=65535` via `_cnt()` at all 12 reporting sites including `PHASE_XY_CHECK`. **Saturating, not widening** —
    the fields live in `struct pe_axis`/`struct phase_exec`, and the asymmetry is deliberate and matches the XY
    guard's existing choice: over-reporting costs a re-home, under-reporting costs the frame. The one 40 kHz
    hot-path site (`pe_shape_reanchors`, formerly an unconditional `+=` every tick) is now branched around when
    the edge is 0, so it is *cheaper* than what it replaced; every other site already sat behind an exceptional
    condition. New offline gate `coreone/test/shaper_golden/counter_saturation_test.py` self-calibrates the
    per-tick rate, asserts EXACT below the ceiling, `== 65535` above it, and asserts explicitly that the value
    is **not** the wrapped one (a revert to `++` reported `lateBEF=12287` and the gate named it by cause). All 7
    gates green from the fork tree; target build `+144 B` (F427), the whole size delta and accounted for. ⚠
    **REFLASH-REQUIRED** on the F427; the H503 extension is unchanged. Behaviour-neutral otherwise (counters and
    reporting only, no control path touched). ⚠ Corollary: `dry`/`hold`/`past`/`bef` values in any log captured
    on the *pre-fix* firmware are residues, not counts — the 21-min dry-run `dry=2` in the bringup doc is
    demoted to unproven accordingly. — 2026-10-12
  - **`lateBEF` teardown verdict — stop crying "XY IS SILENTLY WRONG" over a parked tail; plus a GC-pause
    fix for "Timer too close" (host-side, `phase_exec.py` + new `extras/gc_tune.py`; firmware-inert, NO
    reflash).** Two parts, both host-only. **(a)** The disengage banner asserted "XY IS SILENTLY WRONG" on any
    nonzero `lateBEF`, but the real fault needs **two** things — the MCU shaper holding **and** the host
    advancing commanded position while it holds — and the banner asserted the second unconditionally. These
    counters tick in real time whether or not the host is commanding motion, so a parked, stream-idle
    `PRINT_END` tail accrues `lateBEF` with nothing to lose (holding a stationary position is a no-op). That
    false alarm fired for real on the 2026-10-12 A1 print: `PHASE_XY_CHECK` read `lateBEF=0` mid-print, the
    teardown printed `lateBEF=11631`, and every tick of it accrued in the ~14 s after the last move with the
    stream down to 108–176 B/s of pure housekeeping — `net=0.0000 mm`, `JUMP=0`, `snapMAX=0`, `segERR=0`, part
    dimensionally fine. New `_latebef_verdict(lb)` downgrades to benign **only** when measured dormancy accounts
    for the whole count, via a new `_last_motion_wall` (stamped in the flush callback when it actually advanced
    `_raw_last_xy`, seeded at engage, and **mirrored to the CoreXY slave** — only the master flushes, so an
    unmirrored clock reads idle-since-epoch and would downgrade *every* warning on the slave). It fails closed
    three ways: a value at/above `PE_CNT_MAX` is a **bound, not a count** so it can never buy a downgrade (this
    is where it composes with the saturating-counter fix above — `11631 + 7×65536 ticks ≈ 11.76 s`, the honest
    reading the pre-fix wrap hid); the dormancy is discounted by `_LATEBEF_IDLE_GUARD_S` (1.0 s, since the flush
    stamps ahead of execution); and no tick-rate (never engaged) means no credit, so it warns. **(b)** New
    `extras/gc_tune.py`: `klippy.py` runs `gc.disable()` + `Reactor(gc_checking=True)` and collects by hand,
    escalating to a level-2 whole-heap pass once the gen1/gen2 counters cross 10. That pass ran **268.7 ms**
    (2026-10-12) and **258 ms** (2026-09-12); each time klippy sent and read nothing for ~360 ms, then flushed a
    heater soft-PWM update whose clock was already **in the past** (33.8 / 17.4 ms), and `src/gpiocmds.c`
    hands it to `sched_add_timer` with no past-clock check → `sched.c` `try_shutdown("Timer too close")` →
    MCU shutdown mid-print, twice, with every host-load metric clean. On `klippy:ready` the module does
    `gc.unfreeze()` → `gc.collect(2)` → `gc.freeze()` (the unfreeze is **load-bearing**: klippy re-creates the
    `Printer` object in the same process on a restart, so freezing without releasing first would leak an entire
    printer graph per restart). Measured live: level-2 pass **93.3 ms → 0.0 ms**, on-demand now 0.4–0.5 ms. The
    two events bracket an 8× spread in session size yet produced near-identical 258/269 ms passes, so the cost
    is dominated by the long-lived **startup** graph — which is exactly what `gc.freeze()` removes from the walk
    — not accumulated runtime garbage (corollary: "restart klippy between long prints" is nearly useless; the
    10-12 process was only 22.1 h old). The `[gc_tune]` config section lives host-side in
    [coreone-host](https://github.com/packerlschupfer/coreone-host)'s `printer.cfg`, not here. New offline gate
    `coreone/test/shaper_golden/latebef_verdict_test.py` (13 checks: the real A1 case downgrades, a storm with
    no idle tail still screams and still says re-home, the exact tick boundary, the guard discounts, a ceiling
    value warns even after an hour parked, a missing interval warns, and the unmirrored-slave hazard is pinned);
    teeth-checked (forcing the verdict fail-open turns it red, 9 checks). All 8 gates green. Sequenced strictly
    **after** the saturating-counter fix (it reads `PE_CNT_MAX`); firmware-inert, so the `.py` half converges on
    the host independently of any flash (`systemctl restart klipper`). — 2026-10-13
  - **TMC bus-shadow returns per-register benign values — a longstanding latent bug that shut the printer down
    (host-side, `phase_exec.py`; SAFETY, firmware-inert, NO reflash).** While phase-stepping owns the TMC SPI bus,
    `_set_tmc_checks` shadows each driver's `get_register` so a stray read can't collide with the executor's XDIRECT
    stream. The shadow returned **one constant for every register** — `mt.get_register = lambda *a, **k: 0x001f0000` —
    with a comment reasoning that GSTAT's named bits (reset/drv_err/uv_cp) were all zero in that value. True and
    irrelevant: `TMCErrorCheck.__init__` builds `gstat_reg_info = [0, reg_name, 0xffffffff, 0xffffffff, 0]`, so GSTAT's
    `err_mask` is `0xFFFFFFFF` and **any** nonzero value is an unconditional error — the named bits are never consulted;
    and on TMC2130 `clear_gstat` is False, so `_query_register` can't even attempt a clear, it loops three times and
    raises. Triggered on hardware **2026-10-15**: a second `PHASE_STEP_ON` produced
    `TMC 'stepper_x' reports error: GSTAT: 001f0000 → Transition to shutdown state` — where the `001f0000` was this
    sentinel, never read off the bus (recovered via `FIRMWARE_RESTART`; `DUMP_TMC` after showed `GSTAT 00000000`, no
    driver damage). The single constant can't serve both registers: DRV_STATUS **must** be nonzero (`cs_actual=0x1F`,
    since `tmc.py` reads `cs_actual==0` as a driver reset and also raises), while GSTAT must be 0. Fix: a module-level
    `pe_tmc_shadow_get` returning a per-register map (`{'GSTAT': 0}`, default `0x001f0000`), installed in place of the
    lambda. **Longstanding latent bug, not a regression** — the line dates to the original phase-stepping host import
    (`44781f8ee`, 2026-06-21, ~4 months before the counter-saturation/latebef work); safe across many prints because a
    first engage in a fresh klippy session does not hit it, and the exact trigger path for the GSTAT read during
    suppression is still unidentified (the fix is placed at the shadow, the catch-all layer, not at the trigger). New
    offline gate `coreone/test/shaper_golden/tmc_shadow_test.py` replays `tmc.py`'s real `_query_register` control flow
    (not return values — the masks live in `tmc.py`) and has teeth built in: it asserts the old one-constant shadow
    **still** raises on GSTAT and that a plain `0` would raise on DRV_STATUS, so a value-only revert fails loudly.
    Bundled with it (same tip, host-only): a **keep-alive witness** — `_ka_fires`/`_ka_maxlag` in `get_status` and the
    teardown banner (`KA=%d(fires=%d maxlag=%s)`), updated **after** emission inside the protective try/except so a
    broken witness costs the counter, not the hold — which lets `KA=0` be read as "watching and not needed" rather than
    "timer dead" (the first real healthy dry episode measured `maxlag=4.90s/4.00s`, confirming ~3.1 s of margin below
    `PE_STALE_S`=8 s); and a corrected `MCU_SHAPE` comment that previously stated both that MCU shaping was and was not
    the default. ✅ The MCU staleness latch additionally got its **first HW validation under a real event** 2026-10-15:
    both steppers read `latched=0x3` together (the slave mirror the 2026-10-12 bug couldn't exercise). ✅ **RESOLVED
    (2026-10-18): the lateBEF-vs-latch "contradiction" was never one** — the two mechanisms are independent. `covered`
    is set per-impulse, so one tick can be *covered* (an impulse inside a segment → the latch can't arm) **and** *BEFORE*
    (another impulse's lookback straddling off the front of a sparse ring); `phase_exec.c`'s guard excludes STALE but not
    BEFORE. So the latch bounds the *uncovered* episode the signed-32 wrap makes dangerous, while lateBEF *also* counts a
    benign **retention** condition the latch neither sees nor should suppress. The 2026-10-12 A1 print (`lateBEF=11631`
    **with** `MCU-STALE events=2`, `net 0.0000 mm`, `JUMP=0`) was a dimensionally-fine part, not a bypass. Pinned by the
    new `latebef_vs_stale_test.py` gate; `_latebef_verdict` is **vindicated, not retired**. — 2026-10-18
  - **`gc_tune` gains a reactor-latency watchdog, and a diagnostic can no longer cause the pause it reports
    (host-side, `extras/gc_tune.py`; firmware-inert, NO reflash).** The shipped 139-line module always ran a real
    `gc.collect(2)` on `GC_TUNE_STATUS` and had no `COLLECT`/`RESET` handling — and since Klipper silently ignores
    unrecognised gcode params, a `printer.cfg` whose `PRINT_END` calls `GC_TUNE_STATUS COLLECT=0` (as the live host
    config does) got a full generational collection **with heaters live** — the exact ~260 ms pause family behind both
    "Timer too close" shutdowns. The rewrite (340 lines) makes `COLLECT=0` and `RESET=1` run **no** collection, and adds
    a watchdog that measures reactor lateness against the **deadline** (not the previous fire) and re-arms from **now**
    (so a stall can't owe a burst of catch-up fires), with cumulative buckets, a warn-once, context capture (print
    state / sd_pos suppressed when not printing / print_time), and a `get_status` headline for Moonraker. New offline
    gate `coreone/test/shaper_golden/gc_watchdog_test.py` pins both latency traps, the no-collect contract for
    `COLLECT=0`/`RESET=1`, the warn-once, and the stale-sd_pos suppression. The `[gc_tune]` config section stays
    host-side in [coreone-host](https://github.com/packerlschupfer/coreone-host). All 10 shaper gates green from the
    fork tree; the MCU half is unchanged, so the published F427 assets and their reflash status are untouched by this
    tip. — 2026-10-18
    - **`GC_TUNE` exposes read-only allocation counters in `get_status` — the gen-2 stall mechanism is now observable
      mid-print without triggering it (same file; OBSERVABILITY, no behaviour change, NO reflash).** The stall's
      mechanism is the *cost* of a gen-2 collection pass, and that cost can only be measured by **running** a pass
      (`GC_TUNE_STATUS COLLECT=1`, ~93–240 ms here) — doing that mid-print makes the instrument the fault, so the only
      live signal was `late_max_ms`, the *consequence* not the mechanism. Three new fields, all read-only and
      **non-collecting** (neither `gc.get_count()` nor `gc.get_stats()` traverses or collects): `gc_counts`
      (`list(gc.get_count())` — ⚠ only `[0]` is an object count, allocations-minus-deallocations since the last gen-0
      collection: allocation **pressure**, not heap size. `[1]`/`[2]` are **collection counters** — gen-0 collections
      since the last gen-1, gen-1 since the last gen-2 — NOT object counts; an earlier description of all three as
      "pressure per generation" was wrong [corrected with `24f904a00`]), `gc_collections` (cumulative passes per generation from
      `gc.get_stats()` — `gc_collections[2]` answers "has a gen-2 pass even run yet", which lateness cannot), and
      `gc_uncollectable` (objects GC gave up on — a **leak canary**, must stay flat). `gc.get_count()` is a plain tuple
      read, kept **uncached** so pressure is instantaneous; `gc.get_stats()` builds three dicts and is throttled behind
      `_STATS_TTL = 1.0` (`get_status` is hot — Moonraker polls it several times a second — so an unthrottled call
      would add allocation pressure to the exact hot path this module exists to measure) and **wrapped** so
      instrumentation can never break a status query. New gate `coreone/test/shaper_golden/gc_counters_test.py`
      (6 arms; **ARM C** pins that a status query NEVER collects — else the observability patch becomes a source of the
      stall it observes — and ARM D pins the throttle; teeth-verified against the pre-patch blob at 13 failures). **All
      17 shaper gates green** from the fork tree; MCU sources byte-unchanged → host-only, no reflash. **This is the
      instrument, not the fix:** the periodic-re-freeze repair is not written — `gc_uncollectable` is the canary for the
      leak that fix will carry (re-freezing makes frozen-then-dead objects unreclaimable). Authored in the overlay
      (`pfb-klipper` `46276840b`) per the single-fork inversion rule and folded overlay→fork here. — 2026-10-31
    - **`gc_tune` re-freezes periodically so the level-2 pass cost stays flat — the repair the counters were the
      prerequisite for (same file; BEHAVIOUR CHANGE, firmware-inert, NO reflash; ✅ HARDWARE-VALIDATED 2026-11-01, 3/3 — see below).** The
      at-ready freeze only fixed the startup graph; post-ready allocation stays under the collector, so the gen-2 pass
      cost climbs back during a print (measured offline **0.83 → 5.41 ms** over five rounds without re-freeze; flat
      **0.80–0.84 ms** with it). The mechanism costs nothing *by construction*: freezing an uncollected heap would make
      its cyclic garbage permanently uncollectable, and calling `collect(2)` ourselves mid-print **is** the 258–269 ms
      stall — so instead a 2 s poll watches `reactor.get_gc_stats()[2]` (reactor.py already collects by hand and records
      each level's eventtime) and calls `gc.freeze()` right after a level-2 pass it did **not** pay for. **The module
      itself never collects.** Three measured properties back it: `gc.freeze()` is O(1) (0.004 ms at 800k objects), a
      freeze resets `get_count()` so escalation restarts and passes get *rarer*, and the first post-ready pass lands
      after ~48k allocations (no unprotected window). `GC_RECLAIM` keeps a **hard refusal while printing** (its ~125 ms
      cost is precisely the documented "Timer too close" failure of 2026-09-12/2026-10-12; `FORCE=1` overrides when the
      heaters are known off); the re-freeze timer re-arms from `monotonic()` so a stall cannot queue a burst of
      catch-up polls. New status fields `refreeze`/`refreeze_available`/`refreezes`/`refreeze_frozen_added`/
      `refreeze_last_ms`; **`refreeze_frozen_added` is the LEAK BOUND** — every re-frozen object is permanently
      uncollectable if it later becomes a reference cycle, so it is the number to watch against klippy RSS, not a
      success metric. New gate `coreone/test/shaper_golden/gc_refreeze_test.py` (**ARM B** pins that the poll calls
      `collect()` zero times across 40 ticks; teeth-verified against the pre-patch blob) + `gc_counters_test.py` updated;
      **all 18 shaper gates green** from the fork tree; MCU sources byte-unchanged → host-only, no reflash.
      ✅ **HARDWARE-VALIDATED 2026-11-01 — 3 of 3 durability prints passed the full bar** (operator at the machine; see
      the pass bar below, every criterion met on all three). Closing level-2 pass **1.3 / 0.9 / 1.4 ms** (prints 1/2/3)
      against **93.4–93.6 ms** at-ready pre-freeze on the same instances and the **188.2 / 258 / 269 ms** that killed
      prints — the mechanism holds the pass cost at the post-freeze baseline over 4.15 h. The three samples are
      INTERNALLY CONSISTENT with a prediction registered *before* print 3 ran: print 3 (cold, fresh process) matched
      print 1 (cold) at 1.4/1.3 ms rather than continuing print 2's warm 0.9 ms, because the restart reset the permanent
      generation — three measurements, not three absences of failure. `late_max_ms` 17.562/13.138/17.383 (all < 50,
      `late_over_100ms`==0), `gc_uncollectable`==0, no "Timer too close"; RSS flat while `refreeze_frozen_added` rose
      (much frozen, nothing leaked). **Pre-registered hardware gate** (the single normative pass bar this validation was
      judged against — quote THIS, not a restated copy), ALL required and ALL met: **(1)** `refreezes > 0` — else inconclusive,
      the mechanism was never exercised; **(2)** `gc_collections[2]` must **ADVANCE**, with `refreezes` tracking it
      ~1:1 — *not* hold at baseline (an idle machine satisfies "at baseline" trivially, so that criterion cannot fail);
      **(3)** `late_max_ms` < 50 and `late_over_100ms` == 0; **(4)** `gc_uncollectable` == 0 and no "Timer too close";
      **(5)** klippy RSS bounded — ~108 MB at deploy with a few hundred kB of idle jitter (no single VmRSS reading is a
      precise baseline), so the leak criterion is **sustained monotonic growth over a print alongside rising
      `refreeze_frozen_added`**, never a single delta; **(6)** run in the **normal default config** (no phase-step
      toggle, which would confound); **(7)** sustained over **3 prints ≥ 2 h**. Also corrects three doc errors in this module's status
      comments that earlier framing (including the `46276840b` entry above, now fixed) had carried: only `gc_counts[0]`
      is an object count ([1]/[2] are collection counters); `gc` is disabled in klippy so there are **no automatic
      passes** and `gc_automatic` reads 0 by design; and the leak is frozen-then-dead **reference cycles** specifically
      (ordinary dead objects refcount away while frozen — measured 202.4 → 11.6 MB), not dead objects in general.
      Authored in the overlay (`pfb-klipper` `24f904a00`) per the single-fork inversion rule and folded overlay→fork
      here. — 2026-10-31
    - **`gc_tune` gains a NEVER-RESET session high-water for the worst reactor stall (same file; host-only, no
      reflash).** `PRINT_START` calls `GC_TUNE_STATUS RESET=1`, so every `late_max_ms` figure is per-print: on
      2026-11-10, after 4 d 6 h of uptime spanning several prints, `late_max_ms` read *lower* than three days earlier
      because the stats had been zeroed between — a worst-case that falls while uptime grows, which is exactly the
      signature of a reset hiding a slow degradation. New `late_max_session_*` fields, fed from the same watchdog
      sample (so the two can't disagree) and deliberately reachable by **no** `RESET=` argument — a high-water a macro
      can zero is not one. ⚠ Scope stated precisely: "since the last klippy *restart*", not process-lifetime (a
      `FIRMWARE_RESTART` rebuilds the GCTune and starts the mark at 0). The session fields are created in `__init__`
      (via `_wd_session_init`), **not** lazily at arm-time — a prior lazy version raised `AttributeError` from
      `get_status()` on a `watchdog:False` config, which webhooks turns into an unhandled-exception shutdown (a Mainsail
      subscribe would have shut the printer down). Gate `gc_session_highwater_test.py` (incl. ARM E: the no-samples-yet
      branch must still surface the session mark); `gc_counters_test.py`'s fake reverted to mimicking `__init__` only
      (an earlier "fix" had taught it to call `_wd_session_init`, silencing the detector instead of the fault). Already
      live on the Pi (host-only). Authored in the overlay (`pfb-klipper` `801803459`) per the single-fork inversion
      rule and folded overlay→fork here. — 2026-11-10
  - **Engage-path GSTAT race closed + a failed-engage unwind (host-side, `phase_exec.py`; SAFETY, firmware-inert, NO
    reflash).** A second, distinct route to the same "TMC reports error: GSTAT" shutdown as the 2026-10-15 bus-shadow
    bug — this one in the *enable* path. The engage issues `SET_STEPPER_ENABLE`; `tmc.py` answers the enable-line
    change with a **queued** `register_callback(enable_disable_cb)`, and `_do_enable` → `echeck_helper.start_checks()`
    reads GSTAT **synchronously** before arming its timer. The register shadow used to go up ~100 lines later with
    reactor yield points between, so the queued callback could land while `get_register` was shadowed but `start_checks`
    was still live — reading the shadow's GSTAT, which Klipper masks `0xFFFFFFFF`, can't clear on a TMC2130, and raises →
    shutdown (the engage that died on 2026-10-15 has a Stats line between its two "Run Current" lines; the survivor does
    not). Fix by **ordering**: `_set_tmc_checks` gains `regs=` so the engage shadows `start_checks` **before**
    `SET_STEPPER_ENABLE` and the registers later — the fatal combination (registers shadowed, checks live) is now
    unreachable by construction, not merely unlikely. Plus a **failed-engage unwind**: `cmd_TRAPQ_ENGAGE` is now a thin
    wrapper whose `_engage_undo` ledger **doubles as an arming flag** — it stays `None` through the re-entry guard, so a
    double `PHASE_STEP_ON` (or mid-print `CALIBRATE_PHASE_COGGING`) that raises while an earlier engage is still
    streaming does **not** restore the bus underneath it (an earlier draft restored unconditionally and had exactly that
    worse-than-the-race bug — the two commits are inseparable). A partial engage now restores the checks and clears
    `direct_mode` (motors enrolled in the ledger **before** the write, since `set_register` can raise *after* the flip
    lands — otherwise a failed axis is left silently in `direct_mode`, accepting step/dir without moving until
    `FIRMWARE_RESTART`). New gates `tmc_enable_race_test.py` (asserts checks are shadowed no later than the registers at
    every point, reproduces the one-constant shadow to prove teeth) and `engage_unwind_test.py` (a failed engage clears
    `direct_mode`, a raised-but-landed write is still unwound, the re-entry guard can't reach into a running print); the
    independent `latebef_vs_stale_test.py` resolves the lateBEF caveat above. **All 13 gates green** from the fork tree.
    ✅ **Validation status, plainly:** the **engage path was exercised on hardware** (2026-10-18, operator present):
    a clean engage/disengage at bed centre — no motion, no GSTAT trip, `direct_mode` cleared on both motors, no
    shutdown — and the operator **visually confirmed the gantry held** (the reported figures are all *commanded*
    position, so a physical snap would not show in them; his eyes are the discriminating instrument). Two honest
    limits remain: that engage ran under the FAST interleaving (no reactor pass landed mid-engage — the 2026-10-15
    *survivor* path), so it establishes that the rewrite does not break the normal path, **not** that the race is
    reproduced-and-fixed — the race is closed **by construction** (`start_checks` shadowed no later than the
    registers) and covered by `tmc_enable_race_test.py` (incl. the source call-site order), not by a reproduced
    failure; and the failed-engage **unwind path did not fire** (nothing failed), so it rests on
    `engage_unwind_test.py` alone. — 2026-10-18
  - **Disengage now clears the stale `homed_axes` flag (host-side, `phase_exec.py`; makes the mandatory re-home
    ENFORCEABLE, firmware-inert, NO reflash).** `PHASE_STEP_OFF` detaches the CoreXY itersolve and re-bases the
    open-loop XY position — which is exactly why the disengage banner **demands** a G28 — but the step-gen restore
    path (reached from both disengage and `PHASE_EXEC_STOP`) re-bases position via `th.set_position(th.get_position())`
    without homing or clearing the homed flag, so `homed_axes` still read `'xy'` afterwards. Nothing enforced the
    required re-home: a macro gating on `homed_axes` got a **stale-true**. Fix: `th.get_kinematics().clear_homing_state("xy")`
    after the re-base (the canonical clear path, same call `force_move`/`safe_z_home`/`stepper_enable` use; drops
    `homed_axes` to `''` or `'z'`). `"xy"` only — disengage perturbs only the CoreXY motors, Z is untouched, so no
    needless Z re-home; and **unconditional** — the re-base runs regardless of `net`, so a net-conditional clear would
    re-introduce the stale-true in the `net==0` case (exactly the clean-run signature). **Follow-up (same path, made
    observable):** the re-base and the clear used to share ONE bare `except: pass`, so a re-base failure silently
    **skipped** the clear — the worst case, since a failed re-base makes the XY reference *less* trustworthy, exactly
    when you don't want the axis left marked homed. They are now **independent** try blocks (the clear runs regardless;
    fail-safe is "unhomed", never "still homed"), and each **logs its consequence** instead of swallowing (re-base →
    "Internal error in stepcompress", restart; clear → `homed_axes` is STALE-TRUE, re-home). Both still swallow rather
    than raise — a disengage must complete or it strands `direct_mode` on the drivers (a silently-dead axis) — but
    they now say so. That also makes the hardware check **two-directional**: "clear worked" and "clear never reached"
    no longer both produce an empty log. New offline gate `disengage_clear_homing_test.py` (6 arms, 19 checks — the
    independence arm pinned, each failure distinguishable, engage direction never clears; teeth-verified against the
    real pre-fix blob). No behaviour change when nothing fails; **all 14 gates green**. ✅ **Validated on hardware**
    (2026-10-19, operator present; Pi at `ce3c19fab`, phase_exec blob `333cff2a`): a supervised engage →
    `PHASE_STEP_OFF REHOME=0` dropped `homed_axes` `'xyz' → 'z'` — XY cleared, Z left homed (observed: no needless Z
    re-home), the exact `"xy"` the gate asserts. **Reproduce with `REHOME=0`:** `PHASE_STEP_OFF` defaults `REHOME=1`
    and runs a full G28 right after the disengage, which re-homes everything back to `'xyz'` (Z lifted) inside the same
    macro — the clear *does* fire but is overwritten in-macro, so a plain `PHASE_STEP_OFF` will mislead you into
    thinking it failed. The except-split made the result two-directional: zero `RE-BASE failed` / `STALE-TRUE` /
    tracebacks across two engage/disengage cycles is a *real* negative, not an empty log. This run exercised the
    **normal path only** — the failed-re-base and failed-clear branches never fired (nothing failed), so their
    evidence remains `disengage_clear_homing_test.py`, not this run. — 2026-10-19
  - **Stale-arrival: the host no longer *acts on* a dead window — past-window segments neutralised, next live
    segment re-anchors (host-side, `phase_exec.py`; firmware-inert, NO reflash).** When a motion-carrying
    segment's clock window has already elapsed beyond `_stale_slack`, emitting it is what turns a loss into a
    **lurch**: the MCU either drops it (lost) or freezes at the segment *endpoint*, which at the gantry is an
    instantaneous jump (measured as the 40 mm event — the class that rams a rail). The fix **neutralises** it
    instead — strips velocity/accel and sends it as a HOLD carrying only its own start position — and sets a
    new `_stale_reanchor` flag so the next segment that still carries motion re-seeds the MCU phase origin
    (`reanchor=1`); the flag is cleared on engage. **Why neutralise and not skip** (the obvious fix is wrong):
    dropping the send punches a HOLE in the ring, and the emitter's contract is that clocks **telescope** —
    each segment's `start_clock` is the previous one's exact end clock, which `pe_axis_eval`'s forward cursor
    requires, and whose signed-32 compare goes garbage past 12.783 s @168 MHz. A hole is exactly what produced
    the 3,151,364-event `PE_EVAL_BEFORE` storm. Neutralising keeps `start_clock` and duration **identical**, so
    the ring stays contiguous while the window commands nothing (a late HOLD is harmless — that is why the
    staleness counter only ever counted segments carrying velocity/accel). The re-anchor is **load-bearing, not
    cosmetic**: after a neutralised run the MCU sits at the held position while the host's belief has advanced,
    and without re-seeding it would cross that difference in one segment interval — reintroducing the exact
    lurch. **Three honest limits.** (1) It does **not** recover the lost motion — an elapsed window is
    unrecoverable; what it removes is the lurch, and it keeps the loss **visible** (`xy_trusted=0`, re-home
    demanded). "Stale-arrival solved" would be false; "the host no longer acts on a dead window" is true.
    (2) **Not hardware-validated**: deployed and loaded (Pi `phase_exec.py` blob `940c3312`, klippy ready, zero
    tracebacks, object responding), but a dead window cannot be forced on demand, so the trigger path has never
    fired on the machine — evidence is the gate plus the ordering argument, the **same standing as the TMC
    enable-race entry** above; cite the gate, not a hardware run. (3) `stale_arrival_test.py` still prints
    "STALE-ARRIVAL REPRODUCED … lost 40.000 mm", and that is **correct** — it drives the MCU evaluator, which
    genuinely cannot play an elapsed window, so it stays a characterization check, not a regression. New gate
    `stale_neutralise_test.py` (7 arms, 22 checks), teeth-verified against the real pre-fix blob `333cff2a`
    (5 failures = exactly this fix's own assertions — velocity not stripped, no re-anchor, old log wording —
    while the contiguity, counting, slack, late-hold and inert-when-slack-0 arms still pass there, so the gate
    discriminates this change and nothing else). **All 15 gates green** from the fork tree; MCU sources
    byte-unchanged → host-only, no reflash. Authored in the overlay (`pfb-klipper` `feature/9b-h503-rs485`
    `8a0fad519`) per the single-fork inversion rule and folded overlay→fork here. — 2026-10-26
  - **lateBEF trust guard closed — a non-benign MCU lateBEF seen during a report now actually drops
    `xy_trusted` (host-side, `phase_exec.py`; SAFETY, firmware-inert, NO reflash).** `xy_trusted` is
    `0 if (stale_segs or late_before or mcu_stale)`, but the inputs were **not symmetric**: `_stale_segs` is
    host-side and counted as it happens, so it drops trust immediately, whereas `_late_before` is MCU-side and
    **cached**, written in exactly one place (`cmd_XY_CHECK`). The teardown report read lateBEF fresh from the
    MCU to render its banner and then **discarded the verdict** (`_benign, latewarn = …`), so a run could PRINT
    "XY IS WRONG" while `xy_trusted` still read `1` — the same **fail-OPEN** shape as the 2026-09-26 incident,
    where `stale_segs=0 → xy_trusted=1` let `PRINT_END` drive an absolute park to X242 Y211 on a position the
    machine did not have. (That is why the earlier "SILENTLY WRONG" wording could not be fixed by rewording —
    on this path it was *true*.) Fix: `_adopt_latebef(lb, benign)` adopts a **non-benign** lateBEF into
    `_late_before` and mirrors it to the CoreXY slave (a fail-closed verdict must propagate), so the banner and
    `xy_trusted` agree; it persists until the next ENGAGE (which zeroes the caches), landing the untrusted
    verdict in the idle window where the park guard actually reads it. A **benign** lateBEF is deliberately NOT
    adopted — the parked-tail case is real and common (2026-10-12: lateBEF=11631 accrued entirely after the
    last move, part dimensionally fine), and adopting it would invent a fault and make `PRINT_END`'s park guard
    skip for nothing. ⚠ **Scope qualifier (not overstated):** this closes the guard *where a report queries the
    MCU*, not continuously — outside a report, `xy_trusted` still only tracks lateBEF after a `PHASE_XY_CHECK`
    refresh, so an unrefreshed `1` proves nothing. Gate `latebef_verdict_test.py`: the `"SILENTLY WRONG"`
    assertion was **moved** to the new semantics (must say XY IS WRONG, must NOT say SILENT, must name
    `xy_trusted` and `PHASE_XY_CHECK`) — moved, not relaxed, and commented so nobody later reads it as a
    weakened test — plus new arms pinning `_adopt_latebef` (non-benign adopted, benign not, `lb==0` never).
    **All 15 gates green** from the fork tree; MCU sources byte-unchanged → host-only, no reflash. Not
    hardware-reproduced (the trigger cannot be forced on demand); evidence is the gate plus the real 2026-09-26
    incident shape, the **same standing as the TMC enable-race entry**. Authored in the overlay (`pfb-klipper`
    `7d6af503b`) per the single-fork inversion rule and folded overlay→fork here. — 2026-10-26
  - **Teardown diagnostics snapshot the MCU counters BEFORE the disengage stops its own keep-alive (host-side,
    `phase_exec.py`; REPORTING ONLY, no behaviour change, NO reflash).** `cmd_TRAPQ_DISENGAGE` stopped the keep-alive as
    its first action and read the shape/stale counters only after, so the teardown reported a small window it had just
    created. New `_counter_snapshot()` reads the counters FIRST, before the keep-alive stop; `_report_shape_stats(…,
    snap=…)` reports the snapshot, falls back to a live query for standalone `PHASE_SHAPE_STATS`, and **warns** rather
    than silently using the old contaminated read if the snapshot fails. ⚠ It does **not** keep the keep-alive live
    through the unwind — only *when* the counters are read changes, not teardown behaviour. New gate
    `disengage_snapshot_test.py` (7 arms; **ARM B** pins the snapshot-before-stop ORDER). ⚠ **Scope, corrected (do not
    over-read this):** this was first believed to fix the end-of-print "XY IS WRONG" cry-wolf. It does **not** — the
    disengage window is only ~198 of the ~39135 observed ticks (0.5%); the false alarm recurred with this live. The
    banner fix is the **next** entry (`ebeb0d7f2`). This commit is still correct and is **KEPT, not reverted** — and is
    now load-bearing for a different reason: its pre-teardown snapshot is what lets the verdict read the MCU dry age
    without an extra round-trip on the disengage path. Authored in the overlay (`pfb-klipper` `cf56e8f86`) per the
    single-fork inversion rule and folded overlay→fork here. — 2026-11-01
  - **The end-of-print "position desynced / XY IS WRONG" cry-wolf fixed, properly — lateBEF credited against the MCU's
    own DRY EPISODE (host-side, `phase_exec.py`; REPORTING ONLY, no behaviour change, NO reflash; ⚠ deployed but
    UNVALIDATED).** `_latebef_verdict` excused a nonzero lateBEF only if `now − _last_motion_wall − 1 s guard` covered
    it — but lateBEF accrues at `tick_hz` for exactly as long as the ring is dry, so accrual == dormancy *by
    construction*, and the guard then demanded `dormancy ≥ dormancy + 1 s`: **structurally unsatisfiable for the parked
    tail it was written for.** Worse, `PRINT_END`'s park move stamped `_last_motion_wall` ~1 s before the teardown, so
    `idle_s` read 0.0 and credited nothing → false "XY IS WRONG" at every print end, which `_adopt_latebef` then turned
    into `xy_trusted=0`; the *same* counter was ruled benign 51 s later once nothing had flushed, i.e. the verdict was
    **time-of-asking dependent**. Fix: credit lateBEF against the MCU's own dry episode (`x_age`/`y_age` — the window
    the ticks accrued in, in the same clock they are counted in, so no flush-lead correction and no guard), resolved
    from the `cf56e8f86` snapshot (no extra round-trip) with a live fallback. **Fail-closed:** benign only if the last
    flush was at/before the episode start (`_last_motion_wall ≤ now − dry_s`); a stamp *inside* the episode is motion
    commanded-and-held = real loss, still flagged. Plus a still-time accumulator (`_latebef_still_sum`, banked per
    flush, reset only on `lb==0`/engage, mirrored to the slave) that closes a related fail-open on the dormancy
    fallback. Saturated counters stay un-creditable (the 11631 lesson). New gate `latebef_stilltime_test.py` (10 arms;
    **ARM H** is the real-data arm — print 3's verbatim `lb=39135`/`dry=0.98 s` must be benign AND the same input
    *without* `dry_s` must still flag; **ARM C** pins time-of-asking independence at +1/+3/+10/+51/+300 s, which the
    pre-fix blob fails `[F,T,T,T,T]`). **All 20 gates green** via `run_all.sh`; MCU sources byte-unchanged → host-only,
    no reflash. ⚠ **Fired on print 4 (2026-11-01) but it credited a REAL loss as benign — a latent FAIL-OPEN, now
    closed.** The teardown credited `lateBEF=32275` to the MCU's own 0.81 s dry episode and printed no "XY IS WRONG" —
    but the gantry had not parked (the segment-duration truncation below), so "nothing was lost" was false. Root of the
    mis-credit: this fix credits lateBEF against the **current** dry episode, and its fail-closed check
    (`_last_motion_wall <= now − dry_s`) can PASS while the ticks and the loss belong to an **earlier** long episode
    that already closed — `dry` only ever shows the current one. Same attribute-to-the-wrong-window shape as the
    diagnoses around it, this time inside the repair. **Closed 2026-11-02 by `5fc763dd9` (the latch VETO):** if the MCU
    staleness latch fired at all during the run (`events > 0`), or its count can't be read (`None` — *unknown is not
    zero*, fail-closed), NO benign path may credit the ticks; `events` is resolved from the same stale payload as
    `dry_s` at the same instant so veto and credit can't disagree about which run they describe. Gate
    `latebef_latch_veto_test.py` (5 arms): **ARM A** is print 4 verbatim (`lb=32275, dry=0.81 s, events=2`) → NOT benign
    (teeth: the pre-veto code called that real loss benign); **ARM D** keeps the benign path reachable at `events=0`, so
    the veto can't later be "fixed" into unreachability and resurrect the every-PRINT_END false alarm. ✅ **SHIPPED +
    DEPLOYED; logic covered by ARM A only; HARDWARE EXPOSURE NIL by construction.** `_latebef_verdict` short-circuits
    `if not lb: return True` *ahead* of the veto, and `lateBEF==0` on a healthy machine (the truncation is fixed), so
    the veto is UNREACHABLE on a healthy print — not merely dormant. A "banner stayed silent" result therefore confirms
    only the plumbing (the event count resolves without throwing), NEVER the veto decision; the veto's only test is
    ARM A, and exercising it on hardware would mean deliberately starving the ring (not done). There is deliberately no
    hardware-validation line for it. **Residual, coherent:** `MCU-STALE
    events=2` was **ONE** uncovered episode counted **per-axis** (`events = x_events + y_events`; on CoreXY both axes
    read the same ring; `dry X=Y` identical = one shared episode); the "2×8 s" framing is retired as mis-framed. ⚠ **The
    ≥8 s PRINT_END episode is now root-caused — see `78bcbfdf8` below — to the segment-duration truncation, NOT the
    keep-alive race.** The `_keepalive_event` "no idling can reach the backstop with both defences live" claim was FALSE
    for *that* reason; an earlier revision of this changelog attributed it to a keep-alive race and called it fixed by
    `dac9428b7` — that was wrong (that fix is inert; see its entry). A separate pending fix will report the per-axis
    split (`x=1 y=1`) instead of a summed "events" count. (The summed presentation carried a one-episode event as a
    two-episode mystery for a day — the reusable shape: arithmetic/counters that check out but are attributed to the
    wrong window or unit, refuted only by measurement.) Authored in the overlay (`pfb-klipper` `ebeb0d7f2`) per the
    single-fork inversion rule and folded overlay→fork here. — 2026-11-01
  - **Keep-alive emitted at HALF the period — the MCU-latch race had ZERO margin (host-side, `phase_exec.py`; SAFETY,
    firmware-inert, NO reflash; ⚠ deployed but UNVALIDATED).** The keep-alive bridges the segment ring during idle so
    the MCU staleness latch (`PE_STALE_S` = 8 s) never fires — but emitting at the FULL period gave a worst-case
    first-emit latency of **2 × `_ka_period` = 8.0 s** (the lag must climb from 0 to the threshold = one period, and
    the timer only *checks* each period, so a tick can land just under = another period) against the **8.0 s**
    threshold: **0 s margin**, so the latch could fire before the first bridging hold ever landed. That is the ≥8 s
    uncovered PRINT_END episode measured on print 3, and — this unifies two findings previously filed as
    *incommensurable* — the **same** 4+4-vs-8 collision as the pause-side `maxlag 7.59 s vs latch 8.0 s` already on
    record: **one** cause, not two. Fix: `_KA_EMIT_FRAC = 0.5` emits a bridge at half the period → worst case
    **1.5 × `_ka_period` = 6.0 s, i.e. 2.0 s of margin**. ✅ **Steady state UNCHANGED** (after an emit `t0 := est`, so
    lag restarts ~0 and the next tick sees `lag == _ka_period`, clearing either threshold — one emit per tick before
    and after, only the FIRST-emit latency moves, so no new MCU traffic); `_BRIDGE_MAX` (8.0) still bounds a single
    span, and `SET_PHASE_KEEPALIVE` above 8 s still reaches the latch (period 10 → worst 15 s), so the documented test
    knob survives. **Found from data already on disk** — print 3's teardown carried `KA=0(fires=3732 maxlag=-0.40s)`,
    i.e. the timer ran throughout but emitted ZERO segments (correct while printing, the toolhead buffers ahead), which
    is why the first-emit latency the old comment leaned on had **never once been exercised**. ⚠ Also **corrects a
    FALSE comment** (not merely optimistic): the old `_keepalive_event` note "with both defences live no amount of
    idling can exercise the backstop" — idling is precisely what exercised it; the correction carries the arithmetic
    beside the constant. New gate `keepalive_margin_test.py` (4 arms; **ARM A** recomputes the worst case from the live
    constants and reads `PE_STALE_S` out of the MCU header rather than hardcoding 8.0, so any later edit raising
    `_ka_period`/`_KA_EMIT_FRAC` or lowering `PE_STALE_S` reopens the gap and FAILS; it defaults a missing
    `_KA_EMIT_FRAC` to 1.0 so the pre-fix blob reports `worst 8.0 s, margin 0.0 s` and fails). The gate finds the header
    via a recursive `src/` search (`_find_shaper_h`) so it survives both the flat overlay layout and the fork's
    `src/stm32/` nesting — a first cut hardcoded `src/phase_shaper.h` (overlay-only) and FAILED in the fork tree, caught
    by `run_all.sh` before it shipped and re-folded to the search. `keepalive_unit_test.py` updated — two assertions
    encoded the old full-period rule (the zero-margin design itself); retargeted below the new boundary AND a case
    added pinning the boundary FIRES, so it is held from both sides, not relaxed to match. **All 21 gates green** via
    `run_all.sh` **run from the fork tree**; MCU sources byte-unchanged → host-only, no reflash. ⚠ **INERT — this is
    NOT the fix for the PRINT_END episode (determined on print 4, 2026-11-01).** The teardown showed `KA=0` with
    `maxlag` never reaching even the *old* 4 s threshold, so lowering it to 2 s changed nothing — the gate it loosens is
    never hit on this path. The 4+4-vs-8 zero-margin computation is correct *as arithmetic* but describes a threshold
    crossing that does not occur; the real cause is the segment-duration truncation (next entry). **Kept, not reverted:**
    a lower emit threshold can only ever fire *sooner*, it is harmless, and `keepalive_margin_test.py` still guards
    against anyone reopening the margin later. Just not a cause or a cure. Authored in the overlay (`pfb-klipper`
    `dac9428b7`, gate layout-fix `33118274f`) per the single-fork inversion rule and folded overlay→fork here.
    — 2026-11-01
  - **🚨 Segment durations capped at 2³⁰ ticks — the gantry had NOT been parking on short prints for 19 days (host-side,
    `phase_exec.py`; SAFETY, firmware-inert, NO reflash; ⚠ deployed, physical validation pending).** The MCU packs the
    re-anchor flag into **bit 31 of the duration field** (`src/stm32/phase_exec.c:1486` ORs `PE_SEG_REANCHOR 0x80000000`
    into the duration; `PE_SEG_DUR_MASK 0x7fffffff`), and the host emitted the duration as a plain uint32 with **no
    range check** — so any segment ≥ 2³¹ ticks (12.783 s @168 MHz) arrived **truncated and with the re-anchor flag
    spuriously set**. `PRINT_END`'s presentation lift (`G1 Z{zpres} F900`, `zpres = max(axis_max.z−50, z+20) = 220 mm`,
    ~15 mm/s) is **Z-only**, so it emits ONE XY hold for the whole lift: on a 7 mm-tall part that is a 213 mm / ~14.2 s
    hold = 2.386×10⁹ ticks → the MCU saw a 1.42 s hold plus a bogus re-anchor, the segment ring stopped being covered,
    the staleness latch engaged 8 s later, **and the end-of-print park segments were never evaluated — the head was
    left MID-BED instead of at X242 Y211 (operator-confirmed 2026-11-01).** Present on **every short print** (any part
    **< 28.3 mm tall**) since `f4aa0db35` (2026-10-13) raised the lift floor `168 → zmax−50`; at the old floor the
    longest lift was 11.2 s and could never reach the ceiling. **This is the true root cause of the ≥8 s PRINT_END
    uncovered episode** — the keep-alive race (`dac9428b7`) and the earlier "XY-agnostic reference" hypothesis were
    both plausible-arithmetic misses. Fix: **`PE_SEG_DUR_CAP = 2³⁰`** (6.39 s, a factor of two under the ceiling)
    applied at **all three** long-capable emit sites — the gap hold, the move, and the trailing hold — with each piece
    **recomputed from the move's own constant-accel parameters at its own offset** (never propagated, so no drift and
    `golden.py`'s ULP comparison still holds), clocks telescoping, re-anchor on the **first piece only** (duplicating it
    rewrites the MCU origin per piece — the 2026-10-09 rail strike). New gate `seg_duration_cap_test.py` (5 arms;
    **ARM A** is the check nobody ran on 10-13 — a 14.20 s Z-only lift, assert every duration < 2³¹; pre-fix it reports
    one 2.386×10⁹-tick segment and FAILS. An append-site **enumeration** arm found the *third* emit site a first patch
    left uncapped at up to 30 s). ⚠ **`golden.py` did NOT cover this** — its longest scenario is ~24k ticks against a
    2³⁰ cap, so it proved no-regression and said nothing about the overflow path; **that coverage gap is how this
    shipped for 19 days, and is the reusable lesson.** **All 22 gates green** via `run_all.sh` from the fork tree; MCU
    sources byte-unchanged → host-only, no reflash. ✅ **VALIDATED — PHYSICALLY + MECHANISM-LEVEL 2026-11-02 (operator
    at the machine, 10 mm part < 28.3 mm threshold).** Head confirmed at **X242 Y211** after a post-fix phase-stepped
    short print. Before→after: `MCU-STALE events 2→0`, `dry 0.81s→covered`, `lateBEF 32275→0`, **`reanch 2→0`** (those
    two re-anchors were the SPURIOUS ones minted by the overflow into bit 31 — gone because nothing reaches bit 31 now;
    that is what ties the fix to the cause), banner "XY IS WRONG"→nothing, head MID-BED→X242 Y211. `dry=covered` means
    the ring was never uncovered = mechanism confirmation, not just symptom-absence. Found by an independent subagent
    given the symptom data and none of the (five wrong) prior conclusions; the decisive evidence was the operator's eye
    on the physical park. Authored in the overlay (`pfb-klipper` `78bcbfdf8`) per the single-fork inversion rule and
    folded overlay→fork here. — 2026-11-01
  engaged (shared SPI3). — 2026-06 … 2026-07-03
  - optional **shadow telemetry** (`CRASH_DETECT SHADOW=1`): MCU-side accumulation of the gated-in
    `sg_result` distribution + a would-trip counter, reported to the host on disarm instead of
    shutting down — for tuning `sg_floor`/`crash_sgt` from real prints. Off by default. — 2026-07-26

## Sensing
- HX717 load cell as loadcell **Z-probe**; extruder **filament-presence** sensor on the same HX717
  (channel B), with a 12:1 channel **interleave** so both run during a print. — 2026-06 … 2026-06-28
- Extruder filament sensor **armed for runout** (not insert-only), per Prusa's design of running
  runout on both the hotend and spool sensors independently, with an **edge gate** in
  `_eval_presence` (`hx71x_filament_sensor.py`; port of `filament_sensor.cpp:19-34`): no edge event
  is emitted across a non-working window (out-of-range / uncalibrated / the probe holding HX717
  channel A) — the new state is adopted silently, so a first-after-resume sample cannot fire a false
  runout mid-print. — 2026-09-26
- Loadcell-based extruder **clog / stuck-filament detection** (`estall_detect`; 5-tap FIR over the
  loadcell force stream). — 2026-06-28
  - **Shadow-telemetry histogram re-binned 80 → 2100 — the report percentiles were quantised into noise
    (host-side, `estall_detect.py`; REPORTING ONLY, no detection change, NO reflash).** The per-print
    `filtered`-edge histogram used `TELE_NBINS = 80`, so the bin width was `3*thr/NBINS = 26250` raw counts at
    the default `thr = 700000` — wide enough that >90 % of a print landed in a single bin and `p50`/`p90` read
    identical run-to-run, reporting the bin **centre** rather than the data (seen as percentile values exactly
    one bin apart). Now `TELE_NBINS = 2100` (1000-wide); `_tele_pct` returns the exactly-tracked `_tele_min` /
    `_tele_max` for the two **unbounded** end bins (bin 0 and the last clamp everything outside `[lo, hi)`, so a
    bin-centre estimate there is a constant regardless of the true tail) instead of that constant; and the
    report adds `p1` / `p10` and echoes `bw=` (bin width). ⚠ **Comparability:** `min`/`mean`/`max`/`would-trip`
    are tracked exactly *outside* the histogram and stay comparable across the bin-count change, but the
    **percentiles do not** — a bin-centre estimate shifts up to half a bin (±13125 counts between 80 and 2100),
    so diffing a new `p50` against a pre-patch log reads quantisation as a real change (hence the `bw=` echo).
    **Reporting only: it changes no detection behaviour, no threshold, and cannot alter whether a print trips.**
    New gate `estall_telemetry_test.py` (7 arms) — teeth-verified against the pre-patch file (10 failures; the
    sharpest: the old code returned `p1 = -336875`, the bin-0 centre, for a sample set whose true minimum is
    −5e6). **All 16 gates green** from the fork tree; MCU sources byte-unchanged → host-only, no reflash.
    Authored in the overlay (`pfb-klipper` `9f1f6aca1`) per the single-fork inversion rule and folded
    overlay→fork here. — 2026-10-26
    - **End-bin percentile guard made size-robust (same file; LATENT, no behaviour change today, NO
      reflash).** `_tele_pct`'s top-bin test used the module constant `TELE_NBINS - 1` while the loop walks
      `self._tele_hist`; the two agree only because `_tele_reset` allocates `[0] * TELE_NBINS`, so the
      constant was correct *by coincidence*, not by construction. If the histogram sizing ever diverged, the
      test would silently stop matching and the unbounded top bin would fall through to the bin-centre formula
      — reporting exactly the constant that branch exists to avoid, with no visible break. Now keyed to
      `len(self._tele_hist) - 1`. Gate `estall_telemetry_test.py` gains ARM G, which **forces** the divergence
      (a 10-bin hand-built histogram with everything in the top bin) because a test against today's code passes
      on both versions and proves nothing — teeth-verified against `5596129ba` (ARM G there returns −340500.0,
      the bin-centre constant, instead of the tracked 4242). **All 16 gates green**; MCU sources byte-unchanged
      → host-only, no reflash. Authored in the overlay (`pfb-klipper` `88686af6a`) per the single-fork
      inversion rule and folded overlay→fork here. — 2026-10-26
- Per-tool **filament / material profiles** (`[filaments]`): a port of Prusa's `config_store` filament
  model — the preset table from `src/common/filament.cpp`, `M865`, and the mismatch guard from
  `marlin_print_preview.cpp` (`check_correct_filament_type`). Per-tool throughout (for INDX). Runtime
  state lives host-side in `save_variables`; the module is inert unless `[filaments]` is configured. — 2026-08-30
  - `get_status` exposes a clear state view — `material_unknown` / `must_declare_material` / `has_parameters`
    (old `known` / `awaiting_material` retained as deprecated aliases so config can migrate independently),
    fixing a UI trap where a named-but-preset-less material read as "not set". Plus a **material-change
    purge**: a longer purge (`PURGE_LEN_CHANGE = 60`) when the loaded material actually changes vs the
    standard prime (`PURGE_LEN_NORMAL = 20`) on a same-material reload — tracked via `prev_material` (kept
    across the removal's NONE interlude) and a new `FILAMENT_PURGED` command that `LOAD_FILAMENT` calls after
    purging. The 60 mm is 2× a measured ~30 mm PLA→PETG colour transition; the purge code itself is
    **reviewed-and-reasoned, not yet field-exercised**. — 2026-09-26
- Loadcell **pressure-advance bench probe** (`loadcell_pa_probe.py`): a calibration tool (enable temporarily
  in `printer.cfg`) that drives a slow→fast extrude step in air across a list of PA values and captures the
  HX717 nozzle-loadcell force, writing one CSV + a summary line per PA, to pick an optimal pressure-advance K.
  Reuses the estall HX717 ch-A client and suppresses estall around the sweep. Bench/diagnostic, **not a
  print-path module**; honours `soft_cancel` via `_check_abort`. — 2026-09-26
  - Command help string corrected from the `SPIKE:` placeholder to real operator guidance: the tool
    **extrudes in air**, traces r=40 mm circles, and holds the gcode mutex for 20-30 min — so it must be
    aborted via the `soft_cancel` **webhook**, not the mutex-bound `SOFT_ABORT` gcode (which queues *behind*
    the macro — the 2026-09-26 run that extruded until an emergency stop). Help string only, no code path;
    no gate (nothing references it). Clears the Pi's last held-dirty file so Moonraker `is_valid` returns to
    True. — 2026-10-18

## Peripherals
- ILI9488 colour TFT display type (SPI6 TX-DMA). — 2026-06
  - **MCU-side glyph expansion (reflash-required).** The host used to render each character cell to pixels
    and ship them as ~72 `display_dma_seg` messages per glyph; it now sends the 16-byte mono bitmap +
    position/colour in ONE `display_dma_glyph` message (`src/stm32/display_dma.c`) and the MCU expands it
    into the existing DMA ring — 72 messages → 1, ~98.6% fewer command-link messages per glyph (that flood is
    the same mechanism implicated in the original loadcell shutdowns: `command_task` busy → `sensor_bulk_status`
    late). No MCU font ROM (the host still defines glyphs at runtime, e.g. the heater/chamber icons); no new
    SRAM buffers (expansion streams into the ring in `SEG_MAX` chunks — bss unchanged). `dma_cells_per_flush`
    cap 4→6 (a glyph is 78 ring segments; 6×78=468 ≤ NSEG 512; the cap is now ring-bound, not link-bound).
    The host (`ili9488_display.py`) auto-falls-back to the expanded-pixel path if the MCU predates the
    command, so a host-only deploy cannot break. HW-validated (pixel-identical, visibly snappier). ⚠
    **Unexercised regime:** `menu_dma_cells=32` exceeds the ring (78 segs/cell) so the MCU back-pressures —
    true before too (not a regression), but the spin now happens inside one longer glyph command;
    menu-open-during-a-probe has NOT been re-tested. — 2026-09-28
  - **Wedged-PANEL detection by reading the panel's own status register back — ⚠ REFLASH-REQUIRED; shipped-unvalidated
    at first, VALIDATED on hardware 2026-11-10 (host `ili9488_display.py` + MCU `src/stm32/display_dma.c`; see
    follow-up below).** On 2026-10-25 the panel wedged
    mid-print: six grey rows, no glyphs, **nothing in the log**. The DMA engine was healthy — it was streaming pixels
    at a panel *controller* that had latched up, and every engine-side signal reads normal while the screen is frozen
    (the host only pushes on a cell CHANGE, so an idle screen never fills the ring and never trips the ring-full
    bound). So the detector reads the ILI9488's own status register back over SDO (PG12 MISO) at a **slow 4 MHz** read
    clock (the write clock is 42 MHz; the panel's read cycle min is ~150 ns ≈ 6.6 MHz, so a read at the write clock
    returns garbage — hence a second `spi_config`), a **liveness test of the controller** that the engine counters
    cannot give. `DISPLAY_HEALTH` reports it; the detector never blocks a print (a dead cosmetic LCD must not cost a
    job — `display_dma_wait` already shuts down on expiry, so health is read via new NON-blocking counter commands).
    The engine counters (`tc`/`dmaerr`/`pushes`/`maxcount`/`last_tc`) are added as diagnostic colour. ⚠ **FAILS SAFE
    but UNVALIDATED:** a board that does not route SDO reads all-0x00 / all-0xFF — indistinguishable from a dead panel
    — so the at-ready baseline read **DISARMS** the check and logs why, rather than alarming on healthy hardware;
    whether this board actually drives SDO is **unverified until `DISPLAY_HEALTH` is run post-flash**, so the detector
    may simply be inert and the 10-25 defect stays unobservable. Recorded as shipped-not-fixed at the time — the
    host-crash follow-up below then fixed a connect-time shutdown and the detector was validated on hardware (armed,
    reads the live `0x9c` display-on value, SDO confirmed driven). (Replaces an earlier
    DMA-engine-watchdog design, killed in review by arithmetic: a changed cell costs ~78 ring segments against NSEG
    512, so the 7th change fills the ring and `dd_push` spins → `shutdown("display_dma ring stall")` within seconds on
    a live print screen; a two-poll 30 s detector loses that race every time — which also proved the 10-25 freeze was
    never a dead engine.) Gate `display_health_test.py` — **8/8 mutations killed in the fork layout** (incl. three a
    reviewer found surviving the previous version, and `display_dma.c` deleted outright); resolves the MCU source via a
    recursive `src/` search (`_find_src`, mirroring `keepalive_margin`'s `_find_shaper_h`) so it runs in both layouts.
    All 25 gates green from the fork tree. ⚠ **Reflash reference: `build.sh katapult` = 60496-superseded → the
    canonical fork build is `61132` bytes, sha256 `c47fa66f…`, reproduced on three clean builds at the tag.** —
    2026-11-10
    - **Follow-up — the detector's host half shut klippy down on its FIRST hardware deploy; fixed (host-only, NO
      reflash: `src/` is byte-identical, firmware stays `61132`/`c47fa66f`).** The flash itself was a clean success and
      the MCU half is proven on hardware (the panel answered `display_dma_rdata busy=0 n=3 data=08 00 00`: read clock,
      busy-decline and slow/fast SPI switch all work). But `ili9488_display.py`'s `_hq_ready` issued a **blocking MCU
      query from a `klippy:ready` handler**, and ready handlers run inside `reactor.assert_no_pause()`: the query's
      completion appended its greenlet to `waiting` *before* `pause()` raised, nothing took it back out, the command had
      already reached the MCU, the module's broad `except` logged the reactor's own "pause disabled" diagnostic as a
      retryable blip, ready completed, the connect greenlet retired (`timer = None`), and the ack ~ms later ran
      `complete()` over the stale waiter → `update_timer(None)` AttributeError → shutdown, **2.7 ms after ready**. ⚠ **The
      rule is NOT "never query from a timer"** — two earlier diagnoses said so and both were wrong (upstream `tmc.py`
      queries from a timer every day); it is **never block inside a no-pause section** (`klippy:ready`,
      `klippy:shutdown`, `get_status`, flush callbacks). Fix: `_hq_ready` now only **arms the timer**; the baseline is
      taken by the poll *after* `init()` (so it reads the initialised panel, not the pre-init `0x08` reset default that
      the first deploy sampled); and a `_cannot_block()` guard sits before **every** send so no future caller can
      reintroduce it. ⚠ **Honest lesson for the record:** the fake-reactor gate `display_health_test.py` **passed 66/66
      against the code that crashed** — its fake had no concept of a no-pause section, so it could not express the
      constraint; what caught this was deploying to hardware. New `ready_pause_repro.py` drives the **real** `reactor.py`
      / `Printer._connect` / `raw_send_wait_ack` / gcode dispatcher against a simulated MCU and **reproduces the shutdown
      on the old module and passes on the new** (verified both ways on this fork); `display_health_test.py` gains ARM J
      (ready sends nothing; no send in a no-pause section) + ARM K (busy/None baseline retries, does not disarm). It is
      **not** a `*_test.py` (needs a built `c_helper.so` + `greenlet`), so `run_all.sh` stays 25/25. **A sweep of all
      seven reactor no-pause sections across the module set found no other instance of this bug** — `get_status` in
      seven modules and `phase_exec`'s flush callbacks (which do run inside `motion_queuing`'s no-pause section) are
      plain sends only; measured, not assumed. The pre-fix logs also carried `Internal error - reactor pause disabled`
      **three times**, hardware confirmation of the traced cause. ✅ **VALIDATED ON HARDWARE 2026-11-10 — the detector is
      live (this supersedes the SHIPPED-UNVALIDATED note above).** After the host-only converge (no reflash), klippy
      reached ready in 3 s and held — no crash — and the check armed at +30 s: baseline `reg 0x0a = 00 9c 00`, two
      `DISPLAY_HEALTH` samples both `00 9c 00` = baseline → verdict **panel alive**, register stable, `dmaerr=0`. The
      board DOES drive SDO (so the "may be inert" worry is closed), and the reading has **semantics**, independently
      corroborated against real ILI9488 panels: RDDPM (byte[1]) reads `0x9c` on a fully-initialised awake panel (booster
      on, sleep out, normal mode, **display ON**) and `0x08` at power-up (display OFF / sleep in), so a reset, sleep, or
      wedge demonstrably drops those bits. ⚠ **The detector tests INEQUALITY, not interpretation:** it compares the
      read value against the baseline and warns on any change — it does NOT decode the bits. The bits are meaningful to a
      *human reading the log*, which is why a value change corresponds to a real state change; the detector itself never
      interprets them (and the caught value is partly read-timing-dependent — see the wedge-test follow-up). ⚠ **Boundary
      at the time of this entry (since closed):** a real panel wedge had **not** been induced on live hardware, so the
      catch rested on the RDDPM semantics + gate `display_health_test.py` ARM A + the real-reactor repro's simulated
      wedge — strong inference. **That boundary was later closed — the catch is now DEMONSTRATED on hardware; see the
      `DISPLAY_WEDGE_TEST` follow-up below.** — 2026-11-10
    - **Mechanism that closed that boundary — `DISPLAY_WEDGE_TEST CONFIRM=1` (host-only, NO reflash).** A new gcode
      command that induces a fault **the panel itself
      produces** — `SLPIN` drops RDDPM's sleep-out (bit4) and display-on (bit2), landing on the same `~00 08 00` shape
      as a reset/wedged controller — then drives the **real** `_hq_event` (not a reimplementation), so the production
      warning path is what gets exercised; `SLPOUT`+`DISPON` wake it. The panel command goes **through the DMA engine**
      as a command-phase segment (like the glyph path), never generic SPI, so it cannot drain the ring to a
      `shutdown()` — a cosmetic diagnostic must never cost a print. Guards: `CONFIRM=1` mandatory, refuses mid-print
      (`print_stats` printing/paused), refuses when the detector is unarmed or read-back unavailable. Gate ARM L (15
      checks) pins the guards, the catch-then-recover sequence, and a **FAIL path** — a panel that does not change
      under `SLPIN` reports FAIL, so the test cannot pass itself by construction. The mechanism supplied the test; the
      catch itself is decided by its raw PASS/FAIL on real hardware (see the demonstrated-catch statement below). Suite
      25/25, `--docs` green, gate count unchanged. — 2026-11-10
    - **The wedge test earned its keep on first run — it found a real detector defect, now fixed (host-only, NO
      reflash).** On hardware the induced `SLPIN`/`SLPOUT` cycle left the panel's read state machine WITHOUT its dummy
      byte, so the same healthy register came back `9c 00 00` instead of `00 9c 00` — the value **shifted framing** into
      byte[0]. The old raw-byte comparison read that as a liveness change and re-flagged `PANEL LIVENESS CHANGED` on an
      awake panel, stuck until a `FIRMWARE_RESTART`. That is the exact cry-wolf mode ARM B guards against, reached
      through another door, at the worst timing: a **real wedge that then recovers** is precisely when the framing can
      shift, so the detector would have false-alarmed right after correctly doing its job. Fix: compare the panel
      **VALUE** (`_value()` = first non-zero byte, framing-invariant) — `00 9c 00` and `9c 00 00` both read `0x9c`,
      while a genuine `00 0c 00` → `0x0c` still differs; a framing shift with an unchanged value now logs INFO and
      **adopts the new frame** (no warning, not re-logged), and a real value change still warns in **either** framing so
      the catch is not blunted. `DISPLAY_HEALTH`'s verdict moved to the value too. Gate **ARM M** pins all three
      (framing shift silent, real change warns in either framing, `_value` unit cases). Suite 25/25, `--docs` green,
      gate count unchanged. — 2026-11-10
    - **Detector CATCH: DEMONSTRATED on hardware (clean re-run against the framing fix, 2026-11-10) — recorded as two
      separate statements, because they have different evidence behind them.**
      **(1) The catch is demonstrated.** `DISPLAY_WEDGE_TEST CONFIRM=1` at standby drove the production `_hq_event`
      warning path, caught the induced panel fault, and cleared on recovery — **twice, with two different stimuli**
      (`0x9c`→`0x0c` full sleep, and `0x9c`→`0x98` display-off-but-awake), new-code-live proven by the klippy `MainPID`
      swap. The `PANEL LIVENESS CHANGED` warning is emitted strictly **after** the `SLPIN` in source execution order
      (`cmd_DISPLAY_WEDGE_TEST`: `_panel_cmd(SLPIN)` then `_hq_event`, which is the one and only site that logs it); a
      klippy.log line-order inversion is a two-sink flush artefact (immediate `logging.warning` vs deferred
      `respond_info` mirror) and timestamps cannot settle it — source order can. The caught value is **nondeterministic**
      (`0x0c` vs `0x98` — the 150 ms read catches a panel-state transition in progress), which is exactly why the
      detector tests **inequality, not interpretation**.
      **(2) The framing-absorption fix (`_value()`) is GATE-COVERED ONLY — not hardware-observed.** The clean re-run's
      post-recovery silence happened with **no framing shift present**, so there was nothing to absorb: that silence is
      *consistent with* the fix, not *evidence for* it (a clean result from a condition that never arose). It is pinned
      by gate `display_health_test.py` ARM M in both directions (a framing shift is silent; a real value change still
      warns in either framing) and deliberately **not chased further on hardware** — the trigger is nondeterministic, the
      logic is a few lines pinned both ways, and the failure mode is a *false alarm*, not a *missed catch*, so repeated
      sleep/wake cycles on a live panel would be the wrong trade. — 2026-11-10
    - **Detector no longer cries wolf on our OWN idle-dim (host-only, NO reflash).** The liveness check logged
      `PANEL LIVENESS CHANGED` five times over 2026-11-10…12, every one healthy hardware: `backlight_timeout` +
      `idle_brightness: 0` makes the idle-dim send `DISPOFF`, which clears RDDPM **bit 2**, so the poll read `0x98`
      against a `0x9c` baseline and announced a probable freeze (the "back to baseline" lines were the user touching the
      encoder). Proved 2026-11-12 with `SET_DISPLAY_BACKLIGHT ON=1`. Fix: the poll now judges the read against **what we
      COMMANDED**, masking only bit 2 (full sensitivity to booster / sleep-out / normal-mode kept, and a display-off we
      did NOT ask for still warns); the baseline is deferred while the display is commanded off, and the bit-2 mask
      closes **on observation** (a read that shows the commanded state), not a poll count — because the dim is queued at
      background priority and may not have executed yet. Hooked at `_set_backlight`, the single choke point for both
      opcodes; `DISPLAY_WEDGE_TEST` deliberately bypasses it so its `SLPIN` still reads as a fault. Gate **ARM N** pins
      it; warnings now decode the RDDPM bits and show expected-vs-read + commanded state. — 2026-11-12
- PuppyBus RS-485 master, used to *flash* the H503 enclosure MCU (at runtime the H503 is a
  second Klipper MCU on its own USB); TCA6408A I/O expander. — 2026-06

## Build / tooling
- `coreone/` reproducible build + flash tooling (SWD / DFU / BBF / RS-485); Kconfig/Makefile wiring. — 2026-06
- `coreone/build.sh` — **pin & verify the compiler; fix the hard-float guard's early-exit abort**
  (reproducible-build fixes). The script referenced a vendored `.dependencies/gcc-arm-none-eabi-13.3.1`
  the fork does not carry, so it **silently fell back** to the system `arm-none-eabi-gcc` — and the gcc
  version is a real build input (coexist is 59564 B under 13.3.1 vs 60244 B under 14.2.1). It now resolves
  the compiler explicitly, **aborts loudly** if it is not the pinned `14.2.1` (override with `WANT_GCC=`),
  and names the gcc in the build banner. Separately, the hard-float ordering guard piped `grep | head -1`;
  `head` closing the pipe gave `grep` SIGPIPE and, under `pipefail`+`set -e`, killed the script **141 before
  it verified FP ordering** (the guard's entire purpose) — now `sed -n '1p'`, which never closes the pipe
  early. Firmware-inert (build-script only); the published `v0.13.0-core-one.1` assets are the 14.2.1 build. — 2026-10-11
- `coreone/tools/fwdict.py` — new **firmware-provenance tool**: inflates Klipper's embedded data dictionary straight
  out of a raw `.bin` (it is zlib-compressed, so `strings` finds nothing) and prints the `version` string and the exact
  `build_versions` (gcc / binutils) a binary was built with — no surviving ELF, map or build log needed. Written after a
  week of "which compiler built this asset" questions; it reads the answer directly from the artifact. Also prints a
  dirty-build warning, since two dirty builds differ by a baked-in wall-clock timestamp and a hash difference between
  them looks exactly like a finding. Tools-only, touches no build or firmware. — 2026-11-12
- `coreone/swd-flash.sh` — new **`f427-katapult`** mode: SWD-write a Klipper app to `0x08008000` behind the
  Katapult bootloader (sectors 2-4 only; Katapult at `0x08000000` untouched). The SWD equivalent of
  `flash.sh f427-katapult-usb`, for a board reachable by probe but not by the katapult flashtool (e.g.
  flashing from the Pi, which has no katapult checkout). — 2026-09-28
- `coreone/katapult-preflight.sh` — new **bootloader-presence pre-flight** for the Katapult USB flash path,
  and it exists because the check that decides the flash variant was failing **unsafe**. Which bootloader sits
  at `0x08000000` is the single fact that decides whether a `0x08008000` image is correct or whether it erases
  an open bootloader — so the script looks rather than assumes. The prior logic ran the katapult flashtool under
  a bare `python3`; on a printer host the *system* python has no pyserial, so flashtool died, a `|| true`
  swallowed it, no Katapult device appeared, and the script concluded "this board has **no** Katapult → use the
  SWD/coexist path" — which is exactly what **erases the open bootloader**. A failure to *ask* the question had
  become a confident wrong "no." Now it resolves an interpreter that can `import serial` (klippy's venv;
  `$PYTHON` override), **checks the flashtool exit code instead of discarding it**, and returns a distinct
  **exit 4 = tooling failure, explicitly NOT a verdict about the board** (exit 1 stays a genuine no-Katapult).
  Prints the resolved device path on stdout, everything else on stderr. Protects anyone on the katapult/dev-flash
  flow from a bricked bootloader. Host/tooling only, firmware-inert, NO reflash. — 2026-10-18
- `coreone/docs/OWNERS_GUIDE.md` — new **machine-variant warnings** section (external-user safety, docs only).
  (a) **Belt pitch → XY scale:** Prusa ships 2GT (this port's `rotation_distance 32`) and 1.5GT (`31.5`, on Gen2 /
  INDX / kit-converts); a 1.5GT machine left at 32 prints **1.587 % oversized** in both axes (~3.2 mm on 200 mm),
  consistent and linear so it masquerades as shrinkage. (b) **Gen2 nozzle-cleaner pad:** ~0.5 mm Y clearance
  against the `-19` endstop during **Y homing with X parked over the pad (~166.5-196.5)**; our `G28` Y+15 jog
  protects the X sweep only by coincidence (it predates cleaners) and does nothing for the Y move — so Gen2 is
  **UNVALIDATED** and needs a slow by-hand homing check on a Gen2 base machine. `rotation_distance` stays 32
  (this is a Gen1/2GT port). — 2026-10-18
- `coreone/soft-abort.sh` — out-of-band **soft abort**: calls `soft_cancel.py`'s `soft_cancel/abort` webhook
  directly over klippy's UDS. It's the only way to interrupt a macro that holds the gcode mutex — the
  `SOFT_ABORT` gcode *command* itself queues behind the running macro and never fires. `soft_cancel` registered
  and documented that webhook but nothing in-tree ever called it, so the out-of-band path did not exist for
  any user. Trigger tested end-to-end (`{"status":"aborting"}`); the per-module bail-out (`_check_abort`) has
  not yet been fired in anger. — 2026-09-26
- `soft_cancel.py` — **LCD `SOFT_ABORT` routed out-of-band** (`_install_menu_hook`). An LCD menu `Soft Abort`
  item dispatches through `MenuManager.queue_gcode` → `run_script`, which acquires the gcode mutex — and the
  waits `soft_cancel` overrides (M190/M109/TEMPERATURE_WAIT/SOAK_WAIT) *hold* that mutex while polling, so the
  button would have queued behind the soak it was meant to interrupt (a fake e-stop, worse than none). The
  hook patches `queue_gcode` so a `SOFT_ABORT` script calls `_do_abort()` directly (mutex-free: sets the flag,
  drops heater targets, defers `CANCEL_PRINT` to a reactor callback); `SOFT_ABORT_RESET` keeps the normal path.
  ⚠ Confirmed to install (startup log) and the out-of-band path was validated via the webhook, but the menu
  item has **not** been pressed during a real heat soak — deployed, not proven. — 2026-10-09

## Deliberately NOT included
Printer *configuration* (`printer.cfg`, `boards/xbuddy.cfg`, `h5/extension.cfg`) — it carries real
device serials and lives host-side (`coreone-host`), so this repository stays secret-free.
