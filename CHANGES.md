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
    open-loop XY position — which is exactly why the disengage banner **demands** a G28 — but the restore branch's
    `th.set_position(th.get_position())` re-bases position without homing or clearing the homed flag, so `homed_axes`
    still read `'xy'` afterwards. Nothing enforced the required re-home: a macro gating on `homed_axes` got a
    **stale-true**. Fix: one guarded line right after the re-base — `th.get_kinematics().clear_homing_state("xy")`
    (the canonical clear path, same call `force_move`/`safe_z_home`/`stepper_enable` use; drops `homed_axes` to `''`
    or `'z'`). `"xy"` only — disengage perturbs only the CoreXY motors, Z is untouched, so no needless Z re-home; and
    **unconditional** — the re-base runs regardless of `net`, so a net-conditional clear would re-introduce the
    stale-true in the `net==0` case (exactly the clean-run signature). No behaviour change beyond the honest flag;
    all 13 gates green. ⚠ Confirming `homed_axes` actually drops wants a supervised engage→disengage run (operator's
    lane), same as the engage-path change above. — 2026-10-18
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
