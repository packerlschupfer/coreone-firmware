#!/usr/bin/env python3
# Golden-compare gate for the MCU-side input-shaping evaluator (Phase A).
#
# Asserts that src/phase_shaper.c -- compiled as-shipped by harness.c -- reproduces the
# Python oracle in extras/phase_exec.py (`_shaped_motor` / `_eval_axis`, themselves the
# verbatim replica of chelper/kin_shaper.c) on the same raw axis trajectory.
#
# This is the Phase A validation gate. It is a pure numeric check: no printer, no MCU, no
# motion. It must pass before any of this touches hardware.
#
#   usage:  python3 golden.py [-v]
#
# What "agreement" means here. The MCU evaluates in float32; the oracle in float64. At print
# scale (a 250 mm axis at 200 step-units/mm = 50 000 su) one float32 ULP is already ~0.006 su,
# so an absolute 1e-4 su bound is arithmetically impossible and would be a fake gate. The
# tolerance is therefore ULP-scaled to the magnitude actually under test, and the report
# prints the observed error in ULP so a logic regression (which shows up as tens of thousands
# of ULP, or as an exact-zero-vs-nonzero mismatch) is unmistakable against float32 noise.
# This is the same precision the CURRENT motor-space path already runs at -- phase_seg.start_pos
# has always been a float32 in step-units -- so it is a status-quo bound, not a new cost.

import importlib.util
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# fork layout: coreone/test/shaper_golden/ -> fork root is three levels up; the oracle
# lives at klippy/extras/phase_exec.py (in-tree fork, not the klipper-port overlay).
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))   # fork root
PHASE_EXEC_PY = os.path.join(ROOT, 'klippy', 'extras', 'phase_exec.py')
HARNESS_C = os.path.join(HERE, 'harness.c')
HARNESS_BIN = os.path.join(HERE, 'harness')

CLOCK = 168e6          # F427 timer clock (ticks/s)
REFRESH_HZ = 8000.0    # validated phase-stepping default
TICK_STEP = int(round(CLOCK / REFRESH_HZ))

VERBOSE = '-v' in sys.argv[1:]

# absolute floor (dominates near the origin) and the ULP multiplier that sets the gate at
# print scale. 16 ULP leaves room for the ~5-impulse accumulation without hiding real bugs.
ABS_TOL = 1e-4
ABS_VTOL = 1e-3        # su/s; velocities are ~4 decades larger than positions
ULP_MULT = 8.0         # measured worst case across the vector set is ~3 ULP; 8 leaves headroom
                       # without letting a real logic error hide in float32 noise
FLOAT32_ULP = 2.0 ** -24


# ---------------------------------------------------------------- oracle import

def load_oracle_class():
    # Import the REAL extras/phase_exec.py (module-level imports are just math+logging, so
    # this works with no klippy on the path) and subclass it to skip the config-driven
    # __init__. We want the shipped _eval_axis/_shaped_motor, not a copy that can drift.
    spec = importlib.util.spec_from_file_location('pe_oracle', PHASE_EXEC_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    class Oracle(mod.PhaseExec):
        def __init__(self, px, py):
            self._px = px
            self._py = py

    return Oracle, mod.PhaseExec


# ---------------------------------------------------------------- shapers

def shaper_mzv(freq, zeta=0.1):
    # klippy/extras/shaper_defs.py get_mzv_shaper
    df = math.sqrt(1. - zeta ** 2)
    K = math.exp(-.75 * zeta * math.pi / df)
    t_d = 1. / (freq * df)
    a1 = 1. - 1. / math.sqrt(2.)
    a2 = (math.sqrt(2.) - 1.) * K
    a3 = a1 * K * K
    return ([a1, a2, a3], [0., .375 * t_d, .75 * t_d])


def shaper_ei(freq, zeta=0.1, v_tol=0.05):
    # klippy/extras/shaper_defs.py get_ei_shaper
    df = math.sqrt(1. - zeta ** 2)
    K = math.exp(-zeta * math.pi / df)
    t_d = 1. / (freq * df)
    a1 = .25 * (1. + v_tol)
    a2 = .5 * (1. - v_tol) * K
    a3 = a1 * K * K
    return ([a1, a2, a3], [0., .5 * t_d, t_d])


def quantize_pulses(pulses):
    # The MCU carries pulse offsets as INTEGER clock ticks (no per-tick seconds conversion).
    # Quantize the oracle's offsets identically, otherwise the two sides sample times up to
    # half a tick apart -- ~2e-4 su at 350 mm/s, i.e. above the gate, for no real reason.
    a, t = pulses
    dt = [int(round(ti * CLOCK)) for ti in t]
    return (list(a), [d / CLOCK for d in dt]), dt


# ---------------------------------------------------------------- trajectory model

class Move(object):
    # Mirrors the fields of chelper's `struct pull_move` that _eval_axis actually reads.
    __slots__ = ('print_time', 'move_t', 'start_v', 'accel',
                 'start_x', 'start_y', 'x_r', 'y_r')

    def __init__(self, print_time, move_t, start_v, accel, start_x, start_y, x_r, y_r):
        self.print_time = print_time
        self.move_t = move_t
        self.start_v = start_v
        self.accel = accel
        self.start_x = start_x
        self.start_y = start_y
        self.x_r = x_r
        self.y_r = y_r


def build_moves(spec, x0=0.0, y0=0.0, t0_ticks=0):
    # spec: [(duration_ticks, x_r, y_r, start_v, accel), ...]
    # Times are derived from INTEGER ticks so the oracle and the MCU see bit-identical move
    # boundaries -- the same telescoping-clock discipline _trapq_flush_cb uses.
    moves = []
    t = t0_ticks
    x, y = x0, y0
    for (dur, x_r, y_r, v0, a) in spec:
        pt = t / CLOCK
        mt = dur / CLOCK
        moves.append(Move(pt, mt, v0, a, x, y, x_r, y_r))
        d = v0 * mt + 0.5 * a * mt * mt
        x += x_r * d
        y += y_r * d
        t += dur
    return moves


def axis_segments(moves, reanchor=()):
    # Raw per-axis constant-accel segments, 1:1 with moves. Within a move both axes ARE
    # constant-accel (x(t) = start_x + x_r*(v0*t + a/2*t^2)), which is exactly why the host
    # can stop doing per-motor shaped fitting and just forward these (Phase B).
    #
    # `reanchor` is a set of move indices that carry the host's halt-resume flag in
    # duration's MSB. It must be invisible to the evaluator -- the oracle has no notion of
    # it, so any segment carrying it has to produce identical numbers.
    segs = ([], [])
    n = len(moves)
    for i, m in enumerate(moves):
        sc = int(round(m.print_time * CLOCK))
        dur = int(round((m.print_time + m.move_t) * CLOCK)) - sc
        ra = 1 if (i in reanchor or (i - n) in reanchor) else 0
        segs[0].append((sc, dur, m.start_x, m.x_r * m.start_v, 0.5 * m.x_r * m.accel, ra))
        segs[1].append((sc, dur, m.start_y, m.y_r * m.start_v, 0.5 * m.y_r * m.accel, ra))
    return segs


def emitter_segments(PhaseExecCls, moves, spm=1.0):
    # Drive the REAL Phase B host emitter (PhaseExec._raw_axis_segments) and hand its output
    # to the C evaluator. This closes the loop the synthetic path leaves open:
    #   real emitter -> real MCU evaluator -> Python oracle
    # so a bug anywhere in that chain shows up here, not on the printer. _raw_axis_segments
    # is a @staticmethod with the clock conversion injected precisely so this works with no
    # klippy, no MCU and no toolhead.
    t0 = moves[0].print_time
    t1 = moves[-1].print_time + moves[-1].move_t
    steps, _end_pt, _end_xy = PhaseExecCls._raw_axis_segments(
        moves, t0, t1, spm, lambda pt: int(round(pt * CLOCK)),
        (moves[0].start_x, moves[0].start_y))
    segs = ([], [])
    for (sc, dur, _ept, _exy, xt, yt, ra) in steps:
        segs[0].append((sc, dur, xt[0], xt[1], xt[2], ra))
        segs[1].append((sc, dur, yt[0], yt[1], yt[2], ra))
    return segs


def check_contiguity(segs):
    # STRUCTURAL invariant, not a numeric one: each segment's start_clock must be the
    # previous segment's EXACT end clock, and X and Y must share identical timing.
    #
    # Numeric comparison cannot see this -- a sub-tick boundary gap is 6 ns, far below the
    # float32 noise floor -- yet gaplessness is the documented precondition of pe_axis_eval's
    # forward cursor and the reason the host keeps a telescoping-clock discipline at all.
    # So assert it directly. (Verified to fire: injecting a 1-tick hole per segment, which
    # the numeric compare cannot see, is caught here.)
    bad = []
    for ax in (0, 1):
        for k in range(1, len(segs[ax])):
            psc, pdur = segs[ax][k - 1][0], segs[ax][k - 1][1]
            if segs[ax][k][0] != psc + pdur:
                bad.append('axis%d seg%d: start=%d expected=%d'
                           % (ax, k, segs[ax][k][0], psc + pdur))
    if len(segs[0]) != len(segs[1]):
        bad.append('X/Y segment count differs: %d vs %d' % (len(segs[0]), len(segs[1])))
    else:
        for k in range(len(segs[0])):
            if segs[0][k][:2] != segs[1][k][:2]:
                bad.append('axis timing desync at seg%d: %s vs %s'
                           % (k, segs[0][k][:2], segs[1][k][:2]))
    return bad


# ---------------------------------------------------------------- harness driver

def build_harness():
    cmd = ['gcc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-O2',
           '-o', HARNESS_BIN, HARNESS_C, '-lm']
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit('golden: harness build FAILED')


def run_harness(pulse_dt_x, pulse_ax, pulse_dt_y, pulse_ay,
                segs, ticks, proj, retire):
    lines = ['CLOCK %d' % int(CLOCK)]
    for ax, (a_list, dt_list) in ((0, (pulse_ax, pulse_dt_x)), (1, (pulse_ay, pulse_dt_y))):
        parts = ['PULSE %d %d' % (ax, len(a_list))]
        for a, dt in zip(a_list, dt_list):
            parts.append('%.17g %d' % (a, dt))
        lines.append(' '.join(parts))
    lines.append('PROJ %.17g %.17g %.17g %.17g %.17g %.17g' % proj)
    lines.append('RETIRE %d' % (1 if retire else 0))
    for ax in (0, 1):
        for (sc, dur, pos, v, ha, ra) in segs[ax]:
            lines.append('SEG %d %d %d %.17g %.17g %.17g %d'
                         % (ax, sc, dur, pos, v, ha, ra))
    for t in ticks:
        lines.append('TICK %d' % t)
    lines.append('END')
    r = subprocess.run([HARNESS_BIN], input='\n'.join(lines) + '\n',
                       capture_output=True, text=True)
    if r.returncode:
        sys.stderr.write(r.stdout + r.stderr)
        raise SystemExit('golden: harness run FAILED')
    out = []
    for ln in r.stdout.splitlines():
        f = ln.split()
        assert f[0] == 'T', ln
        out.append({'now': int(f[1]), 'xs': float(f[2]), 'ys': float(f[3]),
                    'vxs': float(f[4]), 'vys': float(f[5]),
                    'posA': float(f[6]), 'posB': float(f[7]),
                    'stx': int(f[8]), 'sty': int(f[9]),
                    'angA': int(f[12]), 'angB': int(f[13]),
                    'pa': int(f[14]), 'pb': int(f[15])})
    return out, r.stderr


# ---------------------------------------------------------------- vectors

def vec_cruise():
    # constant velocity, single long move. The shaper identity: input_shaper(v*T) == v*T.
    return build_moves([(int(0.25 * CLOCK), 1.0, 0.0, 40000.0, 0.0)])


def vec_ramp():
    # accel -> cruise -> decel on X (a plain trapezoid)
    ta, tc, td = int(.05 * CLOCK), int(.10 * CLOCK), int(.05 * CLOCK)
    a = 400000.0
    vmax = a * (ta / CLOCK)
    return build_moves([(ta, 1., 0., 0.0, a),
                        (tc, 1., 0., vmax, 0.0),
                        (td, 1., 0., vmax, -a)])


def vec_corner():
    # +X to a stop, then -X: a velocity zero-crossing, the reversal apex case that forced
    # the host-side peak-breakpoint insertion (_trapq_flush_cb).
    ta = int(.04 * CLOCK)
    a = 500000.0
    vmax = a * (ta / CLOCK)
    return build_moves([(ta, 1., 0., 0.0, a),
                        (ta, 1., 0., vmax, -a),
                        (ta, -1., 0., 0.0, a),
                        (ta, -1., 0., vmax, -a)])


def vec_diag():
    # simultaneous X and Y with DIFFERENT shapers -- the case that forces axis space.
    ta, tc = int(.05 * CLOCK), int(.12 * CLOCK)
    a = 300000.0
    vmax = a * (ta / CLOCK)
    r = 1.0 / math.sqrt(2.0)
    return build_moves([(ta, r, r, 0.0, a),
                        (tc, r, r, vmax, 0.0),
                        (ta, r, r, vmax, -a)])


def vec_dwell():
    # move, explicit zero-velocity HOLD (what _trapq_flush_cb emits for an idle span), move.
    ta = int(.04 * CLOCK)
    a = 300000.0
    vmax = a * (ta / CLOCK)
    return build_moves([(ta, 1., 0., 0.0, a),
                        (ta, 1., 0., vmax, -a),
                        (int(.15 * CLOCK), 1., 0., 0.0, 0.0),      # hold
                        (ta, 0., 1., 0.0, a),
                        (ta, 0., 1., vmax, -a)])


def vec_printscale():
    # same trapezoid, translated to print-scale absolute step-units (250 mm * 200 su/mm).
    # This is the float32-headroom probe, not a logic probe.
    ta, tc, td = int(.05 * CLOCK), int(.10 * CLOCK), int(.05 * CLOCK)
    a = 400000.0
    vmax = a * (ta / CLOCK)
    return build_moves([(ta, 1., 0., 0.0, a),
                        (tc, 1., 0., vmax, 0.0),
                        (td, 1., 0., vmax, -a)],
                       x0=50000.0, y0=42000.0)


def vec_gap():
    # A genuine HOLE in the move list -- move, nothing, move -- rather than vec_dwell's
    # explicit hold move. The oracle freezes across it (_eval_axis forces vel=0 in a gap);
    # the Phase B emitter has to fill it with a hold segment to keep the ring contiguous.
    # Both must land on the same numbers.
    ta = int(.04 * CLOCK)
    a = 300000.0
    vmax = a * (ta / CLOCK)
    m1 = build_moves([(ta, 1., 0., 0.0, a), (ta, 1., 0., vmax, -a)])
    last = m1[-1]
    d = last.start_v * last.move_t + .5 * last.accel * last.move_t ** 2
    m2 = build_moves([(ta, 0., 1., 0.0, a), (ta, 0., 1., vmax, -a)],
                     x0=last.start_x + last.x_r * d, y0=last.start_y,
                     t0_ticks=int(round((last.print_time + last.move_t) * CLOCK))
                     + int(.15 * CLOCK))
    return m1 + m2


def vec_longhold():
    # a multi-second hold segment: probes (float)duration precision in the frozen branch.
    return build_moves([(int(.02 * CLOCK), 1., 0., 0.0, 200000.0),
                        (int(3.0 * CLOCK), 1., 0., 0.0, 0.0),
                        (int(.02 * CLOCK), 1., 0., 0.0, 200000.0)])


VECTORS = [
    # (name, moves_fn, xs, ys, pre_ticks, post_ticks, jitter_ticks)
    ('cruise',      vec_cruise,     1.0,  1.0, 0, 0, 0),
    ('ramp',        vec_ramp,       1.0,  1.0, 0, 0, 0),
    ('corner',      vec_corner,     1.0,  1.0, 0, 0, 0),
    ('diag',        vec_diag,       1.0,  1.0, 0, 0, 0),
    ('dwell',       vec_dwell,      1.0,  1.0, 0, 0, 0),
    ('gap',         vec_gap,        1.0,  1.0, 0, 0, 0),
    ('printscale',  vec_printscale, 1.0,  1.0, 0, 0, 0),
    ('longhold',    vec_longhold,   1.0,  1.0, 0, 0, 0),
    ('ysign_flip',  vec_diag,       1.0, -1.0, 0, 0, 0),
    ('xsign_flip',  vec_diag,      -1.0,  1.0, 0, 0, 0),
    # edge windows: sample BEFORE the first segment and PAST the last one. Both sides must
    # agree on the static-hold / frozen-endpoint behaviour at the buffer edges.
    ('edges',       vec_ramp,       1.0,  1.0, 40, 40, 0),
    # Non-monotonic `now`: the evaluated time must be allowed to step BACKWARDS. To actually
    # reverse it the jitter has to exceed one refresh interval (21000 ticks @8 kHz), so 30000
    # is used -- a smaller value just makes the steps uneven and never exercises the backward
    # walk at all (an earlier 2000-tick version of this vector silently tested nothing).
    # Realistic triggers are a live SET_PHASE_LEAD change and a re-engage, not the per-motor
    # lead skew, which is microseconds and cannot reverse an 8 kHz tick on its own.
    ('jitter',      vec_diag,       1.0,  1.0, 0, 0, 30000),
    ('jitter_step', vec_corner,     1.0,  1.0, 0, 0, 30000),
]

# Vectors carrying the host's reanchor flag. The evaluator must mask it out of `duration`
# (PE_SEG_DUR); reading the raw field turns the duration into ~2^31 ticks and the
# frozen-endpoint branch stops firing, which only shows up PAST the last segment -- hence the
# trailing sample window.
REANCHOR_VECTORS = [
    ('reanchor',    vec_ramp,       1.0,  1.0, 0, 40, {0, -1}),
    ('reanchor_mid', vec_dwell,     1.0,  1.0, 0, 40, {2}),
]


# ---------------------------------------------------------------- compare

def tick_list(moves, pre, post, jitter=0):
    t0 = int(round(moves[0].print_time * CLOCK))
    t1 = int(round((moves[-1].print_time + moves[-1].move_t) * CLOCK))
    start = t0 - pre * TICK_STEP
    ticks = []
    t = start
    n = 0
    while t <= t1 + post * TICK_STEP:
        # `jitter` models the ONE way `now` can go backwards on the real machine: the axis
        # cursors are shared between the two motors while the ISR refreshes them round-robin
        # (one motor per tick), so if the motors are given different lead_ticks the evaluated
        # time oscillates by their difference. A forward-only cursor would over-advance and
        # silently evaluate a stale segment. This is what pe_axis_eval's bounded backward
        # walk exists for, and without this vector that code would ship untested.
        ticks.append(t - (jitter if (n & 1) else 0))
        t += TICK_STEP
        n += 1
    return ticks


def run_vector(Oracle, name, moves_fn, xs, ys, pre, post, shapers, jitter=0,
               reanchor=(), emitter=None, spm=1.0, expect=None):
    (px_q, px_dt), (py_q, py_dt) = shapers
    oracle = Oracle(px_q, py_q)
    moves = moves_fn()
    if emitter is not None:
        # spm != 1 on purpose: the emitter converts mm -> step-units, and with spm == 1
        # that conversion is invisible (an emitter that forgot it entirely would pass).
        # The oracle stays in mm, so its output is scaled by spm at the comparison.
        segs = emitter_segments(emitter, moves, spm)
        contig = check_contiguity(segs)
    else:
        segs = axis_segments(moves, reanchor)
        contig = []
        spm = 1.0
    ticks = tick_list(moves, pre, post, jitter)
    proj = (xs, ys, 0.0, xs, -ys, 0.0)

    got, diag = run_harness(px_dt, px_q[0], py_dt, py_q[0], segs, ticks, proj, retire=False)
    got_r, _ = run_harness(px_dt, px_q[0], py_dt, py_q[0], segs, ticks, proj, retire=True)

    # (1) retirement must not change a single evaluated value. This is the direct test of
    # correction #3 -- if the lookback window or pe_axis_retire is wrong, the retiring run
    # loses history the impulses still need and diverges here.
    # Assert the status bits / diagnostics a vector is SUPPOSED to exercise. Without this a
    # vector can quietly stop reaching the code it exists to cover -- which already happened
    # once here (the first `jitter` vector never actually reversed `now`).
    # RE-ANCHOR ANGLE INVARIANT. This is the check that was impossible before the projection
    # and reanchor arithmetic moved into phase_shaper.c: on the tick a motor re-anchors, the
    # COMMANDED ANGLE must not move. That is the entire point of a reanchor -- it re-seeds
    # phase_offset so the resumed position maps to the angle the commutation was already
    # driving. If it moves, a resume-after-halt steps the field, which at >1/2 electrical
    # period is an unrecoverable pole slip.
    reanchor_bad = []
    reanchor_seen = 0
    for k in range(1, len(got)):
        for tag, akey, pkey in (('A', 'angA', 'pa'), ('B', 'angB', 'pb')):
            if got[k][pkey] & 4:                       # PE_PROJ_REANCHORED
                reanchor_seen += 1
                if got[k][akey] != got[k - 1][akey]:
                    reanchor_bad.append(
                        'motor%s tick=%d angle %d -> %d (must hold)'
                        % (tag, got[k]['now'], got[k - 1][akey], got[k][akey]))

    missing = []
    if expect:
        if expect.get('reanchor'):
            if not any((g['stx'] | g['sty']) & 16 for g in got):
                missing.append('PE_EVAL_REANCHOR never asserted')
            if not reanchor_seen:
                missing.append('PE_PROJ_REANCHORED never consumed')
            elif reanchor_seen > 10:
                # One consumption per motor per flagged segment. A large count means the
                # latch is LEVEL- not EDGE-triggered: PE_EVAL_REANCHOR stays asserted for
                # ~140 ticks, so a level trigger re-seeds phase_offset every tick and pins
                # the commanded angle -- which the angle-hold invariant alone cannot see,
                # because a pinned angle trivially satisfies "did not move".
                missing.append('PE_PROJ_REANCHORED consumed %d times (level, not edge)'
                               % reanchor_seen)
        if expect.get('backscan'):
            tot = 0
            for ln in diag.splitlines():
                for f in ln.split():
                    if f.startswith('backscan='):
                        tot += int(f.split('=')[1])
            if tot == 0:
                missing.append('backward walk never taken')

    retire_bad = 0
    for g, r in zip(got, got_r):
        if g['posA'] != r['posA'] or g['posB'] != r['posB']:
            retire_bad += 1

    max_err = 0.0
    max_verr = 0.0
    max_mag = 0.0
    max_vmag = 0.0
    worst = None
    for g in got:
        T = g['now'] / CLOCK
        (Ap, Av, _Aa), (Bp, Bv, _Ba) = oracle._shaped_motor(moves, T, xs, ys)
        Ap *= spm; Bp *= spm; Av *= spm; Bv *= spm
        eA = abs(g['posA'] - Ap)
        eB = abs(g['posB'] - Bp)
        vA = xs * g['vxs'] + ys * g['vys']
        vB = xs * g['vxs'] - ys * g['vys']
        max_verr = max(max_verr, abs(vA - Av), abs(vB - Bv))
        max_mag = max(max_mag, abs(Ap), abs(Bp))
        max_vmag = max(max_vmag, abs(Av), abs(Bv))
        e = max(eA, eB)
        if e > max_err:
            max_err = e
            worst = (g['now'], g['posA'], Ap, g['posB'], Bp)

    tol = max(ABS_TOL, ULP_MULT * FLOAT32_ULP * max_mag)
    # Velocity is asserted too, NOT just position: it drives the fwd/bwd cogging blend
    # (pe_update_angle), so a wrong derivative shows up as a torque transient at reversals
    # rather than as a position error. An early mutation run passed on position alone with a
    # 1e4 su/s velocity error -- hence this.
    vtol = max(ABS_VTOL, ULP_MULT * FLOAT32_ULP * max_vmag)
    ulps = (max_err / (FLOAT32_ULP * max_mag)) if max_mag > 0 else 0.0
    vulps = (max_verr / (FLOAT32_ULP * max_vmag)) if max_vmag > 0 else 0.0
    ok = ((max_err <= tol) and (max_verr <= vtol) and (retire_bad == 0)
          and not contig and not missing and not reanchor_bad)

    print('%-12s %5d ticks  pos=%.3e (%.1f ULP, tol %.2e)  vel=%.3e (%.1f ULP, tol %.2e)  '
          'retire_mismatch=%d  contig=%s  %s'
          % (name, len(ticks), max_err, ulps, tol, max_verr, vulps, vtol, retire_bad,
             'ok' if not contig else 'BROKEN', 'PASS' if ok else 'FAIL'))
    if contig:
        for b in contig[:4]:
            print('    contiguity: %s' % b)
    for b in reanchor_bad[:4]:
        print('    REANCHOR: %s' % b)
    for b in missing:
        print('    COVERAGE: %s' % b)
    if VERBOSE or not ok:
        print('    %s' % diag.strip().replace('\n', '\n    '))
        if worst:
            print('    worst tick=%d  A: c=%.6f py=%.6f   B: c=%.6f py=%.6f'
                  % worst)
    return ok


def run_identity(Oracle):
    # The un-shaped path must survive: with the identity pulse ([1.0],[0.0]) the evaluator
    # has to reduce EXACTLY to plain segment evaluation. Phase A keeps this path live for the
    # extruder, PHASE_SEG_TEST, the oscillator and the cogging sweeps.
    ident = (([1.0], [0.0]), [0])
    ok = run_vector(Oracle, 'identity', vec_ramp, 1.0, 1.0, 0, 0, (ident, ident))
    return ok


def main():
    if not os.path.exists(PHASE_EXEC_PY):
        raise SystemExit('golden: cannot find %s' % PHASE_EXEC_PY)
    build_harness()
    Oracle, PhaseExecCls = load_oracle_class()

    # The LIVE machine's shapers, read from the SAVE_CONFIG block on 2026-07-30:
    #   shaper_type_x = mzv, shaper_freq_x = 63.6
    #   shaper_type_y = mzv, shaper_freq_y = 49.4
    # NOT ei@55.4 / mzv@43.4 -- those are the commented ILLUSTRATIVE values at the top of
    # [input_shaper], which printer.cfg itself warns are an old example. Everything shaped is
    # driven by the live pulses at runtime, so this only affects which case the gate exercises
    # -- but it should exercise the one the machine actually runs.
    px_raw = shaper_mzv(63.6)
    py_raw = shaper_mzv(49.4)
    px_q, px_dt = quantize_pulses(Oracle._init_shaper_pulses(*px_raw))
    py_q, py_dt = quantize_pulses(Oracle._init_shaper_pulses(*py_raw))

    print('X pulses (MZV @63.6): a=%s  dt_ms=%s'
          % (['%.5f' % a for a in px_q[0]], ['%+.3f' % (d / CLOCK * 1e3) for d in px_dt]))
    print('Y pulses (MZV @49.4): a=%s  dt_ms=%s'
          % (['%.5f' % a for a in py_q[0]], ['%+.3f' % (d / CLOCK * 1e3) for d in py_dt]))
    lookback = -min(min(px_dt), min(py_dt)) / CLOCK * 1e3
    lookahead = max(max(px_dt), max(py_dt)) / CLOCK * 1e3
    print('lookback=%.2f ms  lookahead=%.2f ms  (offsets straddle zero -- correction #3)\n'
          % (lookback, lookahead))

    shapers = ((px_q, px_dt), (py_q, py_dt))
    ok = True
    print('--- evaluator (synthetic 1:1 segments) ---')
    for (name, fn, xs, ys, pre, post, jit) in VECTORS:
        ok &= run_vector(Oracle, name, fn, xs, ys, pre, post, shapers, jit,
                         expect={'backscan': True} if jit else None)
    for (name, fn, xs, ys, pre, post, ra) in REANCHOR_VECTORS:
        ok &= run_vector(Oracle, name, fn, xs, ys, pre, post, shapers, 0, ra,
                         expect={'reanchor': True})
    ok &= run_identity(Oracle)

    # Phase B: the same vectors, but the axis segments now come from the REAL host emitter
    # instead of golden.py's synthetic 1:1 conversion. This is what validates the emitter,
    # at spm=200 so the mm -> step-unit conversion is actually exercised.
    print('\n--- Phase B host emitter (PhaseExec._raw_axis_segments) ---')
    for (name, fn, xs, ys, pre, post, jit) in VECTORS:
        ok &= run_vector(Oracle, 'e:' + name, fn, xs, ys, pre, post, shapers, jit,
                         emitter=PhaseExecCls, spm=200.0)

    print('\nGOLDEN GATE: %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
