"""Gate: a PAUSED print must not be silently replaced by a new file load.

    python3 sdcard_paused_guard_test.py

🚨 THE BUG THIS EXISTS FOR. Upstream `virtual_sdcard.py` guards both file-load commands with
`if self.work_timer is not None: raise "SD busy"`. That is true only while a print is ACTIVELY
RUNNING. `do_pause()` -> `work_handler`'s exit block sets `work_timer = None`, so a PAUSED job
passed the guard, and the `_reset_file()` on the next line discarded it. A long print parked
mid-M600 could be destroyed by one console `SDCARD_PRINT_FILE`, with no error raised and nothing
in the log naming what was lost.

On THIS machine that is not an edge case: both filament sensors drive `_FILAMENT_RUNOUT -> M600`,
so "paused" is the routine interrupted state. Found by `coreone-moonraker` on a rig; confirmed in
the deployed source by `coreone-fw`, including that our `PAUSE` macro reaches stock `do_pause()`
via `rename_existing: PAUSE_BASE`, so the Core One macro does not change the exposure.

🔑 ALLOWLIST, NOT DENYLIST. The guard tests `state in ('standby','complete','cancelled')` rather
than `state not in ('printing','paused')`. An allowlist fails CLOSED on a state nobody enumerated
-- `error`, or anything a future Klipper adds -- where a denylist fails OPEN. The pattern came from
Viola's `druck` via `coreone-slicer`; `coreone-moonraker` shipped the same form on the Moonraker
side in `f3fe0a6`, so both halves now agree on one predicate rather than two that can drift.

⚠ WHY THE PREDICATE IS SAFE DESPITE LOOKING CIRCULAR. `print_stats.state` is driven partly BY
`virtual_sdcard`, so reading it from inside `virtual_sdcard` invites a race. There is none:
`work_handler`'s exit block sets `work_timer = None` two lines before calling
`print_stats.note_pause()`, with no reactor yield between them, so no other greenlet can observe
the gap. ARM D pins that ordering -- if a future refactor inserts a yield there, the guard develops
a hole in exactly the case it exists for, and this gate is the only thing that would notice.
"""
import sys, os, re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import golden

EXTRAS = os.path.dirname(golden.PHASE_EXEC_PY)
OVERLAY_ROOT = os.path.dirname(EXTRAS)          # klipper-port/ in the overlay
PATCH = os.path.join(OVERLAY_ROOT, 'sdcard-paused-guard.patch')
APPLY = os.path.join(OVERLAY_ROOT, 'apply-overlay.sh')
CFG = os.path.join(OVERLAY_ROOT, 'printer.cfg')

fails = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        fails.append(msg)


def arm(title, fn):
    print("\n" + title)
    try:
        fn()
    except Exception as e:
        import traceback
        check(False, "ARM DID NOT RUN: %s: %s" % (type(e).__name__, e))
        traceback.print_exc()


def _need_shipped():
    """Clean early-out so a tree with no guard FAILS READABLY rather than crashing.

    An unguarded tree correctly exits non-zero either way, but the first version
    raised TypeError from `None` in three arms and buried the one message that
    says what is actually wrong. "Gates must FAIL, not crash" -- a crash reads as
    a broken harness rather than a real finding.
    """
    if SHIPPED is None:
        check(False, "this tree ships no guard (mode=%s) — nothing to assert against" % MODE)
        return False
    return True


def _shipped_source():
    """The virtual_sdcard.py THIS TREE SHIPS, plus which tree it is.

    🚨 WHY THIS EXISTS -- THIRD INSTANCE OF ONE MISTAKE. The first version of this
    gate read `dirname(EXTRAS)/sdcard-paused-guard.patch` and applied it to
    `~/git/klipper`. That is the OVERLAY's shape: there the shipped artifact IS
    upstream-plus-patch. **In the FORK there is no patch at all** -- the fork IS the
    applied state, its gates live at `coreone/test/shaper_golden/`, and
    `klippy/extras/virtual_sdcard.py` already contains the guard. So the old gate
    would have gone RED in a clean fork clone on three separate arms, while passing
    here only because `~/git/klipper` and the overlay happen to coexist on this box
    -- i.e. green for reasons unrelated to what it claimed to test. Found by
    `coreone-release` reading it against a real fork tree.

    This is the same class as `display_health_test._find_src` and
    `keepalive_margin_test._find_shaper_h`, both of which exist because I made this
    mistake before and wrote the fix down. Assuming the overlay layout is the bug;
    resolve what the tree actually ships instead.

    ⚠ The mode is DECLARED and an unrecognised tree FAILS. A gate that silently
    skipped its arms in an unknown layout would be the silent-skip hazard this
    project hard-fails on everywhere else.
    """
    applied = os.path.join(EXTRAS, 'virtual_sdcard.py')
    if os.path.exists(applied):
        src = open(applied).read()
        if 'PRINT_START_STATES' in src:
            return src, 'fork'          # the tree ships the guard directly
    if os.path.exists(PATCH):
        import subprocess, shutil, tempfile
        up = os.path.expanduser('~/git/klipper/klippy/extras/virtual_sdcard.py')
        if os.path.exists(up):
            d = tempfile.mkdtemp()
            try:
                os.makedirs(os.path.join(d, 'klippy', 'extras'))
                dst = os.path.join(d, 'klippy', 'extras', 'virtual_sdcard.py')
                shutil.copy2(up, dst)
                if subprocess.run(['patch', '-s', '-p1', '-i', PATCH],
                                  cwd=d, capture_output=True).returncode == 0:
                    return open(dst).read(), 'overlay'
            finally:
                shutil.rmtree(d, ignore_errors=True)
    return None, None


SHIPPED, MODE = _shipped_source()
SRC = open(PATCH).read() if os.path.exists(PATCH) else None
ADDED = "\n".join(l[1:] for l in (SRC or "").split("\n")
                  if l.startswith("+") and not l.startswith("+++"))

def arm_a():
    """The guard reaches the machine by whatever route THIS tree uses."""
    check(SHIPPED is not None,
          "the tree's shipped virtual_sdcard.py could be resolved (mode=%s) — an"
          " unrecognised layout FAILS here rather than skipping silently" % MODE)
    if MODE == 'overlay':
        ap = open(APPLY).read() if os.path.exists(APPLY) else ""
        check("sdcard-paused-guard.patch" in ap, "  overlay: apply-overlay.sh applies the patch")
        m = re.search(r"REQUIRED_PATCHES=\((.*?)\)", ap, re.S)
        check(bool(m) and "sdcard-paused-guard.patch" in m.group(1),
              "  overlay: and it is in REQUIRED_PATCHES, so a missing patch HARD-FAILS"
              " rather than building a klippy without the guard")
    elif MODE == 'fork':
        check("sdcard-paused-guard.patch" not in os.listdir(OVERLAY_ROOT)
              if os.path.isdir(OVERLAY_ROOT) else True,
              "  fork: no patch file is expected — the fork IS the applied state")


def arm_b():
    """It is an ALLOWLIST. This is the arm that fails if someone 'simplifies' it."""
    if not _need_shipped():
        return
    check("PRINT_START_STATES" in SHIPPED, "a named state allowlist is present in the shipped file")
    m = re.search(r"PRINT_START_STATES\s*=\s*\(([^)]*)\)", SHIPPED)
    states = set(re.findall(r"'([a-z]+)'", m.group(1))) if m else set()
    check(states == {"standby", "complete", "cancelled"},
          "the allowlist is exactly standby/complete/cancelled, got %s" % sorted(states))
    check("not in PRINT_START_STATES" in SHIPPED,
          "the test is membership of the ALLOWLIST (fails closed on an unknown state)")
    check("state in ('printing'" not in SHIPPED,
          "and NOT a denylist naming printing/paused — a denylist permits every state"
          " nobody thought of, including `error` and anything Klipper adds later")


def arm_c():
    """Both load paths are guarded, not just the one that was demonstrated."""
    if not _need_shipped():
        return
    if SHIPPED is None:
        return
    check("def _check_can_load" in SHIPPED, "a shared guard helper exists after patching")
    check(SHIPPED.count("self._check_can_load(gcmd)") == 2,
          "exactly two callers — cmd_SDCARD_PRINT_FILE and cmd_M23")
    for fn in ("cmd_SDCARD_PRINT_FILE", "cmd_M23"):
        body = SHIPPED[SHIPPED.index("def " + fn):]
        body = body[:body.index("\n    def ", 1)]
        check("_check_can_load" in body, "  %s goes through the guard" % fn)
        check("work_timer is not None" not in body,
              "  %s no longer has its own work_timer-only check — two guards where one"
              " is wrong is how the wrong one gets kept" % fn)
    guard = SHIPPED[SHIPPED.index("def _check_can_load"):]
    guard = guard[:guard.index("\n    def ", 1)]
    check("SD busy" in guard,
          "the actively-printing case still raises 'SD busy' (Moonraker relays it as HTTP 400,"
          " and existing callers key on that string)")


def arm_d():
    """The ordering the predicate depends on, pinned against a future refactor."""
    if SHIPPED is None:
        check(False, "no shipped source to pin the ordering against")
        return
    # ⚠ Read the TREE'S OWN file, not ~/git/klipper. The first version reached for
    # the upstream checkout, so in a fork tree it passed only because that path
    # happened to exist on one developer's box -- the same assume-the-overlay bug
    # this gate's resolver exists to avoid, surviving in a single arm. Caught by
    # actually running the gate in a fork-shaped tree instead of reasoning about it.
    # work_handler is untouched by the patch, so SHIPPED carries it in either mode.
    tail = SHIPPED[SHIPPED.index("logging.info(\"Exiting SD card print"):]
    i_timer = tail.index("self.work_timer = None")
    i_pause = tail.index("note_pause()")
    check(i_timer < i_pause,
          "work_timer is cleared BEFORE note_pause() in work_handler's exit block")
    between = tail[i_timer:i_pause]
    check("reactor.pause" not in between and "yield" not in between,
          "and nothing yields between them, so no greenlet can see work_timer=None"
          " while state is still 'printing' — this is why reading print_stats from"
          " inside virtual_sdcard is not a race")


def arm_e():
    """The refusal must tell the operator how to proceed, or it is a dead end."""
    if not _need_shipped():
        return
    check("SDCARD_RESET_FILE" in SHIPPED,
          "the refusal names SDCARD_RESET_FILE as the way out")
    check("%s" in SHIPPED and "state" in SHIPPED,
          "and reports WHICH state blocked it, rather than a bare refusal")
    check("CANCEL_PRINT" in SHIPPED,
          "CANCEL_PRINT is offered too — the operator who meant to abandon the paused"
          " job needs a path that is not 'reset the sdcard'")


def arm_f():
    """🔑 The interaction with OUR power-loss recovery, which no upstream test covers.

    RESUME_PRINT_GO does M23 -> M26 -> M24. `coreone-moonraker` measured on a rig that the
    POWER-CUT path is unaffected, because klippy restarts to `standby` and M23 is allowed.
    But PLR is gated on `plr_active`, NOT on print state, so it can also be reached from a
    print that ended in `error` and was never reset — and there the guard refuses M23. That
    is correct by design and a behaviour change for the macro, so the macro resets first.
    """
    if MODE != 'overlay':
        print("  --   SKIPPED: printer.cfg is not a fork artifact (mode=%s)." % MODE)
        print("       The PLR interaction is an OVERLAY concern and is gated there;"
              " the fork ships no printer config at all.")
        return
    check(os.path.exists(CFG), "overlay printer.cfg is present")
    if not os.path.exists(CFG):
        return
    cfg = open(CFG).read()
    i = cfg.find("M23 {file}")
    check(i > 0, "RESUME_PRINT_GO still loads with M23 {file}")
    if i < 0:
        return
    before = cfg[max(0, i - 700):i]
    check("SDCARD_RESET_FILE" in before,
          "and SDCARD_RESET_FILE precedes it, so recovery works from ANY prior state —"
          " without this, PLR from a print that ended in `error` is refused by the guard")
    check("M26 S" in cfg[i:i + 200] and "M24" in cfg[i:i + 300],
          "   the M23 -> M26 -> M24 sequence is otherwise unchanged")
    check("SDCARD_RESET_FILE" in SHIPPED,
          "and the guard's own refusal names SDCARD_RESET_FILE, for the operator who hits"
          " it outside PLR")


arm("ARM A -- the patch ships and cannot be silently skipped:", arm_a)
arm("ARM B -- ALLOWLIST, not a denylist (fails closed on unknown states):", arm_b)
arm("ARM C -- both load paths guarded, old guard removed:", arm_c)
arm("ARM D -- the work_timer/note_pause ordering the predicate rests on:", arm_d)
arm("ARM E -- the refusal is actionable, not a dead end:", arm_e)
arm("ARM F -- PLR resets the sdcard before M23, so recovery survives the guard:", arm_f)

print()
if fails:
    print("SDCARD-PAUSED-GUARD GATE: FAIL -- %d check(s) failed" % len(fails))
    for f in fails:
        print("   - " + f)
    sys.exit(1)
print("SDCARD-PAUSED-GUARD GATE: PASS -- a paused job cannot be silently replaced by"
      " SDCARD_PRINT_FILE or M23; the guard is an allowlist that fails closed on unknown"
      " states, both load paths go through it, the ordering its predicate depends on is"
      " pinned, and the refusal names a way out.")
