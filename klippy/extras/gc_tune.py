# Bound klippy's Python GC pause so a collection cannot shut the MCU down.
#
# WHY THIS EXISTS
# ---------------
# Twice (2026-09-12 and 2026-10-12) the F427 shut down with "Timer too close" mid-print while
# every host-load metric looked perfect. Both times the cause was the same, and it was not the
# serial link, not CPU, not thermals, not phase_exec:
#
#   1. klippy.py calls gc.disable() and builds Reactor(gc_checking=True), so klippy collects by
#      HAND from reactor.py _check_gc -- only when the reactor believes it is idle, escalating
#      to generation 2 once the gen1 and gen2 counters have both crossed 10. A level-2 pass
#      walks every tracked object in the process.
#   2. That pass is uninterruptible and single-threaded. Measured from the shutdown dumps:
#      268.7 ms (10-12) and 258 ms (09-12), each sitting inside a ~360 ms hole in which klippy
#      sent nothing at all and read nothing off the USB link.
#   3. On resume klippy flushes its backlog, and a heater's soft-PWM update goes out with a
#      clock that is now in the PAST -- 33.8 ms (10-12) and 17.4 ms (09-12), both on the same
#      oid=46.
#   4. src/gpiocmds.c command_queue_digital_out stores that clock straight into d->timer.waketime
#      and calls sched_add_timer with no past-clock check, so src/sched.c:94 fires
#      try_shutdown("Timer too close"). Under a 40 kHz ISR the head of the timer list is never
#      more than 25 us out, so a past clock trips it with certainty.
#
# Klipper's canned "host computer is overloaded" text is right in kind and wrong in detail: the
# host is not CPU-bound, it is stopped dead, and no 1-second sysload sample can see it.
#
# WHY gc.freeze() IS THE RIGHT LEVER
# ----------------------------------
# The two events bracket an 8x spread in session size (09-12: 1.22 GB written, send_seq 31.0M;
# 10-12: 148 MB, send_seq 3.34M) and produced near-identical 258/269 ms collections. The cost
# is therefore dominated by the long-lived startup object graph, NOT by accumulated runtime
# garbage -- which also means "restart klippy between long prints" is nearly useless as a
# mitigation (10-12's process was only 22.1 h old). gc.freeze() moves exactly that startup
# graph into a permanent generation the collector never walks again, so what is left to walk is
# only what was allocated after ready.
#
# WHY unfreeze() FIRST
# -------------------
# klippy re-creates the Printer object inside the SAME process on a restart, so klippy:ready
# fires again and the previous graph becomes garbage. Freezing without releasing the old one
# would leak an entire printer object graph per restart, forever. gc.unfreeze() puts the
# permanent generation back under the collector, the explicit collect reclaims it while nothing
# is printing, and only then do we freeze the live graph.
#
# The pass is self-measuring: it times a level-2 collection before and after the freeze and logs
# both, so the log says what the fix bought on this machine instead of asking anyone to take it
# on faith. Set measure: False to skip the extra pre-freeze collection.
#
#   [gc_tune]
#   #enabled: True
#   #measure: True
#
# GC_TUNE_STATUS reports the current freeze count and times a level-2 pass on demand.

import gc, logging, time


class GCTune:
    # A level-2 pass at or above this is worth flagging. Well under the 258/269 ms that
    # actually killed prints, because the margin to a past-dated heater clock is the heater's
    # own PWM_DELAY, not the 269 ms we happened to measure.
    WARN_MS = 100.0

    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = config.getboolean('enabled', True)
        self.measure = config.getboolean('measure', True)
        self.last_report = "not run yet"
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('GC_TUNE_STATUS', self.cmd_GC_TUNE_STATUS,
                               desc=self.cmd_GC_TUNE_STATUS_help)

    def _timed_collect(self, gen=2):
        t0 = time.monotonic()
        freed = gc.collect(gen)
        return (time.monotonic() - t0) * 1e3, freed

    def _handle_ready(self):
        if not self.enabled:
            logging.info("gc_tune: disabled by config; klippy keeps the stock GC behaviour"
                         " (a level-2 pass measured 258-269 ms on this machine and shut the"
                         " MCU down twice -- see the module header)")
            return
        if not (hasattr(gc, 'freeze') and hasattr(gc, 'unfreeze')):
            logging.warning("gc_tune: this Python has no gc.freeze/gc.unfreeze; nothing done")
            return
        # Release any graph frozen by a previous klippy:ready in this same process, then
        # reclaim it. Doing this at ready is deliberate: it is the one moment we can afford
        # to pay for a full collection, because no print is running and no MCU timer depends
        # on the next few hundred ms.
        pre_frozen = gc.get_freeze_count()
        if pre_frozen:
            gc.unfreeze()
        before_ms = freed = None
        if self.measure:
            before_ms, freed = self._timed_collect(2)
        else:
            gc.collect(2)
        gc.freeze()
        frozen = gc.get_freeze_count()
        after_ms, _ = self._timed_collect(2)
        if before_ms is None:
            self.last_report = ("frozen=%d (was %d); level-2 pass after freeze %.1f ms"
                                % (frozen, pre_frozen, after_ms))
        else:
            self.last_report = (
                "frozen=%d (was %d, released %d objects); level-2 pass %.1f ms -> %.1f ms"
                % (frozen, pre_frozen, freed, before_ms, after_ms))
        logging.info("gc_tune: %s", self.last_report)

    cmd_GC_TUNE_STATUS_help = ("Report the GC freeze count and time a level-2 collection. "
                              "Safe while idle; it costs one collection, so do not run it "
                              "during a print.")

    def cmd_GC_TUNE_STATUS(self, gcmd):
        counts = gc.get_count()
        frozen = gc.get_freeze_count() if hasattr(gc, 'get_freeze_count') else -1
        now_ms, freed = self._timed_collect(2)
        # Say what THIS number means, not what some number might mean. A flat warning reads as
        # an alarm when the pass is 0.4 ms, and a diagnostic that cries wolf in the normal case
        # is how lateBEF=11631 scared everyone about a part that was fine.
        if now_ms >= self.WARN_MS:
            verdict = ("  ^^ %.0f ms stalls the reactor for that long. >~250 ms has twice\n"
                       "  post-dated a heater PWM clock and shut the MCU down with\n"
                       "  \"Timer too close\" (2026-09-12, 2026-10-12)." % now_ms)
        else:
            verdict = ("  within budget: the reactor cannot stall long enough on this to\n"
                       "  post-date an MCU timer (the two shutdowns took 258 and 269 ms).")
        gcmd.respond_info(
            "gc_tune: enabled=%d frozen=%d counts=%s automatic=%d\n"
            "  level-2 pass now: %.1f ms (freed %d)\n"
            "  at ready: %s\n%s"
            % (self.enabled, frozen, counts, gc.isenabled(), now_ms, freed,
               self.last_report, verdict))


def load_config(config):
    return GCTune(config)
