# Bound klippy's Python GC pause so a collection cannot shut the MCU down, and MEASURE
# whether the host ever stops for long enough to do it.
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
# WHY THE WATCHDOG (2026-10-14)
# -----------------------------
# The freeze measurement above proves the collection got cheap AT READY. It does not prove the
# host never stopped DURING a print, and neither does a clean print: a 4 h 49 m job on
# 2026-10-13 finished with no shutdown, but klippy's own Stats lines are 1 s apart and so
# cannot resolve the ~270 ms hole that killed the other two. "The failure did not happen" is
# weaker evidence than "the host never stopped for more than N ms", and only the second one
# tells you how much margin is left.
#
# So: a reactor timer that measures ITS OWN firing lateness. It asks the reactor to wake it at
# a known instant and records how late the wake actually was. Anything that stops the single
# klippy thread -- a GC pass, a blocking syscall, a long callback, the OS descheduling the
# process -- shows up as lateness on the very next fire, whatever its phase, because the stall
# necessarily lands inside some scheduled interval. One timer callback and a few float ops per
# second; it cannot plausibly be the thing it is measuring.
#
# ⚠ THE WATCHDOG IS DELIBERATELY INDEPENDENT OF `enabled`. It measures; the freeze is the
# treatment. Keeping them separate makes `enabled: False` a usable A/B arm (watchdog on, freeze
# off, see the stalls come back) instead of a blind disable -- see
# memory: defences_must_be_tested_independently.
#
# ⚠ IT IS NOT GC-SPECIFIC, AND THE REPORT MUST NOT CLAIM IT IS. Lateness names a host stall; it
# does not attribute one. Don't read a number here as a GC measurement -- GC_TUNE_STATUS times
# an actual collection for that.
#
#   [gc_tune]
#   #enabled: True
#   #measure: True
#   #watchdog: True
#   #watchdog_period: 1.0
#   #refreeze: True          # keep the level-2 cost flat as the print runs (see below)
#   #refreeze_poll: 2.0      # how often to NOTICE a pass, not how often to freeze
#
# GC_TUNE_STATUS reports the freeze count, the watchdog's worst lateness, the re-freeze
# counters, and times a level-2 pass on demand. GC_TUNE_STATUS RESET=1 zeroes the watchdog
# stats, so PRINT_START can reset and PRINT_END can report a per-print figure.
#
# GC_RECLAIM releases the permanent generation, collects it, and re-freezes the live graph --
# the only way to give back the cyclic garbage that periodic re-freezing makes uncollectable.
# It REFUSES while printing, because its cost is a full level-2 pass, i.e. the exact stall
# this module exists to prevent.

import gc, logging, time


class GCTune:
    # A level-2 pass at or above this is worth flagging. Well under the 258/269 ms that
    # actually killed prints, because the margin to a past-dated heater clock is the heater's
    # own PWM_DELAY, not the 269 ms we happened to measure.
    WARN_MS = 100.0

    # Lateness buckets, ms. Reported as counts so the shape is visible: a host that is late
    # 10 ms a thousand times is a different animal from one that is late 250 ms once, and a
    # mean would hide both. (memory: truncated_denominator_confound -- an average hides shape.)
    BUCKETS = (10.0, 25.0, 50.0, 100.0, 250.0)

    def __init__(self, config):
        self.printer = config.get_printer()
        self.enabled = config.getboolean('enabled', True)
        self.measure = config.getboolean('measure', True)
        self.watchdog = config.getboolean('watchdog', True)
        # Period floor 0.1 s: below that the callback rate starts to matter against the thing
        # being measured, and the resolution gained is pointless -- a stall is detected on the
        # next fire regardless of period, the period only bounds how soon you LEARN of it.
        self.wd_period = config.getfloat('watchdog_period', 1.0, minval=0.1, maxval=60.)
        self.refreeze = config.getboolean('refreeze', True)
        # Poll period for noticing klippy's own level-2 passes. This is NOT how often we
        # freeze -- we freeze only when a pass actually happened, which on this machine is
        # minutes apart. It bounds how much post-pass allocation gets swept into the freeze
        # (see _do_refreeze), so shorter is cleaner but costs a tuple read per tick.
        self.rf_period = config.getfloat('refreeze_poll', 2.0, minval=0.5, maxval=60.)
        self._stats_at = None            # eventtime of the last gc.get_stats() read
        self._stats_cache = ([], 0)      # (per-gen collections, total uncollectable)
        self._rf_timer = None
        self._l2_seen = None             # last reactor level-2 timestamp we acted on
        self._rf_n = 0
        self._rf_frozen_added = 0
        self._rf_last_ms = 0.
        self._rf_unavailable = False
        self.last_report = "not run yet"
        self._wd_timer = None
        self._wd_due = None
        self._wd_reset()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('GC_TUNE_STATUS', self.cmd_GC_TUNE_STATUS,
                               desc=self.cmd_GC_TUNE_STATUS_help)
        gcode.register_command('GC_RECLAIM', self.cmd_GC_RECLAIM,
                               desc=self.cmd_GC_RECLAIM_help)

    def _timed_collect(self, gen=2):
        t0 = time.monotonic()
        freed = gc.collect(gen)
        return (time.monotonic() - t0) * 1e3, freed

    # ------------------------------------------------------------------ reactor watchdog
    # gc.get_stats() CACHE. get_status is HOT -- Moonraker queries it several times a second --
    # and get_stats() BUILDS THREE DICTS per call. Calling it unthrottled from get_status would
    # add allocation pressure to the very hot path whose allocation this module exists to
    # measure, which would be a self-defeating instrument. gc.get_count() is a plain tuple read
    # and stays uncached. The collection counters move on the scale of seconds at most, so 1 s
    # of resolution loses nothing.
    _STATS_TTL = 1.0

    def _gc_stats_cached(self, eventtime):
        if self._stats_at is None or eventtime - self._stats_at >= self._STATS_TTL:
            try:
                st = gc.get_stats()
                self._stats_cache = ([int(g.get('collections', 0)) for g in st],
                                     sum(int(g.get('uncollectable', 0)) for g in st))
            except Exception:
                # Never let instrumentation break a status query.
                self._stats_cache = ([], 0)
            self._stats_at = eventtime
        return self._stats_cache

    # ------------------------------------------------------------------ periodic re-freeze
    # WHY THIS EXISTS (2026-10-31)
    # ---------------------------
    # The at-ready freeze below fixes the STARTUP graph and nothing else. Everything allocated
    # after ready stays under the collector forever, so the level-2 pass cost CLIMBS BACK as a
    # print runs. Measured offline on a growing long-lived graph: 1.60 -> 2.48 -> 4.99 -> 8.60
    # -> 13.13 -> 16.18 ms over six rounds with no re-freeze, versus a FLAT 1.0-1.4 ms with one.
    # That climb is the open n=3/n=4 of 2026-10-31: a 188 ms stall killed a print at 52 min, and
    # 188 ms is already lethal, so the earlier ">250 ms" note was optimistic.
    #
    # WHY PIGGYBACK ON KLIPPY'S OWN PASS, AND NEVER COLLECT OURSELVES
    # --------------------------------------------------------------
    # gc.freeze() moves whatever is currently in the three generations into the permanent
    # generation. Freezing an uncollected heap would make its cyclic garbage permanently
    # uncollectable, so a freeze wants to happen just after a collection -- but calling
    # collect(2) ourselves mid-print IS the 258-269 ms stall we are trying to prevent.
    # reactor.py already collects by hand (klippy.py:349 does gc.disable(); _check_gc escalates
    # to level 2 only when it believes the reactor is idle) AND it records the eventtime of each
    # level in _last_gc_times, exposed as reactor.get_gc_stats(). So we watch that tuple and
    # freeze right after a level-2 pass we did not pay for. We inherit klippy's own judgement
    # about when stopping is survivable.
    #
    # MEASURED PROPERTIES THAT MAKE THIS SAFE
    #   * gc.freeze() is O(1): 0.002 ms at 50k objects, 0.004 ms at 800k. It is a generation
    #     splice, not a traversal. It cannot be the stall it prevents.
    #   * A freeze RESETS gc.get_count() to (0,0,0), so _check_gc's gi[1]>=10 / gi[2]>=10
    #     escalation restarts from scratch -- level-2 passes get RARER as a side effect.
    #
    # ⚠ THE LEAK, STATED PRECISELY. It is NOT "frozen-then-dead objects" in general: ordinary
    # dead objects are freed by REFCOUNTING whether frozen or not (measured: 202.4 -> 11.6 MB
    # after dropping refs to a frozen heap). The leak is specifically frozen-then-dead
    # REFERENCE CYCLES, which are unreachable AND uncollectable (measured: 175 MB held,
    # collect(2) freed 0). The reclaim path is unfreeze + collect(2) -- measured 900k objects
    # back in 125.7 ms -- and that cost is exactly why it may only run where no MCU timer
    # depends on the next few hundred ms. Hence: reclaim at ready, or on explicit command.
    # NEVER from a timer. See memory: defences_must_be_tested_independently.
    def _do_refreeze(self):
        before = gc.get_freeze_count()
        t0 = time.monotonic()
        gc.freeze()
        self._rf_last_ms = (time.monotonic() - t0) * 1e3
        added = gc.get_freeze_count() - before
        self._rf_n += 1
        if added > 0:
            self._rf_frozen_added += added
        # Log the first one at info so the log says the mechanism engaged, then stay quiet:
        # these recur for the whole life of the process and must not become a log flood.
        if self._rf_n == 1:
            logging.info("gc_tune: re-freeze ENGAGED after klippy's first level-2 pass"
                         " (+%d objects now permanent, freeze took %.3f ms). The level-2 pass"
                         " cost should now stay flat instead of growing with the print.",
                         added, self._rf_last_ms)

    def _refreeze_event(self, eventtime):
        reactor = self.printer.get_reactor()
        try:
            l2 = reactor.get_gc_stats()[2]
        except Exception:
            # No get_gc_stats on this klipper. Say so ONCE and stop polling -- a silent no-op
            # would look identical to "working", which is how a disabled defence hides.
            if not self._rf_unavailable:
                self._rf_unavailable = True
                logging.warning("gc_tune: reactor has no get_gc_stats(); periodic re-freeze"
                                " CANNOT run on this klipper, so the level-2 pass cost will"
                                " grow with the print as before. Only the at-ready freeze is"
                                " active.")
            return reactor.NEVER
        if self._l2_seen is None:
            # First look: adopt the current value. The at-ready freeze has already happened,
            # so there is nothing to do for a pass that predates us.
            self._l2_seen = l2
        elif l2 != self._l2_seen:
            self._l2_seen = l2
            self._do_refreeze()
        # Re-arm from NOW, not from `eventtime`, for the same reason the watchdog does: a stall
        # would otherwise leave a backlog of missed deadlines and fire a burst of catch-up polls
        # onto a host that just proved it is struggling. This poll has nothing to catch up ON --
        # it compares a timestamp, so a skipped tick loses nothing.
        return reactor.monotonic() + self.rf_period

    def _arm_refreeze(self):
        reactor = self.printer.get_reactor()
        if self._rf_timer is None:
            self._rf_timer = reactor.register_timer(self._refreeze_event)
        self._l2_seen = None
        reactor.update_timer(self._rf_timer, reactor.monotonic() + self.rf_period)

    def _reclaim(self):
        """unfreeze -> collect(2) -> freeze. The ONLY path that gives back leaked cycles.

        Costs a full level-2 traversal (125.7 ms for 900k objects, measured), i.e. it is the
        very stall this module exists to prevent. Callers MUST be somewhere that a few hundred
        ms cannot post-date an MCU timer -- klippy:ready, or an operator command while idle.
        """
        pre = gc.get_freeze_count()
        if pre:
            gc.unfreeze()
        ms, freed = self._timed_collect(2)
        gc.freeze()
        return pre, gc.get_freeze_count(), ms, freed

    def _rf_report(self):
        if not self.refreeze:
            return "  periodic re-freeze: OFF (refreeze: False) -- level-2 cost will GROW"
        if self._rf_unavailable:
            return ("  periodic re-freeze: UNAVAILABLE (no reactor.get_gc_stats) -- level-2"
                    " cost will GROW")
        if not self._rf_n:
            return ("  periodic re-freeze: armed, no level-2 pass seen yet (normal -- they are"
                    " minutes apart, and a freeze resets the counters that trigger them)")
        return ("  periodic re-freeze: %d pass(es) followed, %d objects made permanent,"
                " last freeze %.3f ms"
                % (self._rf_n, self._rf_frozen_added, self._rf_last_ms))

    def _wd_reset(self):
        self._wd_max = 0.0
        self._wd_max_ctx = ""
        self._wd_sum = 0.0
        self._wd_n = 0
        self._wd_over = [0] * len(self.BUCKETS)
        self._wd_warned = False
        try:
            self._wd_since = self.printer.get_reactor().monotonic()
        except Exception:
            self._wd_since = None

    def _wd_context(self):
        """Describe WHERE a new worst-case landed. Called only when the max moves, so the
        lookups cannot add load in the common case.

        ⚠ file_position is read ONLY when print_stats says printing. The Stats line's sd_pos
        RETAINS ITS LAST VALUE when no print is running, and trusting it cost a whole
        investigation on 2026-10-14 (three print_stall increments attributed to the wrong
        file at the wrong offsets). An unqualified position is worse than none.
        """
        bits = []
        try:
            # PLAIN ATTRIBUTES ONLY, deliberately. print_stats.get_status() calls
            # _update_filament_usage(), i.e. it MUTATES another module's accounting, and
            # toolhead.get_status() rebuilds the kinematics dict. A diagnostic has no
            # business doing either. NEVER get_last_move_time() here -- that FLUSHES the
            # toolhead, which from a 1 s timer would itself stall the reactor.
            state = None
            ps = self.printer.lookup_object('print_stats', None)
            if ps is not None:
                state = getattr(ps, 'state', None)
                bits.append("print=%s" % (state,))
            if state == 'printing':
                vs = self.printer.lookup_object('virtual_sdcard', None)
                if vs is not None:
                    bits.append("sd_pos=%s" % (getattr(vs, 'file_position', '?'),))
            th = self.printer.lookup_object('toolhead', None)
            if th is not None:
                bits.append("print_time=%.3f" % (getattr(th, 'print_time', -1.) or -1.,))
        except Exception:
            bits.append("context unavailable")
        return " ".join(bits)

    def _watchdog_event(self, eventtime):
        reactor = self.printer.get_reactor()
        now = reactor.monotonic()
        due = self._wd_due
        if due is not None:
            # Lateness is measured against the instant WE asked for, not against the previous
            # fire, so a late wake cannot silently reset the yardstick. Clamp at 0: an early
            # wake is a reactor rounding artefact, not negative lateness.
            late_ms = (now - due) * 1e3
            if late_ms < 0.:
                late_ms = 0.
            self._wd_n += 1
            self._wd_sum += late_ms
            for i, b in enumerate(self.BUCKETS):
                if late_ms >= b:
                    self._wd_over[i] += 1
            if late_ms > self._wd_max:
                self._wd_max = late_ms
                self._wd_max_ctx = self._wd_context()
                # Log the first crossing only. A stall storm would otherwise flood the log
                # with the very writes that make the host slow.
                if late_ms >= self.WARN_MS and not self._wd_warned:
                    self._wd_warned = True
                    logging.warning(
                        "gc_tune: THE HOST STOPPED FOR %.1f ms (reactor fired that late"
                        " at %s). >~250 ms has twice post-dated a heater PWM clock and shut"
                        " the MCU down with \"Timer too close\". This says the host stalled,"
                        " NOT that GC did it -- run GC_TUNE_STATUS while idle to time an"
                        " actual collection.", late_ms, self._wd_max_ctx)
        # Re-arm relative to NOW, not to `due`: re-arming from a missed deadline would queue a
        # burst of catch-up fires right after a stall, i.e. pile work onto a host that just
        # proved it is struggling.
        self._wd_due = now + self.wd_period
        return self._wd_due

    def _arm_watchdog(self):
        reactor = self.printer.get_reactor()
        if self._wd_timer is None:
            self._wd_timer = reactor.register_timer(self._watchdog_event)
        self._wd_reset()
        self._wd_due = reactor.monotonic() + self.wd_period
        reactor.update_timer(self._wd_timer, self._wd_due)

    def _wd_report(self):
        if not self.watchdog:
            return "  reactor watchdog: OFF (watchdog: False) -- no host-stall measurement"
        if not self._wd_n:
            return "  reactor watchdog: armed, no samples yet"
        span = 0.
        try:
            if self._wd_since is not None:
                span = self.printer.get_reactor().monotonic() - self._wd_since
        except Exception:
            pass
        buckets = " ".join("%.0fms:%d" % (b, c)
                           for b, c in zip(self.BUCKETS, self._wd_over) if c)
        return ("  reactor watchdog: worst %.1f ms late over %d fires / %.0f s"
                " (mean %.2f ms)\n"
                "    late >= %s\n"
                "    worst seen at: %s"
                % (self._wd_max, self._wd_n, span, self._wd_sum / self._wd_n,
                   buckets if buckets else "10ms: never",
                   self._wd_max_ctx if self._wd_max_ctx else "n/a"))

    # ------------------------------------------------------------------ freeze at ready
    def _handle_ready(self):
        # The watchdog is armed FIRST and regardless of `enabled`, so the disabled arm is
        # still instrumented. Arming before the freeze also means the freeze's own collection
        # is not charged to the watchdog as lateness.
        if self.watchdog:
            self._arm_watchdog()
        if not self.enabled:
            logging.info("gc_tune: freeze disabled by config; klippy keeps the stock GC"
                         " behaviour (a level-2 pass measured 258-269 ms on this machine and"
                         " shut the MCU down twice -- see the module header).%s"
                         % (" The reactor watchdog is still armed, so this is a measured A/B"
                            " arm rather than a blind disable." if self.watchdog else ""))
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
        # Re-arm so the ready-time collection above is not the watchdog's first sample.
        if self.watchdog:
            self._arm_watchdog()
        # Armed LAST, and only when the freeze itself is enabled: re-freezing while the
        # at-ready freeze is off would be an unfrozen startup graph plus a growing permanent
        # generation, i.e. the leak with none of the benefit.
        if self.refreeze:
            self._arm_refreeze()

    def get_status(self, eventtime):
        # Exposed so Moonraker can graph it and a macro can read a per-print figure without
        # parsing console text. late_max_ms is the headline: how long the host stopped.
        collections, uncollectable = self._gc_stats_cached(eventtime)
        return {
            'enabled': 1 if self.enabled else 0,
            'frozen': gc.get_freeze_count() if hasattr(gc, 'get_freeze_count') else -1,
            'watchdog': 1 if self.watchdog else 0,
            'late_max_ms': round(self._wd_max, 3),
            'late_mean_ms': round(self._wd_sum / self._wd_n, 4) if self._wd_n else 0.,
            'late_fires': self._wd_n,
            'late_over_100ms': self._wd_over[3] if len(self._wd_over) > 3 else 0,
            'late_over_250ms': self._wd_over[4] if len(self._wd_over) > 4 else 0,
            'late_max_context': self._wd_max_ctx,
            # ALLOCATION PRESSURE, readable mid-print without triggering anything. Neither
            # gc.get_count() nor gc.get_stats() traverses or collects, which is the whole point:
            # the pass COST can only be measured by running a pass (see GC_TUNE_STATUS
            # COLLECT=1), and doing that during a print would make the instrument the fault.
            # These two are the risk-free half.
            #   gc_counts  ⚠ ONLY [0] is an object count -- allocations minus deallocations
            #              since the last gen-0 collection. [1] and [2] are COLLECTION
            #              counters: [1] = gen-0 collections since the last gen-1, [2] =
            #              gen-1 collections since the last gen-2. (Verified empirically;
            #              an earlier revision of this comment called all three "pressure per
            #              generation", which is wrong and would mislead anyone graphing [2].)
            #              [2] is still the useful one -- it is exactly what reactor._check_gc
            #              tests against 10 to decide to escalate to level 2 -- but read it as
            #              "distance to the next level-2 pass", NOT as a heap or an object count.
            #   gc_collections  cumulative passes per generation, from gc.get_stats().
            #              gc_collections[2] answers "has a level-2 pass even run yet", which
            #              lateness cannot. ⚠ gc is DISABLED in klippy (klippy.py:349), so every
            #              pass is a MANUAL gc.collect() from reactor._check_gc -- there are no
            #              automatic passes at all, and gc_automatic below is 0 by design.
            #   gc_uncollectable  total objects GC gave up on. A LEAK CANARY -- it must stay flat.
            'gc_counts': list(gc.get_count()),
            'gc_collections': collections,
            'gc_uncollectable': uncollectable,
            'gc_automatic': 1 if gc.isenabled() else 0,
            # Re-freeze telemetry. refreezes climbing while late_max_ms stays flat is the
            # mechanism working. frozen_added is the LEAK BOUND: every one of these objects is
            # permanently uncollectable if it later becomes cyclic garbage, so this is the
            # number to watch against klippy RSS, not a success metric.
            'refreeze': 1 if self.refreeze else 0,
            'refreeze_available': 0 if self._rf_unavailable else 1,
            'refreezes': self._rf_n,
            'refreeze_frozen_added': self._rf_frozen_added,
            'refreeze_last_ms': round(self._rf_last_ms, 4),
        }

    cmd_GC_RECLAIM_help = ("Release the permanent generation, collect it, and re-freeze the "
                           "live graph. Gives back cyclic garbage that periodic re-freezing "
                           "made uncollectable. REFUSES while printing -- it costs a full "
                           "level-2 pass (~100-270 ms), which is the stall this module exists "
                           "to prevent.")

    def cmd_GC_RECLAIM(self, gcmd):
        # HARD REFUSAL, not a warning. This command's whole cost IS the documented failure
        # mechanism: a few hundred ms of stopped host post-dates a heater's next soft-PWM
        # clock and src/sched.c fires try_shutdown("Timer too close"). Letting an operator
        # fire it mid-print would hand them the exact bug. Checked as a plain attribute --
        # print_stats.get_status() mutates filament accounting (see _wd_context).
        ps = self.printer.lookup_object('print_stats', None)
        state = getattr(ps, 'state', None) if ps is not None else None
        if state == 'printing' and not gcmd.get_int('FORCE', 0):
            raise gcmd.error(
                "GC_RECLAIM refused: a print is running. This runs a full level-2 collection"
                " (~100-270 ms of stopped host), and that is exactly what post-dated a heater"
                " PWM clock and shut the MCU down with \"Timer too close\" on 2026-09-12 and"
                " 2026-10-12. Run it when idle. FORCE=1 overrides if you know the heaters are"
                " off.")
        if not (hasattr(gc, 'freeze') and hasattr(gc, 'unfreeze')):
            raise gcmd.error("GC_RECLAIM: this Python has no gc.freeze/gc.unfreeze")
        pre, post, ms, freed = self._reclaim()
        self._rf_frozen_added = 0
        gcmd.respond_info(
            "gc_tune: reclaim done -- frozen %d -> %d, level-2 pass %.1f ms, freed %d\n"
            "  the re-freeze leak bound is now reset; refreezes so far: %d"
            % (pre, post, ms, freed, self._rf_n))

    cmd_GC_TUNE_STATUS_help = ("Report the GC freeze count and the worst reactor stall, and "
                               "time a level-2 collection. COLLECT=0 reports without the "
                               "collection (free, safe mid-print); RESET=1 zeroes the stall "
                               "stats (also free).")

    def cmd_GC_TUNE_STATUS(self, gcmd):
        # RESET is handled before the collection so PRINT_START can zero the stats without
        # paying for -- or risking -- a level-2 pass at the top of a print.
        if gcmd.get_int('RESET', 0):
            self._wd_reset()
            gcmd.respond_info("gc_tune: reactor stall stats reset (frozen=%d)"
                              % (gc.get_freeze_count()
                                 if hasattr(gc, 'get_freeze_count') else -1,))
            return
        counts = gc.get_count()
        frozen = gc.get_freeze_count() if hasattr(gc, 'get_freeze_count') else -1
        # COLLECT=0 EXISTS SO PRINT_END CAN REPORT SAFELY. The watchdog figure is the whole
        # point of calling this from a macro, and timing a collection to get it would be
        # self-defeating: with the freeze disabled (or a future regression that un-freezes)
        # the pass is the 258-269 ms stall itself, and PRINT_END still has live heaters whose
        # next soft-PWM update would be post-dated -- the exact mechanism in the header. A
        # diagnostic must not be able to cause the fault it reports on.
        if not gcmd.get_int('COLLECT', 1):
            gcmd.respond_info(
                "gc_tune: enabled=%d frozen=%d counts=%s automatic=%d\n"
                "  level-2 pass: not timed (COLLECT=0)\n"
                "  at ready: %s\n%s\n%s"
                % (self.enabled, frozen, counts, gc.isenabled(),
                   self.last_report, self._wd_report(), self._rf_report()))
            return
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
            "  at ready: %s\n%s\n%s\n%s"
            % (self.enabled, frozen, counts, gc.isenabled(), now_ms, freed,
               self.last_report, self._wd_report(), self._rf_report(), verdict))


def load_config(config):
    return GCTune(config)
