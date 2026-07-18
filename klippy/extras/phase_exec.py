# Host glue for the phase_exec MCU module (Phase 1a high-rate XDIRECT executor).
#
# Config:  [phase_exec stepper_y]      (one per axis you want to drive)
#            stepper: stepper_y         # optional; defaults to the section name
#
# It reuses the TMC2130's OWN spidev (tmc.mcu_tmc.tmc_spi.spi) so there's no
# second CS claim ("pin used multiple times"). The MCU timer then streams the
# rotating coil-current vector to XDIRECT at RATE Hz — 100x+ the host's ~80 Hz
# ceiling (Phase 0). Constant-velocity sweep for now.
#
# CONTRACT before PHASE_EXEC_START (Phase 0 lessons — do these on the host first):
#   SET_STEPPER_ENABLE STEPPER=<s> ENABLE=1     ; driver power stage on
#   SET_TMC_CURRENT STEPPER=<s> HOLDCURRENT=..  ; IHOLD scales the XDIRECT current
#   SET_TMC_FIELD STEPPER=<s> FIELD=direct_mode VALUE=1
# and AFTER: PHASE_EXEC_STOP, then SET_TMC_FIELD ... direct_mode VALUE=0, re-home.
# (Don't run normal moves on that axis while it's in direct mode.)
#
# NOTE: untested until the phase_exec MCU firmware is flashed — verify the
# interval/clock math and command encoding on first boot.

import math, logging

PHASE_UNITS = 1024
PE_MAX_HARM = 4          # cogging correction harmonics (must match src/phase_exec.c)


class PhaseExec:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.stepper_name = config.get('stepper', self.name)
        self.use_dma = config.getboolean('dma', False)
        self.oid = self.dma_oid = self.mcu = self.spi_oid = None
        self.mcu_stepper = None
        self.cmd_queue = None
        self.start_cmd = self.stop_cmd = self.traj_cmd = self.query_cmd = None
        self.corr_cmd = self.sweep_cmd = self.osc_cmd = self.blend_cmd = None
        self.analytic_cmd = self.seg_cmd = self.seg_query_cmd = None
        self.lead_cmd = None
        self._interval = 0           # cached refresh interval (ticks) from the last engage
        self._lead_frac = 1.0        # velocity-lead as a fraction of interval (1.0 = Prusa)
        self._slave_lead_extra_us = 0.0  # DMA write-skew comp: extra lead on the CoreXY slave
                                         # (its XDIRECT is written ~1 SPI frame after the master's).
                                         # Tunable via SET_PHASE_LEAD SLAVE_EXTRA_US=. Default 0.
        self.spi_div_cmd = self.cs_slack_cmd = self.pin_hs_cmd = None
        # live-trapq streaming state
        self._trapq_engaged = False
        self._trapq = self._ffi_main = self._ffi_lib = None
        self._steps_per_mm = self._trapq_sign = 1.0
        self._trapq_pos_ref = None
        self._trapq_last_pt = 0.
        self._trapq_debug = False
        self._trapq_net = 0.0
        self._trapq_nseg = 0
        # last commanded motor-frame position (mm, sent-frame) at the flush horizon, so an
        # idle flush can feed a zero-velocity HOLD segment instead of leaving the MCU to
        # COAST on stale end-velocity (Fix 2) -> the per-layer Y staircase. None until first seg.
        self._trapq_last_posA = None
        self._trapq_last_posB = None
        # set True when the move queue drains to a HALT (flush_cb sees no moves); the next
        # resume segment then carries reanchor=1 so the MCU re-seeds the phase origin from
        # the held phase (Prusa reset_from_halt). Kills the per-layer-change Y staircase.
        self._reanchor_next = False
        # BACKPRESSURE: host mirror of sent-but-unplayed segment END clocks. When
        # this reaches _max_inflight we STOP streaming and defer the rest of the window to the
        # next flush -> the MCU ring (SEG_RING=1024) can NEVER overflow -> no silent segment
        # DROP -> no field teleport. Prusa's pending_targets backpressures the same way.
        self._inflight = []          # chronological list of segment end-clocks (64-bit)
        # ISOLATION: the backpressure defer DISABLED (huge cap = never defer). The 1024 ring showed
        # overflow=0 without backpressure, so its protection is unneeded here AND its
        # stale-cache-on-defer is a teleport suspect. Restore to 768 only if a run shows overflow>0.
        self._max_inflight = config.getint('max_inflight', 768, minval=16)  # backpressure ON (1024 ring)
        # Reused trapq-extraction buffer + a generous cap so a dense/fast flush window
        # never drops moves (which teleported the field). Allocated lazily (needs _ffi_main).
        self._extract_buf = None
        self._extract_cap = 2048
        # HOST TELEPORT LOGGER: track the last segment's END pos (sent step-units) per motor;
        # if the next segment's START jumps >1/4 elec period (16 su) we log it with context.
        # Pinpoints WHERE the host builds the differential -Y teleport (the field-teleport tripwire's MCU-side twin).
        self._last_end_posA = None
        self._last_end_posB = None
        self._slew = 24.0            # max chain->shaped correction per segment (su); <32 (1/2 elec
                                     # period) so it can't slip a pole, >per-seg drift so it converges
        # T2: max chain-vs-shaped divergence per print (su). If ~0 the chain tracks the intended
        # trajectory (offset is physical); if mm-scale the chain drifts (offset is COMMANDED).
        self._maxdivA = 0.0
        self._maxdivB = 0.0
        # per-segment displacement error (velfit_disp - shaped_disp, su) — should be ~0 for
        # single-interval exact segments. Track the WORST + its params to find the error source.
        self._maxsegerr = 0.0
        self._worst = (0, 0.0, 0.0)   # (span intervals, dt, vel su/s)
        # CoreXY dual-motor: slave '-' motor (x-y) streamed alongside this '+'
        self._trapq_slave_oid = None
        self._trapq_pos_ref_b = None
        self._trapq_sign_b = 1.0
        self._trapq_ysign = 1.0   # flip the Y term of the CoreXY projection (A<->B y)
        self._trapq_xsign = 1.0   # flip the X term of the CoreXY projection (reverses X
                                  # only; a motor-dir flip would reverse BOTH axes since
                                  # X=(A+B)/2 and Y=(A-B)/2 share the same belt travels)
        # input shaping: per-axis normalized+shifted convolution pulses (a[], t[]).
        # Trivial ([1],[0]) = identity (no shaping); set from [input_shaper] at engage.
        self._px = ([1.0], [0.0])
        self._py = ([1.0], [0.0])
        self._pulse_margin = 0.0   # stream-horizon lag = max positive pulse time
        self._shape = False
        self._trapq_merge_tol = 0.001  # segment-merge position budget (mm); see _flush_cb
        # merge_max raised 1->8 (2026-07-03): the comment's own remedy for "dry high" (we hit
        # dry=13047 on fast dense infill @250mm/s). Enables tol-merging -- straight solid infill
        # fuses to one segment with ZERO error (within tol) -- plus the min_seg_dt force-merge.
        self._merge_max = config.getint('merge_max', 8, minval=1)
        # MIN-DURATION RATE CAP (Prusa #3, in firmware): force-merge breakpoints until each segment
        # is >= min_seg_dt, so the segment RATE caps at 1/min_seg_dt REGARDLESS of print speed ->
        # the host can never be outrun by fast dense geometry. Deviation = the curve sagitta over
        # the merge span (~1um on a 1mm-radius hole at 250mm/s / 0.8ms) = negligible; segERR tracks
        # it. Set 0 to disable (pure tol-merge only).
        self._min_seg_dt = config.getfloat('min_seg_dt', 0.0008, minval=0.)
        self._suspended_bulk = []
        # step-gen suppression: X/Y steppers whose trapq we detached to stop wasted step generation
        # while phase-driven; list of (mcu_stepper, saved_trapq) to restore on disengage.
        self._suppressed_trapq = []
        self._group_slave = None
        # Per-direction cogging-correction spectra (mirror Prusa's fwd/bwd current LUTs):
        #   self.corrections = forward {h:(mag,pha)};  self.corr_bwd = backward {h:(mag,pha)}.
        # `harmonicN_mag/_pha` set forward; `harmonicN_mag_bwd/_pha_bwd` set backward and
        # DEFAULT to the forward value (so an un-split config = symmetric = old behaviour).
        # Both re-sent each run (the MCU struct is volatile across a reset).
        self.corrections = {}
        self.corr_bwd = {}
        for h in range(1, PE_MAX_HARM + 1):
            mag = config.getint('harmonic%d_mag' % h, 0)
            pha = config.getint('harmonic%d_pha' % h, 0) % PHASE_UNITS
            if mag:
                self.corrections[h] = (mag, pha)
            magb = config.getint('harmonic%d_mag_bwd' % h, mag)
            phab = config.getint('harmonic%d_pha_bwd' % h, pha) % PHASE_UNITS
            if magb:
                self.corr_bwd[h] = (magb, phab)
        # fwd<->bwd cogging-blend velocity band (mm/s). The MCU weights between the fwd and
        # bwd LUTs linearly across +/-band around zero velocity so the correction is
        # continuous through a reversal (no hard switch -> no torque transient / chatter at
        # the marginal loaded reversal where the bidirectional cogging drift came from).
        # Below the band the correction settles to the symmetric average (the stable state).
        # 0 = legacy hard switch. No-op for symmetric (fwd==bwd) LUTs regardless of band.
        self.blend_vband_mmps = config.getfloat('blend_vband_mmps', 10., minval=0.)
        # idle/hold current: at sustained standstill the MCU ramps the XDIRECT amplitude down to
        # idle_hold_pct% of full, cutting standstill heat (~I^2). Fixes phase-step holding FULL
        # current continuously (direct_mode disables the TMC standstill drop) -> cooked motors ->
        # hot StallGuard homing failures. Amplitude-only -> position-safe. 100 (or 0) = disabled.
        self.idle_hold_pct = config.getint('idle_hold_pct', 100, minval=0, maxval=100)
        self.idle_ms = config.getfloat('idle_ms', 100., minval=1.)
        # SET_PHASE_COGGING ENABLE=0 stashes the live spectra here and zeroes the MCU LUTs
        # for a clean cogging-OFF A/B (localizes the residual pad drift vs the position path
        # / physical slip). ENABLE=1 restores. None = not currently suppressed.
        self._cogging_stash = None
        self.gcode = gcode = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._mcu_identify)
        # Self-heal: if a run crashes (MCU shutdown) PHASE_EXEC_STOP never fires, so
        # the host TMC no-op monkeypatches would persist (they survive a
        # firmware_restart) and break sensorless homing. Restore them on shutdown.
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        gcode.register_mux_command('PHASE_EXEC_START', 'STEPPER',
                                   self.stepper_name, self.cmd_START,
                                   desc="Start the high-rate XDIRECT sweep")
        gcode.register_mux_command('PHASE_EXEC_STOP', 'STEPPER',
                                   self.stepper_name, self.cmd_STOP,
                                   desc="Stop the XDIRECT sweep")
        gcode.register_mux_command('PHASE_EXEC_TRAJECTORY', 'STEPPER',
                                   self.stepper_name, self.cmd_TRAJECTORY,
                                   desc="Drive angle from the live trajectory (ENABLE=0/1)")
        gcode.register_mux_command('PHASE_DMA_STATS', 'STEPPER',
                                   self.stepper_name, self.cmd_STATS,
                                   desc="Report DMA diagnostics (run after STOP)")
        gcode.register_mux_command('SET_PHASE_CORRECTION', 'STEPPER',
                                   self.stepper_name, self.cmd_SET_CORRECTION,
                                   desc="Set a cogging-correction harmonic (MAG, PHA)")
        gcode.register_mux_command('SET_PHASE_BLEND', 'STEPPER',
                                   self.stepper_name, self.cmd_SET_BLEND,
                                   desc="Set the fwd<->bwd cogging-blend band (VBAND mm/s)")
        gcode.register_mux_command('SET_PHASE_COGGING', 'STEPPER',
                                   self.stepper_name, self.cmd_SET_COGGING,
                                   desc="Toggle cogging correction on/off (ENABLE=0|1) for A/B")
        gcode.register_mux_command('SET_PHASE_LEAD', 'STEPPER',
                                   self.stepper_name, self.cmd_SET_LEAD,
                                   desc="Set velocity commutation lead (FRAC of refresh period; 0=off)")
        gcode.register_mux_command('SET_PHASE_IDLE', 'STEPPER',
                                   self.stepper_name, self.cmd_SET_IDLE,
                                   desc="Idle/hold current: HOLD=% of full at standstill, IDLE_MS delay (HOLD=100 off)")
        gcode.register_mux_command('PHASE_SEG_TEST', 'STEPPER',
                                   self.stepper_name, self.cmd_SEG_TEST,
                                   desc="P0: drive ONE motor analytically (synthetic move)")
        gcode.register_mux_command('PHASE_CURRENT_INFO', 'STEPPER',
                                   self.stepper_name, self.cmd_CURRENT_INFO,
                                   desc="Report effective phase-step coil current vs Prusa 550mA")
        gcode.register_mux_command('PHASE_SEG_STATS', 'STEPPER',
                                   self.stepper_name, self.cmd_SEG_STATS,
                                   desc="Report analytic segment queue diagnostics")
        gcode.register_mux_command('PHASE_TRAPQ_ENGAGE', 'STEPPER',
                                   self.stepper_name, self.cmd_TRAPQ_ENGAGE,
                                   desc="P1: stream the live trapq as analytic segments")
        gcode.register_mux_command('PHASE_TRAPQ_DISENGAGE', 'STEPPER',
                                   self.stepper_name, self.cmd_TRAPQ_DISENGAGE,
                                   desc="Stop trapq streaming + clear direct_mode")

    def _mcu_identify(self):
        tmc = self.printer.lookup_object('tmc2130 %s' % (self.stepper_name,))
        self.tmc = tmc                             # for MSCNT read + direct_mode flip
        mcu_spi = tmc.mcu_tmc.tmc_spi.spi          # the shared bus.MCU_SPI
        self.mcu = mcu_spi.get_mcu()
        self.spi_oid = mcu_spi.get_oid()
        self._spi_bus_name = mcu_spi.bus           # e.g. "spi3a" (for slew-rate fix)
        self.oid = self.mcu.create_oid()
        if self.use_dma:
            self.dma_oid = self.mcu.create_oid()
        # find the MCU_stepper — force_move registers every stepper by name,
        # including the extruder (which isn't in the kinematics)
        try:
            self.mcu_stepper = self.printer.lookup_object(
                'force_move').lookup_stepper(self.stepper_name)
        except Exception:
            pass
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        stepper_oid = self.mcu_stepper.get_oid() if self.mcu_stepper else 0
        self.mcu.add_config_cmd(
            "config_phase_exec oid=%d spi_oid=%d stepper_oid=%d"
            % (self.oid, self.spi_oid, stepper_oid))
        if self.use_dma:
            self.mcu.add_config_cmd(
                "config_phase_dma oid=%d exec_oid=%d"
                % (self.dma_oid, self.oid))
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.start_cmd = self.mcu.lookup_command(
            "phase_exec_start oid=%c interval=%u phase_index=%hu"
            " phase_advance=%hi amp=%c swap=%c start_clock=%u run_timer=%c"
            " mscnt=%hu",
            cq=self.cmd_queue)
        self.stop_cmd = self.mcu.lookup_command(
            "phase_exec_stop oid=%c", cq=self.cmd_queue)
        self.traj_cmd = self.mcu.lookup_command(
            "phase_exec_trajectory oid=%c enable=%c", cq=self.cmd_queue)
        self.chain_cmd = self.mcu.lookup_command(
            "phase_exec_chain oid=%c next_oid=%c", cq=self.cmd_queue)
        self.corr_cmd = self.mcu.lookup_command(
            "phase_exec_set_corr oid=%c dir=%c harm=%c mag=%hi pha=%hu",
            cq=self.cmd_queue)
        self.blend_cmd = self.mcu.lookup_command(
            "phase_exec_blend oid=%c vband=%u", cq=self.cmd_queue)
        self.lead_cmd = self.mcu.lookup_command(
            "phase_exec_lead oid=%c ticks=%u", cq=self.cmd_queue)
        try:                                       # only in fw with idle-current support; degrade
            self.idle_cmd = self.mcu.lookup_command(
                "phase_exec_idle oid=%c hold=%c thresh=%u", cq=self.cmd_queue)
        except Exception:
            self.idle_cmd = None                   # old fw -> idle-current dormant (SET_PHASE_IDLE no-op)
        self.sweep_cmd = self.mcu.lookup_command(
            "phase_exec_sweep oid=%c active=%c harm=%c start_clock=%u dur=%u"
            " pha_start=%hi pha_diff=%hi mag_start=%hi mag_diff=%hi", cq=self.cmd_queue)
        self.osc_cmd = self.mcu.lookup_command(
            "phase_exec_osc oid=%c active=%c amp=%hu inc=%hu", cq=self.cmd_queue)
        self.analytic_cmd = self.mcu.lookup_command(
            "phase_exec_analytic oid=%c enable=%c", cq=self.cmd_queue)
        self.spi_div_cmd = self.mcu.lookup_command(
            "phase_exec_spi_div oid=%c div=%c", cq=self.cmd_queue)
        self.cs_slack_cmd = self.mcu.lookup_command(
            "phase_exec_cs_slack value=%u", cq=self.cmd_queue)
        self.pin_hs_cmd = self.mcu.lookup_command(
            "phase_exec_pin_hs pin=%u af=%c", cq=self.cmd_queue)
        self.seg_cmd = self.mcu.lookup_command(
            "phase_exec_seg oid=%c start_clock=%u duration=%u pos=%u v=%u ha=%u"
            " reanchor=%c",
            cq=self.cmd_queue)
        self.seg_query_cmd = self.mcu.lookup_query_command(
            "phase_exec_seg_query oid=%c",
            "phase_exec_seg_state oid=%c depth=%hu overflow=%hu dry=%hu cur_valid=%c"
            " minrep=%hu refills=%hu snapsum=%u snapmax=%u settle=%u coast=%u"
            " jump=%hu jumpmax=%u skipsame=%u",
            oid=self.oid, cq=self.cmd_queue)
        if self.use_dma:
            # synchronous query -> response (returns the parsed reply dict)
            self.query_cmd = self.mcu.lookup_query_command(
                "phase_dma_query oid=%c",
                "phase_dma_stats oid=%c tx=%u ovr=%u dmaerr=%u maxrx=%u skips=%u",
                oid=self.dma_oid, cq=self.cmd_queue)

    def _set_tmc_checks(self, enable):
        # During a run NOTHING on the host may touch spi3a, or it collides with the
        # executor's XDIRECT stream and corrupts the bus. stop_checks() isn't enough:
        # motor-enable/sync events (_do_enable -> _init_registers / _query_phase) also
        # do SPI. So for the duration we: stop the error-checks, shadow start_checks
        # to block re-enable, AND no-op get_register/set_register on EVERY spi3a driver
        # (the executor drives XDIRECT via its own MCU-side SPI path, unaffected).
        # All restored on stop. NOTE: clear direct_mode / restore current only AFTER
        # PHASE_EXEC_STOP, or those host writes get no-op'd too.
        n = 0
        for name, obj in self.printer.lookup_objects(module='tmc2130'):
            try:
                # obj.get_status.__self__ is the TMCCommandHelper; the periodic
                # error-check (the thing that reads GSTAT and collides) lives on
                # its .echeck_helper (a TMCErrorCheck). Resolve robustly.
                cmdhelper = obj.get_status.__self__
                ec = getattr(cmdhelper, 'echeck_helper', cmdhelper)
                mt = obj.mcu_tmc
                if enable:
                    if hasattr(ec, '_pe_saved_start'):
                        ec.start_checks = ec._pe_saved_start
                        del ec._pe_saved_start
                    if hasattr(mt, '_pe_saved_get'):
                        mt.get_register = mt._pe_saved_get
                        mt.set_register = mt._pe_saved_set
                        del mt._pe_saved_get, mt._pe_saved_set
                    ec.start_checks()
                else:
                    ec.stop_checks()
                    # idempotence: if suppress ever runs twice (guarded in cmd_TRAPQ_ENGAGE,
                    # but defend here) do NOT re-save -- the current fn is already the no-op shadow,
                    # so re-saving would lose the REAL get/set_register forever (restore would then
                    # "restore" the no-op) -> drivers stuck in direct_mode after disengage +
                    # SET_TMC_FIELD becomes a silent no-op.
                    if not hasattr(ec, '_pe_saved_start'):
                        ec._pe_saved_start = ec.start_checks
                        ec.start_checks = lambda *a, **k: False
                    if not hasattr(mt, '_pe_saved_get'):
                        mt._pe_saved_get = mt.get_register
                        mt._pe_saved_set = mt.set_register
                        # benign value: DRV_STATUS cs_actual=0x1F (not "reset"), GSTAT
                        # reset/drv_err/uv_cp = 0 -> a read during the run won't false-
                        # trip the TMC error check (0 looks exactly like a driver reset)
                        mt.get_register = lambda *a, **k: 0x001f0000
                        mt.set_register = lambda *a, **k: None
                n += 1
            except Exception:
                pass
        self._n_tmc = n

    def _set_bulk_sensors(self, suspend):
        # The 8kHz refresh pressures the MCU main loop; the first thing to die is the
        # HX717 loadcell's periodic sensor_bulk_status query (host times out -> shutdown).
        # The loadcell bulk reader ALSO costs ~3200 Klipper-timer dispatches/s of its own.
        # So pause every main-MCU (F427) bulk sensor for the duration of the run, exactly
        # like _set_tmc_checks pauses TMC polling. BatchBulkHelper._stop() halts MCU
        # sampling (rest_ticks=0) + host batch processing; _start() resumes. The H503
        # extension sensors are on a separate MCU/link and are left alone.
        if suspend:
            self._suspended_bulk = []
            seen = set()
            for name, obj in self.printer.lookup_objects():
                # ONLY pause STANDALONE bulk sensors (a HX71xBase with a DIRECT
                # batch_bulk, e.g. the filament sensor). Do NOT touch a LoadCell
                # wrapper's .sensor: raw _stop/_start corrupts the LoadCell's
                # sample-collector state and breaks Z-probe homing. The load cell
                # only samples during probing, so it isn't a steady-state burden.
                bb = getattr(obj, 'batch_bulk', None)
                if bb is None or id(bb) in seen:
                    continue
                if not (hasattr(bb, '_stop') and hasattr(bb, '_start')):
                    continue
                omcu = obj.get_mcu() if hasattr(obj, 'get_mcu') \
                    else getattr(obj, 'mcu', None)
                if omcu is not self.mcu:
                    continue
                seen.add(id(bb))
                try:
                    bb._stop()
                    self._suspended_bulk.append(bb)
                except Exception:
                    pass
        else:
            for bb in self._suspended_bulk:
                try:
                    bb._start()
                except Exception:
                    pass
            self._suspended_bulk = []

    def _restore_tmc_patches(self):
        # Un-monkeypatch every tmc2130 (HOST-side only, sends NO MCU commands) so a
        # crashed run can't leave register access no-op'd. Idempotent. The periodic
        # error-check re-arms by itself on the next klippy connect/firmware_restart.
        for name, obj in self.printer.lookup_objects(module='tmc2130'):
            try:
                cmdhelper = obj.get_status.__self__
                ec = getattr(cmdhelper, 'echeck_helper', cmdhelper)
                mt = obj.mcu_tmc
                if hasattr(ec, '_pe_saved_start'):
                    ec.start_checks = ec._pe_saved_start
                    del ec._pe_saved_start
                if hasattr(mt, '_pe_saved_get'):
                    mt.get_register = mt._pe_saved_get
                    mt.set_register = mt._pe_saved_set
                    del mt._pe_saved_get, mt._pe_saved_set
            except Exception:
                pass
        self._n_tmc = 0

    def _suppress_step_gen(self, steppers, enable):
        # stop X/Y itersolve step generation while phase-driven. set_trapq(None)
        # detaches the stepper's kinematic solver from the toolhead trapq -> itersolve
        # emits NO steps for it. The phase streamer reads the toolhead trapq directly so
        # motion is unaffected; this removes the wasted X/Y steps (the TMC ignores them in
        # direct_mode anyway) -> frees host step-gen + MCU step-exec CPU/bandwidth for the
        # 40kHz refresh. RESTORE re-attaches the saved trapq and re-syncs the MCU step
        # position (the counter was frozen while suppressed); the mandatory G28 after
        # disengage then fully resyncs via set_position. NOTE: between restore and G28 the
        # host position is stale, so disengage must go straight to G28 with no moves.
        if enable:
            self._suppressed_trapq = []
            for ms in steppers:
                if ms is None:
                    continue
                self._suppressed_trapq.append((ms, ms.get_trapq()))
                ms.set_trapq(None)
        else:
            had_suppressed = bool(self._suppressed_trapq)
            for ms, saved in self._suppressed_trapq:
                try:
                    ms.set_trapq(saved)
                    ms.note_homing_end()   # reset stepcompress + query real MCU pos
                except Exception:
                    pass
            self._suppressed_trapq = []
            # (C) Counter-slaving on disengage. While suppressed (set_trapq(None))
            # each stepper's itersolve COMMANDED position was frozen at engage,
            # but the toolhead advanced through all phase-step motion. note_homing_end
            # above only re-queries the (also frozen) MCU counter -- it does NOT move
            # the commanded position. So the first post-disengage move (even a G28
            # Z-hop, which still flushes the CoreXY motor step-gens) would try to
            # bridge engage->now in ONE move -> impossible step interval -> "Internal
            # error in stepcompress" shutdown. Re-assert the toolhead's current
            # commanded position: set_position re-bases every stepper's itersolve
            # commanded pos to (x,y,z) while _set_mcu_position keeps the MCU step
            # count (preserved via offset), so subsequent moves emit sane deltas.
            # The G28 that follows then finds the true frame -- and its homing
            # trigger reveals any accumulated open-loop drift.
            if had_suppressed:
                try:
                    th = self.printer.lookup_object('toolhead')
                    th.flush_step_generation()
                    th.set_position(th.get_position())
                except Exception:
                    pass

    def _handle_shutdown(self):
        # MCU is down: restore only host-side state (no MCU commands possible).
        if getattr(self, '_trapq_engaged', False):
            self._trapq_engaged = False
            try:
                self.printer.lookup_object('motion_queuing'
                                           ).unregister_flush_callback(
                                               self._trapq_flush_cb)
            except Exception:
                pass
        # re-attach any detached X/Y trapq (host-side only; MCU pos resyncs on the
        # firmware_restart/G28 that follows a shutdown) so the next homing isn't broken.
        for ms, saved in self._suppressed_trapq:
            try:
                ms.set_trapq(saved)
            except Exception:
                pass
        self._suppressed_trapq = []
        self._restore_tmc_patches()

    def _xdirect_vec(self, idx, amp, swap):
        # Mirror the MCU's pe_compute_frame for one LUT index so the host can
        # PRE-LOAD the TMC's XDIRECT with the rotor's current vector before the
        # direct_mode flip. coil_A=cos, coil_B=sin (then swap), packed signed-9bit
        # in coil_A bits 8:0 / coil_B bits 24:16 — identical to the firmware.
        idx &= (PHASE_UNITS - 1)
        sa = int(round(16383 * math.cos(2 * math.pi * idx / PHASE_UNITS)))  # cos
        sb = int(round(16383 * math.sin(2 * math.pi * idx / PHASE_UNITS)))  # sin
        ca = (amp * sa) >> 14
        cb = (amp * sb) >> 14
        if swap:
            ca, cb = cb, ca
        return ((cb & 0x1ff) << 16) | (ca & 0x1ff)

    def _engage_motor(self, tmc, amp, swap, trim):
        # Seamless handoff from step/dir to XDIRECT, done while the bus is still ours
        # (BEFORE _set_tmc_checks suppression). Read the rotor's actual commutation
        # (MSCNT), pre-load XDIRECT with that exact vector, THEN flip direct_mode — so
        # the instant the TMC switches to direct drive the coils don't move. Returns
        # the (trimmed) MSCNT to hand the MCU so its stream continues from the same
        # angle. trim nulls any residual constant convention offset (default 0).
        fields = tmc.mcu_tmc.get_fields()
        # (Prusa parity, phase_stepping.cpp:409-416 "switch off interpolation first to ensure
        # position is settled"): read MSCNT with INTERPOLATION OFF. With intpol on (our config
        # interpolate:True, MRES=16), the TMC can report a not-yet-settled interpolated microstep
        # index -> a small CONSTANT per-engage phase offset baked into the pre-loaded XDIRECT vector.
        # intpol is a mux (not a filter): disabling it makes MSCNT report the true commanded index
        # at once. Scoped with try/finally so intpol is ALWAYS restored, even if the read throws --
        # a leaked intpol=0 would only coarsen step/dir motion (never brick), but we don't leak it.
        # Deliberately NOT matching Prusa's microsteps(256): our vector is computed from the raw
        # 0..1023 MSCNT (full-resolution regardless of MRES), so an MRES change is a second field to
        # save/restore for zero gain and extra interrupt-window risk. Runs BEFORE _set_tmc_checks
        # suppression, so set/get_register are still real here.
        # No explicit dwell: the CHOPCONF write and the MSCNT read are sent back-to-back on the
        # same MCU command queue, so the TMC applies intpol=0 before the MSCNT read frame arrives;
        # Klipper's pipelined two-transfer read covers chip settling. (A toolhead.dwell would gate
        # print_time, not the real-time SPI queue -- it would not order these writes.)
        intpol_reg = fields.lookup_register("intpol")      # -> "CHOPCONF"
        intpol_saved = fields.get_field("intpol")
        try:
            tmc.mcu_tmc.set_register(intpol_reg, fields.set_field("intpol", 0))
            mscnt = (tmc.mcu_tmc.get_register("MSCNT") + trim) & (PHASE_UNITS - 1)
        finally:
            tmc.mcu_tmc.set_register(intpol_reg, fields.set_field("intpol", intpol_saved))
        vec = self._xdirect_vec(mscnt, amp, swap)
        tmc.mcu_tmc.set_register("XDIRECT", vec)           # pre-load aligned vector
        reg = fields.lookup_register("direct_mode")        # -> "GCONF"
        val = fields.set_field("direct_mode", 1)
        tmc.mcu_tmc.set_register(reg, val)                 # flip: seamless, no lurch
        return mscnt

    def _disengage_motor(self, tmc, amp, swap):
        # Symmetric counterpart to _engage_motor. The sequencer's MSCNT is FROZEN at the engage
        # angle all run (step-gen suppressed -> no pulses), while the last XDIRECT leaves the rotor
        # at the trajectory-end angle. Clearing direct_mode blind lets the sequencer resume on the
        # stale MSCNT and SNAP the rotor to it by up to +-1/2 electrical period (the 512/0.16mm
        # quantum) -> intermittently shifts the sensorless-home trip -> the MSCNT_HOME "phase off
        # 512" failure that a firmware_restart clears. Fix: BEFORE releasing direct_mode, drive
        # XDIRECT to the current MSCNT angle so the rotor is already aligned to the counter; the
        # release is then lurch-free and post-print homing is repeatable. At most a 1/2-period
        # (0.16mm) smooth rotor move; a re-home follows regardless.
        try:
            mscnt = tmc.mcu_tmc.get_register("MSCNT") & (PHASE_UNITS - 1)
            tmc.mcu_tmc.set_register("XDIRECT", self._xdirect_vec(mscnt, amp, swap))
            self.printer.lookup_object('toolhead').dwell(0.010)  # settle coils at aligned vector
            return mscnt
        except Exception:
            return -1

    def cmd_START(self, gcmd):
        rate = gcmd.get_float('RATE', 10000., minval=100., maxval=60000.)
        advance = gcmd.get_int('ADVANCE', 4)       # phase units per update (signed)
        amp = gcmd.get_int('AMP', 120, minval=0, maxval=255)
        swap = gcmd.get_int('SWAP', 1)
        phase = gcmd.get_int('PHASE', 0) % PHASE_UNITS
        trim = gcmd.get_int('TRIM', 0)             # null residual handoff offset (master)
        # WITH=<stepper>: drive a SECOND motor coordinated on the shared bus (CoreXY).
        # This executor becomes the chain MASTER (owns the timer); WITH is the slave.
        with_name = gcmd.get('WITH', None)
        swap2 = gcmd.get_int('SWAP2', swap)        # the slave's coil-swap (its direction)
        trim2 = gcmd.get_int('TRIM2', trim)        # the slave's handoff trim
        traj = gcmd.get_int('TRAJ', 1)             # group default: follow trajectory
        interval = max(1, int(self.mcu.seconds_to_clock(1.0 / rate)))
        # schedule the first tick a hair in the future
        est = self.mcu.estimated_print_time(
            self.printer.get_reactor().monotonic())
        clock = self.mcu.print_time_to_clock(est + 0.2)
        # Resolve the chained motor (if any) up front — we engage it before suppression.
        slave = None
        if with_name is not None:
            slave = self.printer.lookup_object('phase_exec %s' % (with_name,), None)
            if slave is None:
                raise gcmd.error("no [phase_exec %s] for WITH=" % (with_name,))
        # SEAMLESS ENGAGE — must precede _set_tmc_checks (which no-ops TMC SPI): read
        # each rotor's MSCNT, pre-load its aligned XDIRECT vector, flip direct_mode. The
        # returned (trimmed) MSCNT is handed to the MCU so its stream starts there.
        mscnt = self._engage_motor(self.tmc, amp, swap, trim)
        mscnt2 = self._engage_motor(slave.tmc, amp, swap2, trim2) if slave else 0
        self._set_tmc_checks(False)        # exclusive bus access during the run
        # PAUSE_LOADCELL=1 frees the MCU main loop for >4kHz, BUT the filament
        # sensor + Z-probe load cell share ONE HX717 (mode-switched), so pausing it
        # DISTURBS the load cell -> Z-homing fails until a klipper restart. OFF by
        # default (homing-safe). Only enable for a deliberate >4kHz experiment, and
        # `systemctl restart klipper` before the next G28.
        self._pause_lc = bool(gcmd.get_int('PAUSE_LOADCELL', 0))
        if self._pause_lc:
            self._set_bulk_sensors(True)
        self._send_corrections()               # (re)apply this motor's cogging spectrum
        if slave is not None:
            slave._send_corrections()          # ...and the slave's
        self._send_blend(slave)                # fwd<->bwd blend band (self + slave)
        if slave is not None:
            # chain master -> slave -> end
            self.chain_cmd.send([self.oid, slave.oid])
            self.chain_cmd.send([slave.oid, 0])
            self.traj_cmd.send([self.oid, traj])
            self.traj_cmd.send([slave.oid, traj])
            # slave first (params + active, NO timer), then master (timer drives chain)
            self.start_cmd.send([slave.oid, interval, phase, advance,
                                 amp, swap2, clock & 0xffffffff, 0, mscnt2])
            self.start_cmd.send([self.oid, interval, phase, advance,
                                 amp, swap, clock & 0xffffffff, 1, mscnt])
        else:
            self.chain_cmd.send([self.oid, 0])     # ensure no stale chain link
            self.start_cmd.send([self.oid, interval, phase, advance,
                                 amp, swap, clock & 0xffffffff, 1, mscnt])
        self._group_slave = slave
        gcmd.respond_info(
            "[phase_exec %s] RATE=%.0fHz interval=%d, amp=%d, swap=%d, mscnt=%d%s. "
            "tmc_suppressed=%d bulk_paused=%d. PHASE_EXEC_STOP to halt."
            % (self.stepper_name, rate, interval, amp, swap, mscnt,
               (" + WITH=%s swap2=%d mscnt2=%d" % (with_name, swap2, mscnt2))
               if slave else "",
               getattr(self, '_n_tmc', -1), len(self._suspended_bulk)))

    @staticmethod
    def _f2u(x):
        # bit-cast a Python float -> the uint32 the MCU reinterprets as IEEE754 float
        import struct
        return struct.unpack('<I', struct.pack('<f', float(x)))[0]

    def cmd_SEG_TEST(self, gcmd):
        # single-motor validation: drive ONE motor from synthetic analytic segments (no trapq,
        # no input shaping). Extruder-safe — driving a single CoreXY motor open-loop
        # RACKS the gantry, so default/run this on the EXTRUDER. Positions are in
        # step-units (stepper_get_position frame): for the extruder ~380 step-units =
        # 1 mm of filament, 64 step-units = one electrical period.
        rate = gcmd.get_float('RATE', 8000., minval=100., maxval=60000.)
        amp = gcmd.get_int('AMP', 120, minval=0, maxval=255)
        swap = gcmd.get_int('SWAP', 1)
        cur = gcmd.get_float('CURRENT', 0.5, minval=0.05, maxval=1.2)
        vel = gcmd.get_float('VEL', 380.)        # step-units/s (peak)
        accel = gcmd.get_float('ACCEL', 0.)      # step-units/s^2 (0 = pure cruise)
        dur = gcmd.get_float('DUR', 2.0, minval=0.05, maxval=20.)  # cruise seconds
        if self.seg_cmd is None:
            raise gcmd.error("phase_exec %s: not DMA-configured / no seg cmd"
                             % (self.stepper_name,))
        gca = self.printer.lookup_object('gcode')
        gca.run_script_from_command(
            "SET_STEPPER_ENABLE STEPPER=%s ENABLE=1" % (self.stepper_name,))
        gca.run_script_from_command(
            "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f HOLDCURRENT=%.3f"
            % (self.stepper_name, cur, cur))
        # Build a synthetic profile (step-units), contiguous in time, starting at pos=0.
        # ACCEL=0 -> single cruise segment; ACCEL>0 -> symmetric trapezoid to VEL.
        # segs: list of (duration_s, start_v, half_accel); start_pos chained below.
        segs = []
        if accel > 0.:
            ta = vel / accel                     # ramp time to reach VEL
            segs.append((ta, 0.0, 0.5 * accel))  # accelerate 0 -> VEL
            segs.append((dur, vel, 0.0))         # cruise at VEL
            segs.append((ta, vel, -0.5 * accel)) # decelerate VEL -> 0
        else:
            segs.append((dur, vel, 0.0))         # pure cruise
        # Engage seamlessly, then queue + start. Order mirrors cmd_START: engage
        # (own the bus) BEFORE suppressing TMC checks; analytic ON before start so the
        # start command takes the analytic phase_offset (=MSCNT) branch.
        est = self.mcu.estimated_print_time(
            self.printer.get_reactor().monotonic())
        start_pt = est + 0.25
        clock0 = self.mcu.print_time_to_clock(start_pt)
        mscnt = self._engage_motor(self.tmc, amp, swap, 0)
        self._set_tmc_checks(False)              # exclusive bus for the run
        self.analytic_cmd.send([self.oid, 1])    # analytic mode ON (clears ring)
        self._send_corrections()
        # queue the segments with contiguous absolute start clocks + chained start_pos
        pos = 0.0
        acc_ticks = 0
        for (dt, v, ha) in segs:
            dticks = int(self.mcu.seconds_to_clock(dt))
            sc = (clock0 + acc_ticks) & 0xffffffff
            self.seg_cmd.send([self.oid, sc, dticks,
                               self._f2u(pos), self._f2u(v), self._f2u(ha), 0])
            pos += (v + ha * dt) * dt             # endpoint -> next segment's start_pos
            acc_ticks += dticks
        interval = max(1, int(self.mcu.seconds_to_clock(1.0 / rate)))
        # start the chain master timer at clock0; mscnt sets the analytic phase_offset.
        # phase_advance/phase_index unused in analytic mode (segments drive the angle).
        self.start_cmd.send([self.oid, interval, 0, 0, amp, swap,
                             clock0 & 0xffffffff, 1, mscnt])
        self._group_slave = None
        total_s = sum(dt for (dt, _, _) in segs)
        gcmd.respond_info(
            "[phase_exec %s] SEG_TEST: %d seg(s), %.2fs, peak %.0f step-u/s, "
            "accel=%.0f, rate=%.0fHz, amp=%d. Expected travel ~%.1f step-units "
            "(~%.2f electrical periods). PHASE_EXEC_STOP + clear direct_mode after."
            % (self.stepper_name, len(segs), total_s, vel, accel, rate, amp,
               pos, pos / 64.0))

    def cmd_SEG_STATS(self, gcmd):
        if self.seg_query_cmd is None:
            raise gcmd.error("no analytic seg query on %s" % (self.stepper_name,))
        p = self.seg_query_cmd.send([self.oid])
        gcmd.respond_info(
            "[phase_exec %s] seg depth=%d overflow=%d dry=%d cur_valid=%d"
            % (self.stepper_name, p['depth'], p['overflow'], p['dry'],
               p['cur_valid']))

    def _current_info_str(self):
        # Effective coil current = the torque-margin lever for open-loop slip (Prusa: slip at
        # loaded first-layer reversals is fixed ONLY by current). In direct_mode (XDIRECT) the
        # TMC scales by IHOLD (it looks like standstill), so HOLD current is the ceiling at the
        # reversal. Prusa runs 550mA with IHOLD=IRUN. Reads Klipper's cached TMC fields (the
        # registers are write-only -> can't read back). Meaningful while phase-step engaged
        # (the engage sets IHOLD=IRUN=CURRENT); at standstill it shows the normal config.
        tmc = self.tmc
        try:
            st = tmc.get_status()
        except Exception:
            st = {}
        run_c = st.get('run_current')
        hold_c = st.get('hold_current')

        def fld(name):
            try:
                return tmc.fields.get_field(name)
            except Exception:
                return None
        irun, ihold, vsense = fld('irun'), fld('ihold'), fld('vsense')
        msg = ("[phase_exec %s] run=%s hold=%s IRUN=%s IHOLD=%s vsense=%s | "
               "Prusa ref: 550mA, IHOLD=IRUN"
               % (self.stepper_name,
                  ("%.3fA" % run_c) if run_c is not None else "?",
                  ("%.3fA" % hold_c) if hold_c is not None else "?",
                  irun, ihold, vsense))
        notes = []
        if ihold is not None and irun is not None and ihold != irun:
            notes.append("IHOLD!=IRUN -> phase-step runs at the LOWER hold current = slip risk")
        if hold_c is not None and hold_c < 0.5:
            notes.append("hold < Prusa 550mA -> raise CURRENT")
        if notes:
            msg += "  WARN: " + "; ".join(notes)
        return msg

    def cmd_CURRENT_INFO(self, gcmd):
        gcmd.respond_info(self._current_info_str())

    # ---- live trapq -> analytic segment streamer (extruder, host-only) ---
    def _trapq_setup(self, corexy=False):
        # Resolve the trapq + cffi + steps/mm. The extruder's own 1D trapq.
        # CoreXY: the toolhead trapq (cartesian moves), projected per motor.
        import chelper
        self._ffi_main, self._ffi_lib = chelper.get_ffi()
        if corexy:
            self._trapq = self.printer.lookup_object('toolhead').get_trapq()
        else:
            extr = self.printer.lookup_object('extruder', None)
            self._trapq = extr.get_trapq() if extr is not None else None
        self._steps_per_mm = 1.0 / self.mcu_stepper.get_step_dist()

    @staticmethod
    def _init_shaper_pulses(A, T):
        # Replicate chelper/kin_shaper.c init_shaper + shift_pulses EXACTLY so the host
        # convolution matches the firmware's: reverse the traditional (a,t) pairs,
        # normalize so sum(a)=1, then shift t so sum(a*t)=0 (identity for const-velocity:
        # input_shaper(v*T) = v*T). Returns (a_list, t_list).
        n = len(A)
        if n == 0:
            return ([1.0], [0.0])
        inv_a = 1.0 / sum(A)
        a = [0.0] * n
        t = [0.0] * n
        for i in range(n):
            a[n - i - 1] = A[i] * inv_a
            t[n - i - 1] = -T[i]
        ts = sum(a[i] * t[i] for i in range(n))
        for i in range(n):
            t[i] -= ts
        return (a, t)

    def _load_shaper_pulses(self, enable):
        # Pull the live [input_shaper] X/Y pulses for the CoreXY projection. Z and the
        # extruder are never shaped here (Klipper shapes only x/y/z; extruder has none).
        # The stream-horizon lag = the largest POSITIVE pulse time, so every shaped
        # sample T+t stays within moves already present at the flush callback (moves
        # ahead of flush_time up to step_gen_time are available; kin_flush_delay covers
        # the shaper scan window).
        self._px = ([1.0], [0.0])
        self._py = ([1.0], [0.0])
        self._pulse_margin = 0.0
        self._shape = False
        if not enable:
            return
        is_obj = self.printer.lookup_object('input_shaper', None)
        if is_obj is None:
            return
        for shaper in is_obj.get_shapers():
            if shaper.axis not in ('x', 'y') or not shaper.is_enabled():
                continue
            n, A, T = shaper.get_shaper()
            pulses = self._init_shaper_pulses(list(A), list(T))
            if shaper.axis == 'x':
                self._px = pulses
            else:
                self._py = pulses
            self._shape = True
        margin = 0.0
        for _a, tl in (self._px, self._py):
            for ti in tl:
                if ti > margin:
                    margin = ti
        self._pulse_margin = margin

    def _eval_axis(self, moves, tau):
        # Unshaped cartesian (or extruder) state at absolute print_time tau: the move
        # with the largest print_time <= tau. Returns (xp,xv,xa, yp,yv,ya). pull_move
        # .accel is the FULL accel (2*half_accel); dist = v*t + 0.5*accel*t^2. Klipper's
        # trapq is contiguous so an interior tau always lands in a real move; OUTSIDE any
        # move (before the first, or in a dwell gap) the toolhead is idle = FROZEN, so
        # hold the boundary position with vel=0 AND accel=0 (returning the move's accel
        # there would inject phantom acceleration — that was the shaped-segment first-pass bug).
        lo, hi = 0, len(moves)
        while lo < hi:
            mid = (lo + hi) >> 1
            if moves[mid].print_time <= tau:
                lo = mid + 1
            else:
                hi = mid
        idx = lo - 1
        if idx < 0:                      # before all moves: static at first move's start
            m = moves[0]
            return (m.start_x, 0.0, 0.0, m.start_y, 0.0, 0.0)
        m = moves[idx]
        trel = tau - m.print_time
        if trel > m.move_t:
            # The pt-bisection winner does NOT contain tau. A well-formed trapq is contiguous, so
            # normally this is a real gap/dwell -> freeze at the end. BUT a SHADOW/OVERLAP entry (a
            # move whose print_time falls INSIDE an earlier, longer move's span) also wins the
            # bisection while the move that TRULY contains tau sits earlier in the list -> we'd
            # freeze at the shadow's stale endpoint = the shaped-eval glitch on travel arrivals
            # (root-cause 2026-07-03). So look back for a move that CONTAINS tau; if found use
            # it in-move; else freeze at the LATEST true endpoint <= tau (not the winner's, which may
            # be a short shadow). Behavior-identical on a clean/contiguous list.
            contain = None
            best = idx
            k = idx - 1
            while k >= 0 and idx - k <= 8:      # overlap windows are short; bound the back-scan
                mk = moves[k]
                tk = tau - mk.print_time
                if 0.0 <= tk <= mk.move_t:
                    contain = mk; trel = tk; break
                if (mk.print_time + mk.move_t
                        > moves[best].print_time + moves[best].move_t):
                    best = k
                k -= 1
            if contain is not None:
                m = contain
            else:
                m = moves[best]
                d = m.start_v * m.move_t + 0.5 * m.accel * m.move_t * m.move_t
                return (m.start_x + m.x_r * d, 0.0, 0.0,
                        m.start_y + m.y_r * d, 0.0, 0.0)
        dist = m.start_v * trel + 0.5 * m.accel * trel * trel
        vel = m.start_v + m.accel * trel
        xp = m.start_x + m.x_r * dist
        xv = m.x_r * vel
        xa = m.x_r * m.accel
        yp = m.start_y + m.y_r * dist
        yv = m.y_r * vel
        ya = m.y_r * m.accel
        return (xp, xv, xa, yp, yv, ya)

    def _shaped_motor(self, moves, T, xs, ys):
        # Shaped CoreXY motor projection at absolute time T:
        #   X_s = sum_i ax_i * X(T+tx_i),  Y_s = sum_j ay_j * Y(T+ty_j)
        # (x-shaper convolves X, y-shaper convolves Y — independent, exactly as
        # kin_shaper.c does). Then motor A('+') = xs*X_s + ys*Y_s, B('-') = xs*X_s - ys*Y_s
        # (xs/ys = per-axis direction signs). Sum of per-move quadratics is itself a
        # quadratic, so between pulse/move-boundary crossings this is an EXACT constant-
        # accel segment. Returns pos/vel/accel (mm) for both motors as
        # ((Apos,Avel,Aacc),(Bpos,Bvel,Bacc)).
        px_a, px_t = self._px
        py_a, py_t = self._py
        Xp = Xv = Xa = Yp = Yv = Ya = 0.0
        for a, ti in zip(px_a, px_t):
            xp, xv, xa, _yp, _yv, _ya = self._eval_axis(moves, T + ti)
            Xp += a * xp; Xv += a * xv; Xa += a * xa
        for a, ti in zip(py_a, py_t):
            _xp, _xv, _xa, yp, yv, ya = self._eval_axis(moves, T + ti)
            Yp += a * yp; Yv += a * yv; Ya += a * ya
        Xp *= xs; Xv *= xs; Xa *= xs
        return ((Xp + ys * Yp, Xv + ys * Yv, Xa + ys * Ya),
                (Xp - ys * Yp, Xv - ys * Yv, Xa - ys * Ya))

    def _build_st_forward(self, moves, pts, xs, ys):
        # FORWARD-CURSOR shaped-state build (Prusa m_move[] model). Instead of _eval_axis BISECTING
        # the move buffer at every (pts[k]+pulse), keep ONE cursor per shaper pulse that advances
        # monotonically as pts[k] increases (pts is sorted and each pulse ti is fixed, so pts[k]+ti
        # is monotonic in k). No per-sample re-search -> O(1) amortized vs O(log n) bisect = less host
        # CPU on dense arc geometry, AND on the zero-duration-filtered (monotonic, non-overlapping)
        # queue the cursor structurally cannot land on a shadow entry. A bounded containment back-
        # scan stays as belt-and-braces for any non-zero overlap that ever slips the filter.
        # Returns the same ((Apos,Avel,Aacc),(Bpos,Bvel,Bacc)) list as [_shaped_motor(pts[k])...].
        npts = len(pts); nmoves = len(moves)
        Xp = [0.0] * npts; Xv = [0.0] * npts; Xa = [0.0] * npts
        Yp = [0.0] * npts; Yv = [0.0] * npts; Ya = [0.0] * npts
        for shaper, P, Vl, Ac, is_x in ((self._px, Xp, Xv, Xa, True),
                                        (self._py, Yp, Yv, Ya, False)):
            for a, ti in zip(shaper[0], shaper[1]):
                cur = 0
                for k in range(npts):
                    tau = pts[k] + ti
                    while cur + 1 < nmoves and moves[cur + 1].print_time <= tau:
                        cur += 1              # advance forward (never search backward)
                    m = moves[cur]; trel = tau - m.print_time
                    if trel > m.move_t:       # winner doesn't contain tau -> containment back-scan
                        best = cur; j = cur - 1; found = False
                        while j >= 0 and cur - j <= 8:
                            mj = moves[j]; tj = tau - mj.print_time
                            if 0.0 <= tj <= mj.move_t:
                                m = mj; trel = tj; found = True; break
                            if (mj.print_time + mj.move_t
                                    > moves[best].print_time + moves[best].move_t):
                                best = j
                            j -= 1
                        if not found:
                            m = moves[best]; trel = tau - m.print_time   # >move_t -> frozen below
                    sx = m.start_x if is_x else m.start_y
                    r = m.x_r if is_x else m.y_r
                    if trel < 0.0:
                        P[k] += a * sx                                    # before first move: static
                    elif trel > m.move_t:
                        d = m.start_v * m.move_t + 0.5 * m.accel * m.move_t * m.move_t
                        P[k] += a * (sx + r * d)                          # frozen end (vel=acc=0)
                    else:
                        dist = m.start_v * trel + 0.5 * m.accel * trel * trel
                        vel = m.start_v + m.accel * trel
                        P[k] += a * (sx + r * dist)
                        Vl[k] += a * (r * vel)
                        Ac[k] += a * (r * m.accel)
        st = []
        for k in range(npts):
            xp = Xp[k] * xs; xv = Xv[k] * xs; xa = Xa[k] * xs
            yp = Yp[k]; yv = Yv[k]; ya = Ya[k]
            st.append(((xp + ys * yp, xv + ys * yv, xa + ys * ya),
                       (xp - ys * yp, xv - ys * yv, xa - ys * ya)))
        return st

    def _trapq_flush_cb(self, flush_time, step_gen_time):
        # Fired on every motion flush. Stream the SHAPED per-motor trajectory as exact
        # constant-accel segments. The shaped position X_s(t)=sum a_i*X(t+t_i) is
        # piecewise-quadratic, breaking only where a pulse offset (t+t_i) crosses a trapq
        # move boundary (Prusa's `nearest_next_change`). Between breakpoints the sum of
        # quadratics is itself one constant-accel segment, evaluated analytically and
        # pushed PER MOTOR (CoreXY: A='+'=Xs+ys*Ys, B='-'=Xs-ys*Ys). With trivial pulses
        # ([1],[0]) this reduces exactly to plain live-trapq streaming (one segment per move, chopped at the
        # flush window edges). We lag the stream horizon by _pulse_margin (max positive
        # pulse time) so every shaped sample stays within moves already in the trapq.
        if not self._trapq_engaged or self._trapq is None or self.seg_cmd is None:
            return
        t0 = self._trapq_last_pt
        t1 = flush_time - self._pulse_margin
        if t1 <= t0 + 1e-9:
            return
        # after a long idle while engaged (PAUSE / M600 / long TEMPERATURE_WAIT), t0 is stale
        # and [t0,t1] spans the WHOLE gap -> the DT_MAX grid (a breakpoint every 4 ms) would insert
        # thousands of them in ONE reactor callback (a 60 s pause = ~15k -> ~30k+ _shaped_motor
        # evals) -> klippy stalls seconds-to-minutes -> MCU serial timeout / "Timer too close" on
        # resume. The MCU held last_pos through the drain, so SKIP the gap: process only the tail
        # and re-anchor the origin on resume (identical to the queue-drained path below). Do NOT
        # emit a gap-spanning hold segment -- a >12.8 s duration corrupts the reanchor duration-MSB
        # packing (phase_exec.c).
        GAP_MAX = 1.0
        if t1 - t0 > GAP_MAX:
            t0 = self._trapq_last_pt = t1 - 0.25
            self._reanchor_next = True
        px_t = self._px[1]
        py_t = self._py[1]
        all_t = px_t + py_t
        min_t = min(all_t)
        max_t = max(all_t)
        # pull every move that any shaped sample in [t0,t1] could touch (+5ms guard).
        # The old 256 cap SILENTLY DROPPED the OLDEST moves when a dense/fast window
        # held >256 -> wrong shaped-pos anchor for the window's early samples -> the segment
        # start_pos jumped -> the MCU field TELEPORTED (field-teleport JUMP, overflow=0). Big cap + reused
        # buffer (no per-flush alloc churn) + a tripwire if we ever hit it.
        cap = self._extract_cap
        if self._extract_buf is None:
            self._extract_buf = self._ffi_main.new('struct pull_move[%d]' % cap)
        data = self._extract_buf
        cnt = self._ffi_lib.trapq_extract_old(
            self._trapq, data, cap, t0 + min_t - 0.005, t1 + max_t + 0.005)
        if cnt >= cap:
            logging.warning("phase_exec %s: trapq extract cap HIT (%d) -> possible dropped "
                            "moves -> field teleport; raise _extract_cap", self.stepper_name, cnt)
        if not cnt:
            # idle span: feed a zero-velocity HOLD segment so the MCU FREEZES at the last
            # commanded position. The old bare `return` left the MCU to COAST on stale
            # end-velocity (Fix 2) -> a mid-travel host stall at a layer change flew the
            # motor >1/2 electrical period -> a permanent per-layer slip (the Y staircase).
            # The comment here used to say "MCU holds last_pos" -- true before Fix 2, false
            # after; this restores that contract explicitly. Gapless: [t0,t1] abuts the
            # prior segment (which ended at t0 = the prior t1).
            if self._trapq_last_posA is not None:
                c0 = self.mcu.print_time_to_clock(t0)
                sc = c0 & 0xffffffff
                dticks = int(self.mcu.print_time_to_clock(t1) - c0)
                if dticks > 0:
                    self.seg_cmd.send([self.oid, sc, dticks,
                                       self._f2u(self._trapq_last_posA),
                                       self._f2u(0.0), self._f2u(0.0), 0])
                    if (self._trapq_slave_oid is not None
                            and self._trapq_last_posB is not None):
                        self.seg_cmd.send([self._trapq_slave_oid, sc, dticks,
                                           self._f2u(self._trapq_last_posB),
                                           self._f2u(0.0), self._f2u(0.0), 0])
            self._trapq_last_pt = t1
            self._reanchor_next = True      # queue drained -> the next resume re-anchors
            return
        # Filter ZERO-DURATION moves (move_t<=0): they carry no displacement (dist=v*0+0.5*a*0=0),
        # so dropping them leaves the shaped trajectory identical -- but they ARE the shadow entries
        # (mt=0 arc-vertex moves whose print_time falls inside a neighbor's span, the PHOVERLAP/
        # containment source) AND they roughly double the arc boundary/segment count that overflows
        # the MCU seg ring. Removing at the source fixes both: no shadows to recover from, ~half the
        # segments on dense arc geometry. Contiguous non-zero moves still chain exactly.
        moves = sorted((data[i] for i in range(cnt) if data[i].move_t > 1e-9),
                       key=lambda m: m.print_time)
        if not moves:            # (rare) window was ALL zero-duration -> keep them so _eval_axis
            moves = sorted((data[i] for i in range(cnt)),   # still has a valid anchor (no crash);
                           key=lambda m: m.print_time)       # containment-first eval covers shadows
        # PHOVERLAP tripwire: a move whose print_time starts BEFORE the running-max end of
        # prior moves = a shadow/overlap entry -> it wins _eval_axis's pt-bisection while an earlier
        # longer move truly contains tau -> the stale-frozen-endpoint shaped glitch. _eval_axis now
        # recovers from it (containment-first); this logs the culprit's identity so its origin can
        # be named. Cheap (one linear scan/flush); fires only on a dirty list.
        _pend = -9e99
        for _mm in moves:
            if _mm.print_time < _pend - 1e-9:
                logging.warning("PHOVERLAP pt=%.6f prev_end=%.6f mt=%.6f sv=%.1f xr=%.3f yr=%.3f "
                                "sx=%.4f sy=%.4f", _mm.print_time, _pend, _mm.move_t, _mm.start_v,
                                _mm.x_r, _mm.y_r, _mm.start_x, _mm.start_y)
            _e = _mm.print_time + _mm.move_t
            if _e > _pend:
                _pend = _e
        # breakpoints: every move boundary shifted back by each pulse time, in (t0,t1)
        bset = set()
        # breakpoints must include every move's START **and END**. A move END is a kink whenever
        # a gap/dwell follows it: `_eval_axis` forces vel=0 in the gap, so the (unshaped) velocity
        # steps at the move end. Missing those ends let a segment span the kink -> vel-fit inexact
        # -> per-segment displacement error -> the multi-mm chain drift (the feature offset). For
        # contiguous moves end==next start (harmless dup in the set); for gap-followed moves this
        # is the previously-missing kink. THIS makes single-interval segments displacement-exact.
        bounds = []
        for m in moves:
            bounds.append(m.print_time)
            bounds.append(m.print_time + m.move_t)
        for b in bounds:
            for ti in all_t:
                bp = b - ti
                if t0 + 1e-9 < bp < t1 - 1e-9:
                    bset.add(bp)
        # cap segment DURATION: a breakpoint every DT_MAX so no segment spans a long non-quadratic
        # region. A 119ms segment carried a 218su vel-fit error = the chain-drift source; at
        # <=DT_MAX the trapezoidal error is negligible. Grid points join the SET so they merge with
        # existing breakpoints (free in dense regions; only fills sparse/slow spans).
        DT_MAX = 0.004
        n = 1
        while t0 + n * DT_MAX < t1 - 1e-9:
            bset.add(t0 + n * DT_MAX)
            n += 1
        pts = [t0] + sorted(bset) + [t1]
        spm = self._steps_per_mm
        slave_oid = self._trapq_slave_oid
        # CoreXY projects both axes; the extruder is x-only (its trapq y_r is the
        # pressure-advance FLAG, not motion) -> zero the Y term + no X flip when not corexy.
        ys = self._trapq_ysign if slave_oid is not None else 0.0
        xs = self._trapq_xsign if slave_oid is not None else 1.0
        sa = self._trapq_sign
        sb = self._trapq_sign_b
        # SEGMENT MERGING: the breakpoint split is exact but over-fine (a split at every
        # move-boundary±pulse, even where the accel doesn't change — cruise, ramp interiors).
        # Greedily coalesce consecutive intervals into ONE segment whose accel is chosen to
        # match the true END velocity (no velocity discontinuity), as long as the position
        # error at EVERY covered breakpoint stays within tol. Equal-accel runs merge with ~0
        # error; the shaper-transition stairs merge with a bounded, re-anchored position step
        # <= tol (each segment's start pos/vel are evaluated fresh from the true trajectory,
        # so errors never accumulate). Cuts segment count + command bandwidth a lot.
        npts = len(pts)
        # cache true (pos,vel) per breakpoint once (npts shaped evals) -> merge is arithmetic
        st = self._build_st_forward(moves, pts, xs, ys)
        # PEAK-BREAKPOINT FIX: the shaped trajectory's reversal apexes (per-motor velocity
        # zero-crossings) fall BETWEEN the move-boundary breakpoints, so the merged quadratic
        # under-reaches the apex -> infill/corners fall SHORT of the wall, worse at speed (the
        # 2026-06-29 phase-step-vs-stepdir finding). Insert a breakpoint EXACTLY at each
        # velocity zero-crossing so every apex is a checked segment endpoint (commanded pos
        # reaches it within tol). Shaped velocity is piecewise-linear between breakpoints
        # (const accel), so the crossing is exact: t = Ta - v0*dt/(v1-v0). Only fires at real
        # reversals -> negligible extra segments (unlike a blanket tight MERGE_TOL).
        zc = []
        for k in range(npts - 1):
            Ta = pts[k]; dtk = pts[k + 1] - Ta
            if dtk <= 1e-9:
                continue
            for mi in (0, 1):                      # motor A('+'), then CoreXY B('-')
                v0 = st[k][mi][1]; v1 = st[k + 1][mi][1]
                if v0 * v1 < 0.0:                  # sign change -> apex inside this interval
                    tzc = Ta + (-v0) * dtk / (v1 - v0)
                    if Ta + 1e-9 < tzc < pts[k + 1] - 1e-9:
                        zc.append(tzc)
        if zc:
            pts = sorted(set(pts) | set(zc))
            npts = len(pts)
            st = self._build_st_forward(moves, pts, xs, ys)
        tol = self._trapq_merge_tol
        if tol < 1e-6:
            tol = 1e-6                          # float-noise floor: equal-accel still merges
        i = 0
        # backpressure: expire segments the MCU has already played (end-clock < current MCU clock)
        now_clock = self.mcu.print_time_to_clock(
            self.mcu.estimated_print_time(self.printer.get_reactor().monotonic()))
        infl = self._inflight
        while infl and infl[0] < now_clock:
            infl.pop(0)
        while i < npts - 1:
            if len(infl) >= self._max_inflight:
                break                          # backpressure: ring near-full -> defer rest to next flush
            Ta = pts[i]
            Ap0 = st[i][0][0]; Av0 = st[i][0][1]
            Bp0 = st[i][1][0]; Bv0 = st[i][1][1]
            # extend [i -> bestj] while the velocity-matched single quadratic holds < tol.
            # (VEL-fit: half_accel matches end-velocity -> the accel is physically correct, so the
            # now+lead look-ahead is correct = no field teleport. A POS-fit forces the exact end
            # position but corrupts the accel -> the lead over-shoots mid-segment = staircase; DON'T.)
            bestj = i + 1
            j = i + 1
            while j < npts and (j - i) <= self._merge_max:
                dte = pts[j] - Ta
                if dte < 1e-9:
                    j += 1
                    continue
                haA = 0.5 * (st[j][0][1] - Av0) / dte
                haB = 0.5 * (st[j][1][1] - Bv0) / dte
                ok = True
                for q in range(i + 1, j + 1):
                    tq = pts[q] - Ta
                    if (abs(Ap0 + (Av0 + haA * tq) * tq - st[q][0][0]) > tol or
                            abs(Bp0 + (Bv0 + haB * tq) * tq - st[q][1][0]) > tol):
                        ok = False
                        break
                # extend if accurate (tol-merge) OR the CURRENT segment [i->bestj] is still under
                # the rate-cap floor (force-merge to reach min_seg_dt; check bestj not the candidate
                # j, else we stop one breakpoint short and the cap is 2x too weak).
                if ok or (pts[bestj] - Ta) < self._min_seg_dt:
                    bestj = j
                    j += 1
                else:
                    break
            Te = pts[bestj]
            dt = Te - Ta
            if dt < 1e-9:
                i = bestj
                continue
            haA = 0.5 * (st[bestj][0][1] - Av0) / dt   # vel-fit: physically-correct accel (lead-safe)
            haB = 0.5 * (st[bestj][1][1] - Bv0) / dt
            # clamp the vel-fit accel: a shaper velocity SPIKE at a feature transition (glitch v
            # exceeding machine max, sub-DT_MAX so uncaptured) fits a huge half_accel -> a large
            # displacement error -> the first-feature startup drift the slew then hauls back (the
            # shudder). Normal shaped |half_accel| << HAMAX so this only tames the anomalies.
            HAMAX = 20000.0
            if haA > HAMAX: haA = HAMAX
            elif haA < -HAMAX: haA = -HAMAX
            if haB > HAMAX: haB = HAMAX
            elif haB < -HAMAX: haB = -HAMAX
            # TELESCOPING CLOCKS (Prusa phase_stepping.cpp:143-144 ticks(end_N)-ticks(end_{N-1})):
            # duration = clock(Te) - clock(Ta) from the SAME rounded absolute endpoints, so
            # start_clock_{N+1} == start_clock_N + dticks_N -> GAPLESS. The old
            # int(seconds_to_clock(dt)) truncated dt independently of the rounded start, leaving a
            # sub-tick GAP at every segment boundary where the MCU clamps de<0->0 and FREEZES the
            # commutation at the segment start, then resumes. That per-boundary freeze-then-resume,
            # x thousands of move boundaries under open-loop load, is a directional micro-slip that
            # cancels on a round-trip but ratchets over a path = the registration-drift root cause.
            c0 = self.mcu.print_time_to_clock(Ta)
            sc = c0 & 0xffffffff
            dticks = int(self.mcu.print_time_to_clock(Te) - c0)
            baseA = Ap0 * spm
            if self._trapq_pos_ref is None:
                self._trapq_pos_ref = baseA    # pos=0 at engage -> angle=MSCNT
            shaped_posA = sa * (baseA - self._trapq_pos_ref)
            # CHAIN (the teleport fix): start at the PREVIOUS segment's exact quadratic endpoint,
            # not the re-evaluated shaped pos. The merge fits half_accel to end-VELOCITY, so the
            # quadratic endpoint drifts from the shaped pos by up to merge_tol -> `posA=shaped`
            # teleported the field by that gap (0.1-0.55mm > 1/2 elec period) at EVERY boundary
            # -> pole slip = the staircase. Chaining is C1-continuous; pos error vs shaped stays
            # bounded by merge_tol (error_k = quad_end_k - shaped_k, doesn't accumulate).
            # SLEW-CORRECTED CHAIN: chain to the prev endpoint (position-continuous, no teleport)
            # but pull toward the INTENDED (shaped) frame by <= SLEW su/segment. The clamp is
            # < 1/2 electrical period (32 su) so a nudge can NEVER slip a pole, yet it CONVERGES
            # the dead-reckoning drift (measured 9mm chainDIV) to ~0 instead of integrating it
            # into the feature-offset. This is the fix for the commanded offset.
            if self._last_end_posA is None:
                posA = shaped_posA
            else:
                dA = shaped_posA - self._last_end_posA
                if dA > self._slew: dA = self._slew
                elif dA < -self._slew: dA = -self._slew
                posA = self._last_end_posA + dA
            vA = sa * Av0 * spm
            hAa = sa * haA * spm               # haA already = half_accel (mm/s^2)
            # re-anchor flag rides the FIRST segment after a halt (both motors together);
            # cleared immediately so the rest of the resume streams normally.
            ra = 1 if self._reanchor_next else 0
            self._reanchor_next = False
            self.seg_cmd.send([self.oid, sc, dticks, self._f2u(posA),
                               self._f2u(vA), self._f2u(hAa), ra])
            if slave_oid is not None:          # CoreXY motor B ('-')
                baseB = Bp0 * spm
                if self._trapq_pos_ref_b is None:
                    self._trapq_pos_ref_b = baseB
                shaped_posB = sb * (baseB - self._trapq_pos_ref_b)
                if self._last_end_posB is None:
                    posB = shaped_posB
                else:
                    dB = shaped_posB - self._last_end_posB
                    if dB > self._slew: dB = self._slew
                    elif dB < -self._slew: dB = -self._slew
                    posB = self._last_end_posB + dB
                vB = sb * Bv0 * spm
                hBa = sb * haB * spm
                self.seg_cmd.send([slave_oid, sc, dticks, self._f2u(posB),
                                   self._f2u(vB), self._f2u(hBa), ra])
            # T2: track how far the chain has drifted from the intended (shaped) frame.
            if abs(shaped_posA - posA) > abs(self._maxdivA):
                self._maxdivA = shaped_posA - posA
            if slave_oid is not None and abs(shaped_posB - posB) > abs(self._maxdivB):
                self._maxdivB = shaped_posB - posB
            # per-segment vel-fit displacement error (should be ~0 for a single exact quadratic)
            segerr = (vA * dt + hAa * dt * dt) - sa * (st[bestj][0][0] - Ap0) * spm
            if abs(segerr) > abs(self._maxsegerr):
                self._maxsegerr = segerr
                self._worst = (bestj - i, dt, vA)
                if abs(segerr) > 50.:   # GLITCH capture (rare, on new worst only): log the anatomy
                    # to localize the missed crossing. mid = shaped pos halfway between i and bestj:
                    # if it sits BETWEEN st[i] and st[bestj] the eval is (impossibly) smooth -> a bad
                    # endpoint value; if it SPIKES past both, an uncaptured crossing sits between the
                    # two breakpoints (pts incomplete). vel scale tells pos-jump vs vel-spike.
                    smid = self._shaped_motor(moves, 0.5 * (Ta + Te), xs, ys)
                    logging.warning(
                        "PHGLITCH segERR=%.1f dt=%.6f span=%d Ta=%.5f | A i=(p%.4f v%.1f) "
                        "mid=p%.4f j=(p%.4f v%.1f) | B i=(p%.4f v%.1f) mid=p%.4f j=(p%.4f v%.1f)",
                        segerr, dt, bestj - i, Ta,
                        st[i][0][0], st[i][0][1], smid[0][0], st[bestj][0][0], st[bestj][0][1],
                        st[i][1][0], st[i][1][1], smid[1][0], st[bestj][1][0], st[bestj][1][1])
            # advance the chain: next segment starts at THIS segment's exact quadratic endpoint.
            self._last_end_posA = posA + (vA + hAa * dt) * dt
            if slave_oid is not None:
                self._last_end_posB = posB + (vB + hBa * dt) * dt
            self._trapq_net += (vA + hAa * dt) * dt
            self._trapq_nseg += 1
            infl.append(self.mcu.print_time_to_clock(Te))   # backpressure: track this segment in-flight
            if self._trapq_debug:
                est_now = self.mcu.estimated_print_time(
                    self.printer.get_reactor().monotonic())
                extra = ("" if slave_oid is None
                         else " | B: pos=%.1f v=%.1f ha=%.1f" % (posB, vB, hBa))
                logging.info("PHSEG #%d Ta=%.4f dt=%.5f merged=%d | A: pos=%.1f "
                             "v=%.1f ha=%.1f%s | netA=%.1f (%.4fmm)"
                             % (self._trapq_nseg, Ta, dt, bestj - i,
                                posA, vA, hAa, extra, self._trapq_net,
                                self._trapq_net / spm))
            i = bestj
        # cache the flush-horizon position (motor-frame, sent units) so the NEXT flush, if
        # idle (cnt==0), can hold here instead of coasting. st[-1] is the shaped pos at t1.
        if i >= npts - 1:
            # window fully streamed: advance the idle-hold cache + horizon to t1
            if self._trapq_pos_ref is not None:
                self._trapq_last_posA = sa * (st[-1][0][0] * spm - self._trapq_pos_ref)
                if slave_oid is not None and self._trapq_pos_ref_b is not None:
                    self._trapq_last_posB = sb * (st[-1][1][0] * spm - self._trapq_pos_ref_b)
            self._trapq_last_pt = t1
        else:
            # backpressure broke early -> resume at pts[i] next flush; leave the cache (not idle)
            self._trapq_last_pt = pts[i]

    def _spi_hs_pins(self):
        # Resolve this driver's SPI SCK + MOSI pins (the ones that clock data out).
        # The MCU exposes BUS_PINS_<bus> as "MISO,MOSI,SCK" (e.g. "PC11,PC12,PC10").
        # Return pin NAMES, NOT ints: phase_exec_pin_hs's "pin" param is auto-bound to
        # the firmware 'pin' enumeration, so msgproto maps name->value on encode. Passing
        # the pre-encoded int (42 for PC10) raised "Unknown value '42' in enumeration
        # 'pin'" -- the enum keys are names, not values. Only hit when spi_div<3 (the
        # overclocked-SPI hs path), so it survived earlier validation. [fix 2026-06-28]
        consts = self.mcu.get_constants()
        names = consts['BUS_PINS_%s' % (self._spi_bus_name,)].split(',')
        miso, mosi, sck = names[0], names[1], names[2]
        return [sck, mosi]

    def cmd_TRAPQ_ENGAGE(self, gcmd):
        # refuse a second engage. A double PHASE_STEP_ON (or CALIBRATE_PHASE_COGGING mid
        # phase-stepped print) would re-run the TMC-suppress / step-gen-detach / MSCNT-seed with
        # the driver already in direct_mode under the monkeypatch -> lost real TMC registers, dead
        # X/Y itersolve, corrupted phase origin. Disengage (PHASE_STEP_OFF) before re-engaging.
        if getattr(self, '_trapq_engaged', False):
            raise gcmd.error("phase-stepping already ENGAGED -- PHASE_STEP_OFF before re-engaging")
        rate = gcmd.get_float('RATE', 8000., minval=100., maxval=60000.)
        amp = gcmd.get_int('AMP', 255, minval=0, maxval=255)   # full amp (torque)
        swap = gcmd.get_int('SWAP', 1)
        cur = gcmd.get_float('CURRENT', 0.5, minval=0.05, maxval=1.2)
        sign = gcmd.get_int('SIGN', 1)         # flip if motor A goes backward
        with_name = gcmd.get('WITH', None)     # CoreXY: also drive the '-' motor (x-y)
        if self.seg_cmd is None:
            raise gcmd.error("phase_exec %s: not DMA-configured / no seg cmd"
                             % (self.stepper_name,))
        corexy = with_name is not None
        slave = swap2 = sign2 = None
        if corexy:
            slave = self.printer.lookup_object('phase_exec %s' % (with_name,), None)
            if slave is None:
                raise gcmd.error("no [phase_exec %s] for WITH=" % (with_name,))
            swap2 = gcmd.get_int('SWAP2', swap)
            sign2 = gcmd.get_int('SIGN2', sign)
        self._trapq_setup(corexy)
        if self._trapq is None:
            raise gcmd.error("no trapq for %s" % (self.stepper_name,))
        gca = self.printer.lookup_object('gcode')
        gca.run_script_from_command(
            "SET_STEPPER_ENABLE STEPPER=%s ENABLE=1" % (self.stepper_name,))
        gca.run_script_from_command(
            "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f HOLDCURRENT=%.3f"
            % (self.stepper_name, cur, cur))
        if corexy:
            gca.run_script_from_command(
                "SET_TMC_CURRENT STEPPER=%s CURRENT=%.3f HOLDCURRENT=%.3f"
                % (slave.stepper_name, cur, cur))
        # auto-log the effective phase-step current (torque-margin lever for open-loop slip)
        try:
            gcmd.respond_info(self._current_info_str())
            if corexy:
                gcmd.respond_info(slave._current_info_str())
        except Exception:
            pass
        est = self.mcu.estimated_print_time(
            self.printer.get_reactor().monotonic())
        clock0 = self.mcu.print_time_to_clock(est + 0.25)
        self._trapq_sign = float(sign)
        self._trapq_sign_b = float(sign2) if corexy else 1.0
        self._trapq_ysign = float(gcmd.get_int('YSIGN', 1))  # -1 flips CoreXY Y
        self._trapq_xsign = float(gcmd.get_int('XSIGN', 1))  # -1 flips CoreXY X
        step_suppress = corexy and bool(gcmd.get_int('STEP_SUPPRESS', 1))  # step-gen suppression
        self._trapq_merge_tol = gcmd.get_float('MERGE_TOL', 0.001, minval=0.)
        # P5+: override the shared TMC SPI prescaler (CR1 BR) for the run. SPI clk =
        # 42MHz / 2^(div+1): 0=21MHz(Prusa) 1=10.5 2=5.25 3=2.625(Klipper 4MHz->/16 floor).
        # A faster SPI shrinks each XDIRECT frame -> higher refresh rate before skips.
        spi_div = gcmd.get_int('SPI_DIV', 3, minval=0, maxval=7)
        # CS-latch slack (busy-loop iters after BSY=0, before CS-high) for overclocked SPI
        cs_slack = gcmd.get_int('CS_SLACK', 50, minval=0, maxval=5000)  # 0 was marginal CS-latch at 5.25MHz
        # fold in input shaping (CoreXY only; IS=0 disables for A/B ringing tests)
        self._load_shaper_pulses(corexy and bool(gcmd.get_int('IS', 1)))
        self._trapq_debug = bool(gcmd.get_int('DEBUG', 0))
        self._trapq_pos_ref = self._trapq_pos_ref_b = None
        self._inflight = []                    # backpressure: fresh engage -> ring cleared -> no in-flight
        self._last_end_posA = self._last_end_posB = None   # host teleport logger reset
        self._maxdivA = self._maxdivB = 0.0                # T2 divergence tracker reset
        self._maxsegerr = 0.0; self._worst = (0, 0.0, 0.0)
        self._trapq_last_posA = self._trapq_last_posB = None   # no hold until first seg
        self._reanchor_next = False    # engage seeds the origin from MSCNT; no re-anchor yet
        self._trapq_last_pt = est              # only stream moves after engage
        self._trapq_net = 0.0
        self._trapq_nseg = 0
        self._trapq_slave_oid = slave.oid if corexy else None
        # seamless engage (reads MSCNT, preloads XDIRECT, flips direct_mode) on BOTH
        # motors BEFORE suppressing TMC checks; analytic ON before start.
        mscnt = self._engage_motor(self.tmc, amp, swap, 0)
        mscnt2 = self._engage_motor(slave.tmc, amp, swap2, 0) if corexy else 0
        self._set_tmc_checks(False)
        # bus is now exclusively ours -> crank the SPI prescaler (one SPI3, both motors)
        # First force the SCK/MOSI pins to very-high slew (OSPEEDR=0x03): Klipper sets
        # them to "high" (0x02) which rolls off the edges >~5-10MHz on the TMC traces and
        # corrupts XDIRECT at the overclocked rate. Prusa uses very-high -> 21MHz clean.
        if spi_div < 3 and self.pin_hs_cmd is not None:
            for pin in self._spi_hs_pins():     # SCK + MOSI of this driver's bus
                self.pin_hs_cmd.send([pin, 6])  # SPI3 alternate function = AF6 on F4
        if self.spi_div_cmd is not None:
            self.spi_div_cmd.send([self.oid, spi_div])
        if self.cs_slack_cmd is not None:      # CS-latch hold for the overclocked SPI
            self.cs_slack_cmd.send([cs_slack])
        self.analytic_cmd.send([self.oid, 1])
        if corexy:
            self.analytic_cmd.send([slave.oid, 1])
        self._send_corrections()
        if corexy:
            slave._send_corrections()
        self._send_blend(slave if corexy else None)   # fwd<->bwd blend band
        interval = max(1, int(self.mcu.seconds_to_clock(1.0 / rate)))
        self._interval = interval              # cache for SET_PHASE_LEAD fraction math
        self._engage_amp = amp                 # cache for SET_PHASE_IDLE + clean disengage re-sync
        self._engage_swap = swap
        self._engage_rate = rate
        if corexy:
            slave._interval = interval
            slave._engage_amp = amp
            slave._engage_swap = swap2
            slave._engage_rate = rate
            # chain master -> slave -> end; slave first (no timer), then master (timer)
            self.chain_cmd.send([self.oid, slave.oid])
            self.chain_cmd.send([slave.oid, 0])
            self.start_cmd.send([slave.oid, interval, 0, 0, amp, swap2,
                                 clock0 & 0xffffffff, 0, mscnt2])
            self.start_cmd.send([self.oid, interval, 0, 0, amp, swap,
                                 clock0 & 0xffffffff, 1, mscnt])
        else:
            self.chain_cmd.send([self.oid, 0])
            self.start_cmd.send([self.oid, interval, 0, 0, amp, swap,
                                 clock0 & 0xffffffff, 1, mscnt])
        # Velocity commutation lead (re-sent after start_cmd, which reset it to interval).
        # Honors a non-default _lead_frac set via SET_PHASE_LEAD before the engage.
        self._send_lead(slave if corexy else None)
        self._send_idle(slave if corexy else None)   # idle/hold current (config idle_hold_pct)
        # register the streamer LAST, once engage is fully set up
        self.printer.lookup_object('motion_queuing').register_flush_callback(
            self._trapq_flush_cb)
        self._trapq_engaged = True
        self._group_slave = slave
        # suppress X/Y step generation (the streamer drives the motors now). Detach
        # AFTER the flush cb is live + the toolhead is idle at engage (no pending X/Y
        # moves to lose). Restored in DISENGAGE/STOP/shutdown.
        if step_suppress:
            self._suppress_step_gen([self.mcu_stepper, slave.mcu_stepper], True)
        self._step_suppressed = step_suppress
        if self._shape:
            shp = (" IS=on margin=%.1fms px=%d py=%d"
                   % (self._pulse_margin * 1e3, len(self._px[0]), len(self._py[0])))
        else:
            shp = " IS=off"
        shp += " Xstep=%s merge=%.3fmm spi=%.2fMHz csslack=%d" % (
            "SUPPRESSED" if step_suppress else "on", self._trapq_merge_tol,
            42.0 / (1 << (spi_div + 1)), cs_slack)
        gcmd.respond_info(
            "[phase_exec %s] trapq-streaming ENGAGED @%dHz amp=%d%s%s. Moves now drive "
            "the %s analytically from the live trapq. PHASE_TRAPQ_DISENGAGE when done."
            % (self.stepper_name, rate, amp,
               (" + WITH=%s (CoreXY x-y)" % with_name) if corexy else
               " (extruder, heat first)", shp,
               "CoreXY gantry" if corexy else "rotor"))

    def cmd_TRAPQ_DISENGAGE(self, gcmd):
        self._trapq_engaged = False
        try:
            self.printer.lookup_object('motion_queuing'
                                       ).unregister_flush_callback(
                                           self._trapq_flush_cb)
        except Exception:
            pass
        slave = getattr(self, '_group_slave', None)
        if self.seg_query_cmd is not None:
            p = self.seg_query_cmd.send([self.oid])
            net_mm = (self._trapq_net / self._steps_per_mm
                      if self._steps_per_mm else 0.)
            # snap = feed-position discontinuity at each ring refill, milli-step-units;
            # /200000 = mm (200 step-units/mm). Big snapsum -> the slip is the feed/refill
            # path; ~0 -> commutation. The localizer for the per-layer staircase.
            gcmd.respond_info("[phase_exec %s] streamed %d segs, net %.4fmm | "
                              "final seg depth=%d overflow=%d dry=%d minrep=%d | "
                              "refills=%d snapSUM=%.3fmm snapMAX=%.3fmm | "
                              "settleEXC=%.3fmm maxCOAST=%.3fmm | "
                              "JUMP=%d jumpMAX=%.3fmm skipSame=%d | "
                              "T2 chainDIV A=%.3fmm B=%.3fmm | segERR=%.1fsu worst(span=%d dt=%.5f v=%.0f)"
                              % (self.stepper_name, self._trapq_nseg, net_mm,
                                 p['depth'], p['overflow'], p['dry'],
                                 p.get('minrep', -1), p.get('refills', -1),
                                 p.get('snapsum', 0) / 200000.,
                                 p.get('snapmax', 0) / 200000.,
                                 p.get('settle', 0) / 200000.,
                                 p.get('coast', 0) / 200000.,
                                 p.get('jump', -1), p.get('jumpmax', 0) / 200000.,
                                 p.get('skipsame', -1),
                                 self._maxdivA / 200., self._maxdivB / 200.,
                                 self._maxsegerr, self._worst[0], self._worst[1],
                                 self._worst[2]))
            # SLAVE (the +Y-drift motor on CoreXY) seg health -- a detached integrator that
            # drops (overflow) or starves (dry) free-runs -> directional position ratchet.
            if slave is not None and slave.seg_query_cmd is not None:
                ps = slave.seg_query_cmd.send([slave.oid])
                warn = " <-- DROP/STARVE/TELEPORT: position ratchet risk" if (
                    ps['overflow'] or ps['dry'] or ps.get('jump', 0)) else ""
                gcmd.respond_info("[phase_exec %s] (slave) seg depth=%d overflow=%d "
                                  "dry=%d minrep=%d | refills=%d snapSUM=%.3fmm "
                                  "snapMAX=%.3fmm | settleEXC=%.3fmm maxCOAST=%.3fmm | "
                                  "JUMP=%d jumpMAX=%.3fmm skipSame=%d%s"
                                  % (slave.stepper_name, ps['depth'], ps['overflow'],
                                     ps['dry'], ps.get('minrep', -1), ps.get('refills', -1),
                                     ps.get('snapsum', 0) / 200000.,
                                     ps.get('snapmax', 0) / 200000.,
                                     ps.get('settle', 0) / 200000.,
                                     ps.get('coast', 0) / 200000.,
                                     ps.get('jump', -1), ps.get('jumpmax', 0) / 200000.,
                                     ps.get('skipsame', -1), warn))
        self.stop_cmd.send([self.oid])
        if slave is not None:
            self.stop_cmd.send([slave.oid])        # stop the chained '-' motor too
        if self.analytic_cmd is not None:
            self.analytic_cmd.send([self.oid, 0])
            if slave is not None:
                self.analytic_cmd.send([slave.oid, 0])
        self._set_tmc_checks(True)
        # Re-sync each rotor to its frozen MSCNT BEFORE releasing direct_mode -> the sequencer
        # resume doesn't snap +-1/2 period (the intermittent MSCNT_HOME "phase off 512" failure).
        m1 = self._disengage_motor(self.tmc, getattr(self, '_engage_amp', 255),
                                   getattr(self, '_engage_swap', 1))
        m2 = self._disengage_motor(slave.tmc, getattr(slave, '_engage_amp', 255),
                                   getattr(slave, '_engage_swap', 1)) \
            if slave is not None else -1
        gca = self.printer.lookup_object('gcode')
        gca.run_script_from_command(
            "SET_TMC_FIELD STEPPER=%s FIELD=direct_mode VALUE=0"
            % (self.stepper_name,))
        if slave is not None:
            gca.run_script_from_command(
                "SET_TMC_FIELD STEPPER=%s FIELD=direct_mode VALUE=0"
                % (slave.stepper_name,))
        # re-attach the X/Y trapq + resync MCU step pos (G28 next fully resyncs)
        self._suppress_step_gen(None, False)
        self._group_slave = None
        gcmd.respond_info("[phase_exec %s] trapq-streaming DISENGAGED; direct_mode cleared "
                          "on %s (rotor re-synced to MSCNT %d/%d pre-release). "
                          "RE-HOME (G28) before normal XY moves."
                          % (self.stepper_name,
                             "both CoreXY motors" if slave else "the motor", m1, m2))

    def cmd_STOP(self, gcmd):
        self.stop_cmd.send([self.oid])
        if self.analytic_cmd is not None:
            self.analytic_cmd.send([self.oid, 0])  # leave analytic mode (harmless if off)
        if getattr(self, '_trapq_engaged', False):
            self._trapq_engaged = False
            try:
                self.printer.lookup_object('motion_queuing'
                                           ).unregister_flush_callback(
                                               self._trapq_flush_cb)
            except Exception:
                pass
        slave = getattr(self, '_group_slave', None)
        if slave is not None:
            self.stop_cmd.send([slave.oid])        # stop the chained motor too
            self._group_slave = None
        self._set_tmc_checks(True)         # resume Klipper's TMC polling
        self._suppress_step_gen(None, False)   # re-attach X/Y trapq if suppressed
        if getattr(self, '_pause_lc', False):
            self._set_bulk_sensors(False)  # resume the HX717 bulk reader (if paused)
            self._pause_lc = False
        gcmd.respond_info("[phase_exec %s] stopped (last current vector held; "
                          "clear direct_mode + re-home)." % (self.stepper_name,))

    def cmd_TRAJECTORY(self, gcmd):
        enable = gcmd.get_int('ENABLE', 1)
        if self.mcu_stepper is None:
            raise gcmd.error("no MCU_stepper bound for %s" % (self.stepper_name,))
        self.traj_cmd.send([self.oid, enable])
        gcmd.respond_info("[phase_exec %s] trajectory mode %s — angle now follows the "
                          "live commanded position." % (self.stepper_name,
                                                        "ON" if enable else "OFF"))

    def cmd_STATS(self, gcmd):
        if not self.use_dma or self.query_cmd is None:
            raise gcmd.error("no DMA play-out on %s" % (self.stepper_name,))
        p = self.query_cmd.send([self.dma_oid])
        gcmd.respond_info(
            "[phase_dma %s] tx=%d ovr=%d dmaerr=%d maxrx=%d skips=%d"
            % (self.stepper_name, p['tx'], p['ovr'], p['dmaerr'],
               p['maxrx'], p['skips']))

    def _send_corrections(self):
        # Push both direction spectra to the MCU struct. Idempotent; called at the
        # start of every run (the MCU struct is volatile across a reset). dir 0=fwd,
        # 1=bwd. Sending mag=0 for a bwd harmonic clears it (so a removed entry resets).
        if self.corr_cmd is None:
            return
        for h, (mag, pha) in self.corrections.items():
            self.corr_cmd.send([self.oid, 0, h, mag, pha])
        for h, (mag, pha) in self.corr_bwd.items():
            self.corr_cmd.send([self.oid, 1, h, mag, pha])

    def _blend_steps_per_mm(self):
        # Robust steps/mm (mm->step-units): prefer the live stepper, fall back to whatever
        # _trapq_setup cached. Used to convert the mm/s blend band to MCU step-units/s.
        if self.mcu_stepper is not None:
            try:
                return 1.0 / self.mcu_stepper.get_step_dist()
            except Exception:
                pass
        return self._steps_per_mm

    def _send_blend(self, slave=None):
        # Push the fwd<->bwd blend band (step-units/s) for self + the CoreXY slave. Both
        # share the master's steps/mm (the slave's segments are scaled by it). Idempotent;
        # co-sent with the corrections at every run start (MCU struct is volatile on reset).
        spm = self._blend_steps_per_mm()
        for ex in (self, slave):
            if ex is not None and ex.blend_cmd is not None:
                cnt = float(ex.blend_vband_mmps) * spm
                ex.blend_cmd.send([ex.oid, self._f2u(cnt)])

    def set_correction(self, h, mag, pha, direction=None):
        # Programmatic setter (used by the cogging calibration). Updates the stored
        # spectrum AND pushes it live. mag=0 disables. direction None = both fwd+bwd
        # (symmetric, the default for existing callers); 0 = forward, 1 = backward.
        mag = int(mag); pha = int(pha) % PHASE_UNITS
        dirs = (0, 1) if direction is None else (int(direction) & 1,)
        for d in dirs:
            store = self.corrections if d == 0 else self.corr_bwd
            if mag:
                store[h] = (mag, pha)
            else:
                store.pop(h, None)
            if self.corr_cmd is not None:
                self.corr_cmd.send([self.oid, d, h, mag, pha])

    def get_corrections(self, direction=0):
        return dict(self.corrections if int(direction) & 1 == 0 else self.corr_bwd)

    def sweep(self, harm, start_clock, dur_per_1024, pha_s, pha_d, mag_s, mag_d):
        # Arm an MCU-side calibration sweep: harmonic `harm`'s {pha,mag} ramps from
        # (pha_s,mag_s) to (pha_s+pha_d, mag_s+mag_d) across [start_clock, +1024*dur].
        if self.sweep_cmd is not None:
            self.sweep_cmd.send([self.oid, 1, int(harm),
                                 int(start_clock) & 0xffffffff, max(1, int(dur_per_1024)),
                                 int(pha_s), int(pha_d), int(mag_s), int(mag_d)])

    def sweep_stop(self, harm):
        if self.sweep_cmd is not None:
            self.sweep_cmd.send([self.oid, 0, int(harm), 0, 1, 0, 0, 0, 0])
        self.corrections.pop(int(harm), None)
        self.corr_bwd.pop(int(harm), None)

    def osc(self, amp, inc):
        # hum the held angle in place at a fixed freq (inc = freq*1024/rate per tick)
        if self.osc_cmd is not None:
            self.osc_cmd.send([self.oid, 1, int(amp), int(inc)])

    def osc_stop(self):
        if self.osc_cmd is not None:
            self.osc_cmd.send([self.oid, 0, 0, 0])

    def cmd_SET_CORRECTION(self, gcmd):
        h = gcmd.get_int('HARMONIC', minval=1, maxval=PE_MAX_HARM)
        mag = gcmd.get_int('MAG', 0, minval=-2000, maxval=2000)
        pha = gcmd.get_int('PHA', 0) % PHASE_UNITS
        # DIR: -1 (default) = both directions; 0 = forward only; 1 = backward only
        d = gcmd.get_int('DIR', -1, minval=-1, maxval=1)
        self.set_correction(h, mag, pha, direction=(None if d < 0 else d))
        gcmd.respond_info(
            "[phase_exec %s] harmonic %d %s: mag=%d pha=%d (fwd=%d bwd=%d active)"
            % (self.stepper_name, h,
               ('both' if d < 0 else ('fwd' if d == 0 else 'bwd')),
               mag, pha, len(self.corrections), len(self.corr_bwd)))

    def cmd_SET_BLEND(self, gcmd):
        # Live-tune the fwd<->bwd blend band (mm/s) for THIS executor. Takes effect at the
        # next run start (re-sent with the corrections); also pushed immediately if the MCU
        # struct is live. VBAND=0 = legacy hard switch.
        self.blend_vband_mmps = gcmd.get_float('VBAND', self.blend_vband_mmps, minval=0.)
        if self.blend_cmd is not None and self.oid is not None:
            cnt = float(self.blend_vband_mmps) * self._blend_steps_per_mm()
            self.blend_cmd.send([self.oid, self._f2u(cnt)])
        gcmd.respond_info("[phase_exec %s] blend band = %.1f mm/s"
                          % (self.stepper_name, self.blend_vband_mmps))

    def _send_lead(self, slave):
        # Push the velocity-lead (ticks = interval * _lead_frac) to master (+slave).
        if self.lead_cmd is None or not self._interval:
            return
        ticks = max(0, int(round(self._interval * self._lead_frac)))
        self.lead_cmd.send([self.oid, ticks])
        if slave is not None and slave.lead_cmd is not None:
            slave._lead_frac = self._lead_frac
            base = (slave._interval or self._interval) * self._lead_frac
            # DMA write-skew comp: the slave's XDIRECT lands ~1 SPI frame after the master's
            # (chained via the master's TC IRQ), so its field is applied late -> effectively
            # less lead. Add extra lead to re-sync. NOTE: analytically this skew produces a
            # velocity-proportional Y offset (+11us*vel_B/2) that CANCELS over balanced up/down
            # infill lines, so it may NOT move the accumulating +Y drift -- this knob is to
            # TEST that empirically. Default 0 = current behavior.
            try:
                extra = self.mcu.seconds_to_clock(self._slave_lead_extra_us * 1e-6)
            except Exception:
                extra = 0
            slave.lead_cmd.send([slave.oid, max(0, int(round(base + extra)))])

    def cmd_SET_LEAD(self, gcmd):
        # Velocity commutation lead as a FRACTION of the refresh period (Prusa = 1.0,
        # phase_stepping.cpp:796). 0 = legacy no-lead (reproduces the speed-dependent slip).
        # Takes effect immediately if engaged; persists to the next engage. The CoreXY slave
        # is driven together (it shares the master's PHASE_STEP_ON).
        self._lead_frac = gcmd.get_float('FRAC', self._lead_frac, minval=0., maxval=4.)
        self._slave_lead_extra_us = gcmd.get_float(
            'SLAVE_EXTRA_US', self._slave_lead_extra_us, minval=-50., maxval=50.)
        slave = self.printer.lookup_object(
            'phase_exec %s' % (self._group_slave.stepper_name,), None) \
            if self._group_slave is not None else None
        if self.lead_cmd is not None and self.oid is not None and self._interval:
            self._send_lead(slave)
        ticks = int(round(self._interval * self._lead_frac)) if self._interval else 0
        gcmd.respond_info("[phase_exec %s] velocity lead = %.2f x refresh (%d ticks)"
                          " | slave extra = %.1f us (write-skew comp)"
                          % (self.stepper_name, self._lead_frac, ticks,
                             self._slave_lead_extra_us))

    def _send_idle(self, slave):
        # Push idle/hold current to master (+slave). hold = idle_hold_pct% of the engage amp;
        # thresh = idle_ms in per-motor refresh ticks. Round-robin refreshes each motor at rate/2
        # (one motor per TIM8 tick); a solo motor at full rate. pct>=100 -> hold=full, thresh=0.
        if self.idle_cmd is None:
            return
        amp = getattr(self, '_engage_amp', None)
        rate = getattr(self, '_engage_rate', None)
        if amp is None or rate is None:
            return
        per_motor_hz = rate / (2.0 if slave is not None else 1.0)
        if self.idle_hold_pct >= 100:
            hold, thresh = int(amp), 0
        else:
            hold = max(1, int(round(amp * self.idle_hold_pct / 100.)))
            thresh = max(1, int(round(self.idle_ms / 1000. * per_motor_hz)))
        self.idle_cmd.send([self.oid, hold, thresh])
        if slave is not None and slave.idle_cmd is not None:
            slave.idle_cmd.send([slave.oid, hold, thresh])

    def cmd_SET_IDLE(self, gcmd):
        # Idle/hold current: HOLD = % of full current held at standstill (100 = off = Prusa
        # behavior); IDLE_MS = stillness delay before ramping down. Live if engaged; persists to
        # the next engage. CoreXY slave driven together (shares the master's PHASE_STEP_ON).
        self.idle_hold_pct = gcmd.get_int('HOLD', self.idle_hold_pct, minval=0, maxval=100)
        self.idle_ms = gcmd.get_float('IDLE_MS', self.idle_ms, minval=1.)
        slave = self.printer.lookup_object(
            'phase_exec %s' % (self._group_slave.stepper_name,), None) \
            if self._group_slave is not None else None
        self._send_idle(slave)
        gcmd.respond_info("[phase_exec %s] idle current: hold=%d%% of full, delay=%.0fms%s"
                          % (self.stepper_name, self.idle_hold_pct, self.idle_ms,
                             "  (DISABLED)" if self.idle_hold_pct >= 100 else ""))

    def cmd_SET_COGGING(self, gcmd):
        # Clean A/B instrument: ENABLE=0 zeroes every cogging harmonic (both dirs) on the
        # MCU while stashing the live spectra; ENABLE=1 restores them exactly. Lets a single
        # print compare cogging-OFF vs symmetric vs bidirectional with no config churn. The
        # baseline ~0.25mm pad drift survives with SYMMETRIC LUTs (where cogging round-trips
        # cancel); cogging-OFF isolates whether ANY correction is implicated vs the position
        # path / physical reversal slip.
        enable = gcmd.get_int('ENABLE', 1, minval=0, maxval=1)
        if not enable:
            if self._cogging_stash is None:
                # stash the current fwd+bwd spectra, then zero each harmonic on the MCU
                self._cogging_stash = (dict(self.corrections), dict(self.corr_bwd))
                for h in range(1, PE_MAX_HARM + 1):
                    if self.corr_cmd is not None:
                        self.corr_cmd.send([self.oid, 0, h, 0, 0])
                        self.corr_cmd.send([self.oid, 1, h, 0, 0])
                self.corrections = {}
                self.corr_bwd = {}
            gcmd.respond_info("[phase_exec %s] cogging OFF (stashed %s)"
                              % (self.stepper_name,
                                 "live" if self._cogging_stash else "already-off"))
        else:
            if self._cogging_stash is not None:
                fwd, bwd = self._cogging_stash
                for h in range(1, PE_MAX_HARM + 1):
                    m, p = fwd.get(h, (0, 0))
                    self.set_correction(h, m, p, direction=0)
                    m, p = bwd.get(h, (0, 0))
                    self.set_correction(h, m, p, direction=1)
                self._cogging_stash = None
            gcmd.respond_info("[phase_exec %s] cogging ON (fwd=%d bwd=%d active)"
                              % (self.stepper_name,
                                 len(self.corrections), len(self.corr_bwd)))


def load_config_prefix(config):
    return PhaseExec(config)
