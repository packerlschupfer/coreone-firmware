# SG_PROFILE AXIS=<X|Y> [DIST=40] [SPEED=25]
#
# Diagnostic: move one axis in the SAFE (away-from-endstop) direction while
# sampling the TMC2130 StallGuard sg_result (from DRV_STATUS), then report
# min/max/mean/stddev. Use it to compare the StallGuard signal quality between
# axes -- e.g. to find WHY Y false-trips on homing where X doesn't (weaker
# signal => low mean; mechanical noise => high stddev; a tight spot => a dip).
# The axis must already be homed. Run X and Y back to back for an apples-to-
# apples comparison; optionally with the display on vs off.
import math

class SGProfile:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SG_PROFILE', self.cmd_SG_PROFILE,
                                    desc="Sample StallGuard sg_result during a "
                                         "slow move (axis must be homed)")
        self._samples = []
        self._timer = None
        self._tmc = None
        self._fields = None

    def _sample(self, eventtime):
        try:
            status = self._tmc.mcu_tmc.get_register_raw("DRV_STATUS")
            sg = self._fields.get_field("sg_result", status["data"])
            cs = self._fields.get_field("cs_actual", status["data"])
            self._samples.append((sg, cs))
        except Exception:
            pass
        return eventtime + 0.003

    def cmd_SG_PROFILE(self, gcmd):
        axis = gcmd.get('AXIS').upper()
        if axis not in ('X', 'Y'):
            raise gcmd.error("AXIS must be X or Y")
        dist = gcmd.get_float('DIST', 40., above=1.)
        speed = gcmd.get_float('SPEED', 25., above=0.)
        name = 'stepper_' + axis.lower()
        self._tmc = self.printer.lookup_object('tmc2130 ' + name)
        self._fields = self._tmc.mcu_tmc.get_fields()
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        ts = toolhead.get_status(eventtime)
        if axis.lower() not in ts['homed_axes']:
            raise gcmd.error("home %s first" % (axis,))
        ai = 0 if axis == 'X' else 1
        pos = toolhead.get_position()[ai]
        amin = ts['axis_minimum'][ai]
        amax = ts['axis_maximum'][ai]
        # safe direction = toward bed centre (away from the frame just homed to)
        sign = 1.0 if abs(pos - amin) < abs(pos - amax) else -1.0
        reactor = self.printer.get_reactor()
        self._samples = []
        self._timer = reactor.register_timer(self._sample, reactor.NOW)
        try:
            self.gcode.run_script_from_command(
                "G91\nG1 %s%.2f F%d\nG90"
                % (axis, sign * dist, int(speed * 60)))
            toolhead.wait_moves()
        finally:
            reactor.unregister_timer(self._timer)
            self._timer = None
        sgs = [s for s, c in self._samples]
        css = [c for s, c in self._samples]
        if not sgs:
            raise gcmd.error("SG_PROFILE %s: no samples" % (axis,))
        n = len(sgs)
        mean = sum(sgs) / n
        sd = math.sqrt(sum((x - mean) ** 2 for x in sgs) / n)
        # count "dips" near a stall (sg_result<=10) -- the false-trip candidates
        low = sum(1 for x in sgs if x <= 10)
        gcmd.respond_info(
            "SG_PROFILE %s: n=%d sg_result min=%d max=%d mean=%.1f stddev=%.1f"
            " low(<=10)=%d  cs_actual~%d  (%.0fmm @ %.0fmm/s)"
            % (axis, n, min(sgs), max(sgs), mean, sd, low,
               (sum(css) // len(css)) if css else -1, dist, speed))


def load_config(config):
    return SGProfile(config)
