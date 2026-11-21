# Mesh-progress on the LCD: show "Mesh N/total" while BED_MESH_CALIBRATE probes.
#
# bed_mesh probes the grid through the generic probe session, which emits a
# "probe:update_results" event after each averaged point. Our load_cell_probe wires
# its tap session through probe.SampleAveragingHelper (probe.py), which is the class
# that fires that event -- so the hook works for the loadcell probe too. The same
# event also fires for G28 Z and PROBE_ACCURACY, so we wrap BED_MESH_CALIBRATE and
# only update the display while a real mesh is running.
#
# Enable with an empty section:  [mesh_progress]


class MeshProgress:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.gcode = self.printer.lookup_object('gcode')
        self.active = False
        self.total = 0
        self.count = 0
        self.last_xy = None
        self.prev_cmd = None
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.printer.register_event_handler("probe:update_results", self._on_results)

    def _connect(self):
        # best-effort total point count from the [bed_mesh] probe counts
        try:
            bm = self.config.getsection('bed_mesh')
            pc = bm.get('probe_count', None) or bm.get('round_probe_count', None)
            if pc:
                nums = [int(x) for x in pc.replace(',', ' ').split()]
                self.total = nums[0] * (nums[1] if len(nums) > 1 else nums[0])
        except Exception:
            self.total = 0
        # wrap BED_MESH_CALIBRATE so progress shows only during an actual mesh
        try:
            self.prev_cmd = self.gcode.register_command("BED_MESH_CALIBRATE", None)
            if self.prev_cmd is not None:
                self.gcode.register_command(
                    "BED_MESH_CALIBRATE", self._cmd_calibrate,
                    desc="BED_MESH_CALIBRATE with live LCD point progress (Mesh N/total)")
        except Exception:
            self.prev_cmd = None

    def _set_msg(self, msg):
        ds = self.printer.lookup_object('display_status', None)
        if ds is None:
            return
        try:
            ds.set_message(msg)
        except Exception:
            try:
                ds.message = msg
            except Exception:
                pass

    def _cmd_calibrate(self, gcmd):
        if self.prev_cmd is None:
            return
        self.active = True
        self.count = 0
        self.last_xy = None
        try:
            self.prev_cmd(gcmd)
        finally:
            self.active = False
            self._set_msg(None)            # clear; the print's next M117 takes over

    def _run_total(self):
        # Actual point count for THIS run. ADAPTIVE meshes probe fewer points than
        # the configured probe_count, so prefer the live generated-point list and
        # fall back to the static config-derived total.
        try:
            bm = self.printer.lookup_object('bed_mesh', None)
            pts = bm.bmc.probe_mgr.get_base_points()
            if pts:
                return len(pts)
        except Exception:
            pass
        return self.total

    @staticmethod
    def _xy(results):
        # Probe XY of this event. New probe API: epos has .bed_x/.bed_y; older
        # payloads are a plain [x, y, z] list. Returns None if neither fits.
        try:
            pos = results[0]
        except Exception:
            return None
        try:
            return (float(pos.bed_x), float(pos.bed_y))
        except AttributeError:
            try:
                return (float(pos[0]), float(pos[1]))
            except Exception:
                return None

    def _on_results(self, results):
        # Klipper fires this once PER SAMPLE (run_probe loops _probe `samples`
        # times per point, and the event lives inside _probe). Samples at one
        # point -- and samples_tolerance retries -- share the same XY, so count
        # a point only when XY moves (mesh spacing >> this 0.5mm threshold).
        if not self.active:
            return
        xy = self._xy(results)
        if xy is None:
            return
        if (self.last_xy is None
                or abs(xy[0] - self.last_xy[0]) > 0.5
                or abs(xy[1] - self.last_xy[1]) > 0.5):
            self.count += 1
            self.last_xy = xy
        total = self._run_total()
        self._set_msg("Mesh %d/%d" % (self.count, total)
                      if total else "Mesh %d" % self.count)


def load_config(config):
    return MeshProgress(config)
