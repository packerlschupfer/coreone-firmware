# Sheet profiles: per-build-sheet first-layer Z offset, selectable + persistent.
#
# Each sheet stores a Live-Z DELTA from the probe's z_offset baseline (which stays
# frozen as the mechanical reference). SET_SHEET applies the active sheet's delta as a
# gcode offset (SET_GCODE_OFFSET); PRINT_START re-applies it (APPLY_SHEET) so every
# print starts at the selected sheet's calibrated squish; SHEET_SAVE_Z folds the
# current live babystep into the active sheet. Values persist in [save_variables]
# (no klippy restart -- unlike re-baking the probe z_offset).
#
# Selection is printer-side (the gcode can't know which sheet is clipped on). The
# active sheet + its offset are in get_status, so the LCD/Mainsail can show them.
#
#   [sheets]
#   sheets: textured, smooth        # available sheet names (no spaces)
#   default_sheet: textured         # optional; defaults to the first listed

class Sheets:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        raw = config.get('sheets')
        self.names = [n.strip() for n in raw.replace(',', ' ').split() if n.strip()]
        if not self.names:
            raise config.error("[sheets] needs at least one sheet name")
        self.default = config.get('default_sheet', self.names[0])
        if self.default not in self.names:
            raise config.error(
                "[sheets] default_sheet '%s' not in sheets list" % (self.default,))
        self.active = self.default
        self.offsets = {n: 0.0 for n in self.names}
        self.printer.register_event_handler("klippy:connect", self._connect)
        self.gcode.register_command("SET_SHEET", self.cmd_SET_SHEET,
                                    desc="Select the active build sheet")
        self.gcode.register_command("SHEET_SAVE_Z", self.cmd_SHEET_SAVE_Z,
                                    desc="Save the current live Z offset to the active sheet")
        self.gcode.register_command("APPLY_SHEET", self.cmd_APPLY_SHEET,
                                    desc="Apply the active sheet's Z offset (used by PRINT_START)")
        self.gcode.register_command("LIST_SHEETS", self.cmd_LIST_SHEETS,
                                    desc="List build sheets and their saved Z offsets")

    def _var(self, name):
        return "sheet_" + name.lower()

    def _connect(self):
        # Load persisted offsets + active sheet from save_variables (if present).
        sv = self.printer.lookup_object('save_variables', None)
        if sv is None:
            return
        v = sv.allVariables
        for n in self.names:
            val = v.get(self._var(n))
            if val is not None:
                try:
                    self.offsets[n] = float(val)
                except (TypeError, ValueError):
                    pass
        a = v.get('sheet_active')
        if a in self.names:
            self.active = a

    def get_status(self, eventtime):
        return {
            'active': self.active,
            'offset': round(self.offsets.get(self.active, 0.0), 4),
            'available': list(self.names),
            'offsets': {n: round(o, 4) for n, o in self.offsets.items()},
        }

    def _apply(self, z):
        self.gcode.run_script_from_command("SET_GCODE_OFFSET Z=%.4f MOVE=0" % (z,))

    def _persist(self, var, value):
        # value is a numeric literal (e.g. -0.0450).
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=%s VALUE=%s" % (var, value))

    def _persist_str(self, var, value):
        # The gcode parser strips one layer of quotes, so wrap the literal as
        # VALUE="'name'" -> literal_eval still receives a quoted string 'name'.
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=%s VALUE=\"'%s'\"" % (var, value))

    def cmd_SET_SHEET(self, gcmd):
        name = gcmd.get('NAME')
        if name not in self.names:
            raise gcmd.error("Unknown sheet '%s'. Available: %s"
                             % (name, ", ".join(self.names)))
        self.active = name
        self._apply(self.offsets[name])           # apply offset first (user-visible)
        self._persist_str('sheet_active', name)   # then persist the selection
        gcmd.respond_info("Sheet '%s' active (Z offset %.4f)"
                          % (name, self.offsets[name]))

    def cmd_APPLY_SHEET(self, gcmd):
        self._apply(self.offsets[self.active])

    def cmd_SHEET_SAVE_Z(self, gcmd):
        gm = self.printer.lookup_object('gcode_move')
        z = gm.get_status(self.printer.get_reactor().monotonic())['homing_origin'][2]
        self.offsets[self.active] = z
        self._persist(self._var(self.active), "%.4f" % (z,))
        gcmd.respond_info("Saved Z offset %.4f to sheet '%s'" % (z, self.active))

    def cmd_LIST_SHEETS(self, gcmd):
        lines = ["Build sheets (active: %s):" % (self.active,)]
        for n in self.names:
            mark = "*" if n == self.active else " "
            lines.append("%s %s: %.4f" % (mark, n, self.offsets[n]))
        gcmd.respond_info("\n".join(lines))


def load_config(config):
    return Sheets(config)
