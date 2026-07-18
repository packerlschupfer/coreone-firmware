# Filament profiles: what material is loaded in each tool, and the parameters
# that material implies (nozzle / preheat / bed / chamber temps, filtration...).
#
# WHY THIS EXISTS
# ---------------
# Klipper has no material concept at all. Before this module the printer knew only
# that filament was PRESENT (the HX717 ch-B extruder sensor) -- never WHAT. Material
# was inferred from numbers inside PRINT_START ("EXTRUDER>=240 means it's a hot
# material", "CHAMBER>=45 means it needs a warm enclosure"), and LOAD/UNLOAD_FILAMENT
# used a single hardcoded 220/215 C compromise for every material on the shelf.
#
# This is the Klipper port of Prusa's model:
#   src/common/filament.cpp        -- the preset parameter table (numbers copied verbatim)
#   src/common/filament.hpp        -- FilamentTypeParameters, PresetFilamentType
#   config_store().get/set_filament_type(physical_extruder)  -- PER-TOOL loaded material
#   marlin_print_preview.cpp:241   -- check_correct_filament_type (the gcode-vs-loaded gate)
#     NOTE: upstream DELETED that function (BFW-8307, c484450b8). The behaviour survives
#     but moved into `gcode_compatibility` reporting through a CompatibilityReport, which
#     aggregates the filament check with the other gcode-compat checks. Our FILAMENT_CHECK
#     is modelled on the older standalone shape -- fine, but do not go looking for
#     check_correct_filament_type in a current checkout, it is not there.
#   filament_sensors_handler.cpp:286 -- sensor says gone -> stored type reverts to none
#   src/marlin_stubs/M865.cpp      -- the M865 gcode (compat implemented below)
#
# INDX-READY BY CONSTRUCTION (this is why the state is per-tool even though we have
# one tool today). The INDX toolchanger is 8 bare nozzles, and Prusa's own filament
# state was ALREADY per-extruder -- get_filament_type() takes a physical_extruder.
# Building this global today would mean rewriting it later, so:
#   * every piece of loaded-material state is a list indexed by tool (tools: 1 now, 8 on INDX)
#   * "no tool selected" is a first-class state (INDX idles at NoTool, and many M-codes
#     must keep working there), exposed as active_tool = -1
#   * machine-global values (bed, chamber) are RECONCILED across all loaded tools by
#     _reduce() rather than read off one tool -- with one tool that's an identity, with
#     eight it's the thing that stops two materials fighting over one bed
#   * the presence signal is looked up BY OBJECT NAME per tool (sensor_tool0: ...), not
#     hardcoded to our HX717 sensor. On INDX presence moves to 8x TMP1826 1-Wire bits
#     reported by the xBuddy Extension -- that becomes a config line, not a code change.
#
#   [filaments]
#   tools: 1                                        # physical tool count (INDX: 8)
#   sensor_tool0: hx71x_filament_sensor extruder    # optional presence source for tool 0
#   clear_on_removal: True                          # revert to unknown when filament leaves
#
# User materials / overriding a preset's numbers: either M865 (persisted) or a config
# section, e.g.
#
#   [filaments PETG-CF]
#   nozzle: 240
#   bed: 90
#   abrasive: True

import re

# --- Prusa's preset table, ported verbatim from src/common/filament.cpp ------------
# Its own comment: "These temperatures correspond to slicer defaults for MBL."
#
# ONE DELIBERATE DIVERGENCE -- `preheat`. Prusa's FilamentTypeParameters defaults
# nozzle_preheat_temperature to 170 for everything and only states it explicitly for PC
# and FLEX (both 170 on a loadcell machine). Our PRINT_START has always used a LOWER
# 150 for cooler materials, because we probe with the nozzle TOUCHING the bed and 170 on
# PLA oozes onto the loadcell. The values below reproduce our existing rule exactly
# (170 if nozzle >= 240 else 150), which also happens to agree with Prusa on the two
# materials Prusa pins by hand. Do not "fix" these to 170 without re-testing the mesh.
#
# `mvs` is NOT from Prusa's table -- it is our own fallback max volumetric speed,
# used to bound the LOAD_FILAMENT purge rate when the slicer does not send MVS=.
# Deliberately conservative: it is only a floor-safe guess, and the purge is capped
# at 3 mm/s regardless, so it only ever matters for the low-MVS materials.
PRESETS = {
    #        nozzle preheat  bed  hbrk  ch_min ch_max ch_tgt  filt   abras  is_flexible
    'PLA':  dict(nozzle=215, preheat=150, bed=60,  heatbreak=45, mvs=15.,
                 chamber_min=15, chamber_max=38, chamber_target=20),
    'PETG': dict(nozzle=230, preheat=150, bed=85,  heatbreak=60, mvs=12.,
                 chamber_min=15, chamber_max=45, chamber_target=30),
    'ASA':  dict(nozzle=260, preheat=170, bed=100, heatbreak=65, mvs=12.,
                 chamber_min=40, chamber_max=75, chamber_target=70, filtration=True),
    'PC':   dict(nozzle=275, preheat=170, bed=100, heatbreak=65, mvs=12.,
                 chamber_min=40, chamber_max=80, chamber_target=75, filtration=True),
    'PVB':  dict(nozzle=215, preheat=150, bed=75, mvs=8.,
                 chamber_min=15, chamber_max=38, chamber_target=20),
    'ABS':  dict(nozzle=255, preheat=170, bed=100, heatbreak=65, mvs=12.,
                 chamber_min=40, chamber_max=75, chamber_target=70, filtration=True),
    'HIPS': dict(nozzle=220, preheat=150, bed=100, mvs=12.,
                 chamber_min=40, chamber_max=75, chamber_target=70, filtration=True),
    'PP':   dict(nozzle=240, preheat=170, bed=100, mvs=8.,
                 chamber_min=30, chamber_max=70, chamber_target=60, filtration=True),
    # FLEX/TPU is the material this matters for: its real MVS is ~2.5, so the
    # otherwise-safe 3 mm/s purge would be nearly 3x over its limit.
    'FLEX': dict(nozzle=240, preheat=170, bed=50, mvs=2.5,
                 chamber_min=15, chamber_max=40, chamber_target=25, filtration=True,
                 is_flexible=True),
    'PA':   dict(nozzle=285, preheat=170, bed=100, mvs=10.,
                 chamber_min=40, chamber_max=70, chamber_target=65),
}

# Field defaults -- mirrors FilamentTypeParameters' member initialisers.
FIELDS = {
    'nozzle': 215, 'preheat': 170, 'bed': 60, 'heatbreak': 45,
    'chamber_min': None, 'chamber_max': None, 'chamber_target': None,
    'filtration': False, 'abrasive': False, 'is_flexible': False,   # is_flexible: material PROPERTY
    'mvs': 12.,
}
NUMERIC = ('nozzle', 'preheat', 'bed', 'heatbreak',
           'chamber_min', 'chamber_max', 'chamber_target')
FLOAT = ('mvs',)
BOOLEAN = ('filtration', 'abrasive', 'is_flexible')
# `is_flexible` is upstream's FINAL name and, more importantly, its final SEMANTICS:
# a material PROPERTY, not a policy. It flip-flopped upstream (do_not_autoretract ->
# is_flexible -> do_not_auto_retract -> is_flexible, src/common/filament.hpp:128).
# Consumers DERIVE behaviour from it -- "skip auto-retract", "do not yank it through the
# gears during a tip-forming unload" -- rather than the flag naming one of those actions.

# 1.75mm filament cross-section: mm3/s -> mm/s of filament.
FILAMENT_AREA = 2.405
# The rate LOAD_FILAMENT/M600 purge at when MVS imposes no tighter limit. Validated
# no-smoke 2026-07-26; Prusa's own Core One purge rate.
PURGE_RATE_CAP = 3.0
# Headroom under MVS for the purge -- MVS is a sustained-print limit, and a purge into
# air with a cold-ish path is the worst case for it.
PURGE_MVS_MARGIN = 0.8
# Purge length for a normal load/prime vs one that must clear a DIFFERENT material out of
# the melt zone.
#
# CHANGE = 60, and this one is MEASURED, not inferred. On a real PLA->PETG change on this
# machine (2026-08-30) the operator watched the extrudate through a 100mm hand purge and
# the colour transitioned at ~30mm. 60 is 2x that -- deliberate margin over an observed
# transition, not a guess. Consistent with the other data point: the 20mm auto-purge was
# visibly insufficient, which it would be, sitting below the 30mm transition.
# ⚠ Do NOT "correct" this upward to match Prusa. Stock Buddy purges 80mm on EVERY load
# (ADVANCED_PAUSE_PURGE_LENGTH 40, doubled for high-flow; Core One defaults high-flow),
# then prompts "Is color correct? / Purge more" in unbounded 80mm steps -- one fixed 80mm
# with no prompt on a headless build. Their 80 is larger because it is INDISCRIMINATE: no
# reload-vs-change distinction anywhere in their tree, and no per-machine observation. We
# have both, so we can be tighter on purpose.
#
# NORMAL = 20: a same-material re-prime has nothing to flush. Prusa still spends 80mm here
# because it cannot tell the material is unchanged; we track that (prev_material) and skip
# work it cannot skip. The load moves already push 60mm (40 slow + 20 fast) before the
# purge starts, and nothing observed suggests 20mm is short.
PURGE_LEN_NORMAL = 20.
PURGE_LEN_CHANGE = 60.

# M865 letter -> field, from src/marlin_stubs/M865.cpp's documented parameter block.
M865_FIELDS = {
    'T': 'nozzle', 'P': 'preheat', 'B': 'bed', 'H': 'heatbreak',
    'C': 'chamber_target', 'D': 'chamber_min', 'E': 'chamber_max',
    'F': 'filtration', 'A': 'abrasive', 'G': 'is_flexible',
}

# Names that mean an existing preset under a different label. Our machine profile
# renders TPU as FLEX (Prusa's vendor profile sets filament_type=FLEX), but a human
# typing SET_FILAMENT or picking off the LCD will say TPU, so accept both.
ALIASES = {'TPU': 'FLEX', 'TPE': 'FLEX', 'NYLON': 'PA', 'POLYCARBONATE': 'PC'}

NONE = '---'          # Prusa's own name for "nothing loaded" (none_filament_parameters)
NO_TOOL = -1          # INDX idles here; Marlin calls it NoTool


class Filaments:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.tools = config.getint('tools', 1, minval=1, maxval=8)
        self.clear_on_removal = config.getboolean('clear_on_removal', True)
        # Presence source per tool, by object name. Absent -> that tool never
        # auto-clears (which is the safe direction: we keep believing the last
        # thing we were told rather than silently forgetting it).
        self.sensors = [config.get('sensor_tool%d' % (i,), None)
                        for i in range(self.tools)]
        self._sensor_objs = [None] * self.tools
        self._present = [None] * self.tools

        # loaded[tool] = material name, or NONE when nothing/unknown is loaded.
        self.loaded = [NONE] * self.tools
        # swapped[tool]: the record was cleared BY A REMOVAL, so something may now be
        # loaded that nobody has identified. Distinct from NONE-because-never-known,
        # and the distinction is load-bearing -- see cmd_FILAMENT_CHECK.
        self.swapped = [False] * self.tools
        # prev_material[tool]: the last REAL material this tool held before the current
        # one. Kept across the NONE interlude that a removal creates, which is the whole
        # point -- at SET_FILAMENT time the outgoing material has already been cleared by
        # the sensor, so without this we could not tell a material CHANGE (PLA -> PETG,
        # needs a long purge to clear the old melt) from a reload of the SAME material
        # (needs only the standard prime). Cleared by FILAMENT_PURGED once a load has
        # actually purged for it.
        self.prev_material = [NONE] * self.tools
        # Per-print MVS from the slicer (FILAMENT_CHECK MVS=). NOT persisted -- it
        # arrives with every print and is more current than any table we hold.
        self.mvs = [None] * self.tools
        # Per-material parameter overrides (M865 / [filament NAME] sections).
        self.overrides = {}
        # NOTE the prefix is 'filaments ', not 'filament ': Klipper resolves a section
        # `[filament X]` to a module named filament.py, which does not exist. Prefix
        # sections have to share this module's own name.
        for c in config.get_prefix_sections('filaments '):
            self._load_override_section(c)

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

        self.gcode.register_command(
            "SET_FILAMENT", self.cmd_SET_FILAMENT,
            desc="Tell the printer which material is loaded (TYPE=, optional TOOL=)")
        self.gcode.register_command(
            "FILAMENT_PURGED", self.cmd_FILAMENT_PURGED,
            desc="Clear the material-change purge flag (LOAD_FILAMENT calls this after purging)")
        self.gcode.register_command(
            "CLEAR_FILAMENT", self.cmd_CLEAR_FILAMENT,
            desc="Forget the loaded material for a tool (optional TOOL=)")
        self.gcode.register_command(
            "QUERY_FILAMENT", self.cmd_QUERY_FILAMENT,
            desc="Report the loaded material and its parameters")
        self.gcode.register_command(
            "LIST_FILAMENTS", self.cmd_LIST_FILAMENTS,
            desc="List known materials and what is loaded where")
        self.gcode.register_command(
            "FILAMENT_CHECK", self.cmd_FILAMENT_CHECK,
            desc="Compare the gcode's material against what is loaded (used by PRINT_START)")
        self.gcode.register_command(
            "M865", self.cmd_M865,
            desc="Prusa M865: manage filament types and their parameters")

    # --- material name handling ---------------------------------------------------
    def _normalise(self, raw):
        """PLA/petg/'PETG-CF' -> a known preset name where possible.

        Slicer profiles carry names like PETG-CF, PLA Blend, ABS+. Longest-prefix
        match against the preset table gets those to the right parameters instead of
        dropping them to unknown. An unmatched name is still REMEMBERED verbatim --
        it has no parameters, but it can still catch a gcode-vs-loaded mismatch,
        which is the higher-value half of knowing what is loaded.
        """
        name = re.sub(r'[^A-Za-z0-9+_-]', '', (raw or '').strip()).upper()
        if not name or name == NONE:
            return NONE, None
        if name in self.overrides or name in PRESETS:
            return name, name
        if name in ALIASES:
            return name, ALIASES[name]
        cands = [p for p in PRESETS if name.startswith(p)]
        if cands:
            return name, max(cands, key=len)
        return name, None

    def params(self, name):
        """Merged parameters for a material name: defaults <- preset <- overrides."""
        _, base = self._normalise(name)
        p = dict(FIELDS)
        if base in PRESETS:
            p.update(PRESETS[base])
        # An override may target the alias (PETG-CF) or the base preset (PETG);
        # the more specific one wins.
        for key in (base, name):
            if key and key in self.overrides:
                p.update(self.overrides[key])
        return p

    def known(self, name):
        _, base = self._normalise(name)
        return base is not None or name in self.overrides

    # --- config / persistence -----------------------------------------------------
    def _load_override_section(self, c):
        name = c.get_name().split(None, 1)[1].strip().upper()
        ovr = {}
        for f in NUMERIC:
            v = c.getint(f, None)
            if v is not None:
                ovr[f] = v
        for f in FLOAT:
            v = c.getfloat(f, None)
            if v is not None:
                ovr[f] = v
        for f in BOOLEAN:
            v = c.getboolean(f, None)
            if v is not None:
                ovr[f] = v
        self.overrides.setdefault(name, {}).update(ovr)

    def _var_tool(self, tool):
        return 'fil_tool%d' % (tool,)

    def _var_swapped(self, tool):
        return 'fil_swapped%d' % (tool,)

    def _var_ovr(self, name, field):
        return 'fil_ovr_%s_%s' % (re.sub(r'[^A-Za-z0-9]', '_', name).lower(), field)

    def _persist(self, var, literal):
        self.gcode.run_script_from_command(
            "SAVE_VARIABLE VARIABLE=%s VALUE=%s" % (var, literal))

    def _persist_str(self, var, value):
        # The gcode parser strips one layer of quotes, so wrap as VALUE="'name'"
        # -> literal_eval still receives a quoted string. Same trick as sheets.py.
        self._persist(var, "\"'%s'\"" % (value,))

    def _handle_connect(self):
        sv = self.printer.lookup_object('save_variables', None)
        if sv is not None:
            v = sv.allVariables
            for i in range(self.tools):
                val = v.get(self._var_tool(i))
                if isinstance(val, str) and val:
                    self.loaded[i] = val.upper()
                # Must persist: a swap followed by a power cycle is exactly the case
                # where the printer would otherwise forget that it does not know.
                self.swapped[i] = bool(v.get(self._var_swapped(i), 0))
            # Restore persisted per-material overrides (M865-set).
            prefix = 'fil_ovr_'
            for key, val in v.items():
                if not key.startswith(prefix):
                    continue
                if val is None:
                    continue          # tombstone written by `M865 R` -- override cleared
                body = key[len(prefix):]
                for f in FIELDS:
                    if body.endswith('_' + f):
                        mat = body[:-(len(f) + 1)].replace('_', '-').upper()
                        self.overrides.setdefault(mat, {})[f] = val
                        break
        # Safety: a preset must not ask for more than the heaters can deliver.
        # Prusa does this at compile time (temperatures_are_within_spec static_assert);
        # here it is a connect-time check. Read from the heater objects, NOT from
        # configfile's status 'settings' -- that is built from access_tracking and is
        # not populated yet this early.
        emax = bmax = None
        try:
            emax = self.printer.lookup_object('extruder').get_heater().max_temp
        except Exception:
            pass
        try:
            bmax = self.printer.lookup_object('heater_bed').heater.max_temp
        except Exception:
            pass
        for name in list(PRESETS) + list(self.overrides):
            p = self.params(name)
            if emax is not None and max(p['nozzle'], p['preheat']) > emax - 5:
                raise self.printer.config_error(
                    "[filaments] '%s' wants nozzle %dC but extruder max_temp is %.0fC"
                    % (name, max(p['nozzle'], p['preheat']), emax))
            if bmax is not None and p['bed'] > bmax - 5:
                raise self.printer.config_error(
                    "[filaments] '%s' wants bed %dC but heater_bed max_temp is %.0fC"
                    % (name, p['bed'], bmax))

    def _handle_ready(self):
        for i, sname in enumerate(self.sensors):
            if sname:
                self._sensor_objs[i] = self.printer.lookup_object(sname, None)
                if self._sensor_objs[i] is None:
                    raise self.printer.config_error(
                        "[filaments] sensor_tool%d: no such object '%s'" % (i, sname))
        reactor = self.printer.get_reactor()
        reactor.register_timer(self._poll, reactor.monotonic() + 2.)

    # --- presence tracking ---------------------------------------------------------
    def _poll(self, eventtime):
        """Prusa reverts a tool's stored type to none when its sensor says the filament
        left (filament_sensors_handler.cpp:286). Same idea, with two guards:

        1. IDLE ONLY. Our tool-0 sensor shares the loadcell's HX717 and is mode-switched
           (ch B idle, ch A while probing), so mid-print readings are not trustworthy
           enough to erase state on. A false 'absent' during a print would silently
           forget the material and take the chamber/bed reconciliation with it.
        2. A tool with no configured sensor never auto-clears.

        Insert does NOT auto-set a material -- the printer cannot see colour or type, so
        it waits to be told (SET_FILAMENT / the LCD menu / PRINT_START's MATERIAL=).
        """
        if self.clear_on_removal:
            printing = self._print_state() in ('printing', 'paused')
            for i, obj in enumerate(self._sensor_objs):
                if obj is None:
                    continue
                try:
                    present = bool(obj.get_status(eventtime).get('filament_detected'))
                except Exception:
                    continue
                prev, self._present[i] = self._present[i], present
                if prev and not present and not printing and self.loaded[i] != NONE:
                    gone = self.loaded[i]
                    # swapped=True: whatever goes in next is UNIDENTIFIED, and the
                    # printer must not later guess it from a gcode file.
                    self._set(i, NONE, swapped=True)
                    self.gcode.respond_info(
                        "Filament removed from tool %d -- forgetting '%s'. "
                        "SET_FILAMENT TYPE=<material> after loading (a print will "
                        "not start until you do)." % (i, gone))
        return eventtime + 2.

    def _print_state(self):
        ps = self.printer.lookup_object('print_stats', None)
        if ps is None:
            return 'standby'
        try:
            return ps.get_status(self.printer.get_reactor().monotonic())['state']
        except Exception:
            return 'standby'

    def _set(self, tool, name, swapped=False):
        if self.loaded[tool] != name:
            self.mvs[tool] = None      # a slicer MVS belongs to the material it came with
            # Remember the outgoing material, but ONLY when it was a real one -- a removal
            # sets NONE, and overwriting with NONE there would erase exactly the fact we
            # need (that PLA was in the machine before PETG went in).
            if self.loaded[tool] != NONE:
                self.prev_material[tool] = self.loaded[tool]
        self.loaded[tool] = name
        self._persist_str(self._var_tool(tool), name)
        if self.swapped[tool] != swapped:
            self.swapped[tool] = swapped
            self._persist(self._var_swapped(tool), '1' if swapped else '0')

    # --- purge rate ------------------------------------------------------------------
    def _effective_mvs(self, tool):
        """Best available max volumetric speed for a tool, mm3/s.

        Preference: the slicer's per-print MVS (most current) > the material's table
        value > the global default. See _sane_mvs for why the slicer's value is not
        trusted blindly.
        """
        if tool != NO_TOOL and self.mvs[tool] is not None:
            return self.mvs[tool]
        name = NONE if tool == NO_TOOL else self.loaded[tool]
        return self.params(name)['mvs'] if name != NONE else FIELDS['mvs']

    def _sane_mvs(self, raw):
        """Filter a slicer-supplied MVS. Returns None if it should be ignored.

        Two documented OrcaSlicer quirks (from the slicer chat, 2026-08-30):
          0    means "no limit" in Orca, NOT zero speed -- so it tells us nothing.
          >=100 is the flow-calibration placeholder (Orca temporarily writes 200 into
               the filament profile and it can persist to the file), not a material limit.
        """
        try:
            v = float(raw)
        except (TypeError, ValueError):
            return None
        return v if 0. < v < 100. else None

    def purge_rate(self, tool=None):
        """Filament feed rate for a purge, mm/s -- min(3 mm/s, 80% of MVS).

        The 3 mm/s cap is the validated no-smoke rate and is what almost every
        material lands on. The MVS term only bites for genuinely slow materials:
        FLEX at MVS 2.5 comes out at ~0.83 mm/s, where the flat 3 mm/s would have
        been ~2.9x over its limit.
        """
        if tool is None:
            tool = self.active_tool()
        limit = PURGE_MVS_MARGIN * self._effective_mvs(tool) / FILAMENT_AREA
        return min(PURGE_RATE_CAP, limit)

    def safe_purge_rate(self):
        """The slowest rate ANY known material needs -- for purging something we cannot
        identify (a load with no TYPE=, after a swap).

        The generic default is wrong here. We cannot rule out FLEX/TPU (MVS 2.5), and the
        3 mm/s cap is ~2.9x over its limit. Purging too slow costs seconds; purging too
        fast smokes or jams. So when we do not know, assume the most demanding material.
        """
        mvs = [self.params(n)['mvs']
               for n in list(PRESETS) + list(self.overrides)]
        return min(PURGE_RATE_CAP,
                   PURGE_MVS_MARGIN * min(mvs or [FIELDS['mvs']]) / FILAMENT_AREA)

    # --- tool selection ------------------------------------------------------------
    def active_tool(self):
        """Which physical tool is selected.

        THE INDX SEAM. With one tool the answer is always 0. When the toolchanger
        lands this reads the changer's current tool and must be able to return
        NO_TOOL, because INDX idles with no nozzle picked up and the LCD/menus and
        plain M-codes have to keep working in that state.
        """
        return 0 if self.tools == 1 else self._active_tool

    _active_tool = 0

    def _resolve_tool(self, gcmd, allow_none=False):
        t = gcmd.get_int('TOOL', self.active_tool())
        if t == NO_TOOL and allow_none:
            return t
        if t < 0 or t >= self.tools:
            raise gcmd.error("TOOL=%d out of range 0..%d" % (t, self.tools - 1))
        return t

    # --- machine-global reconciliation ---------------------------------------------
    def _reduce(self):
        """Collapse every loaded tool's parameters into the machine-wide values.

        With one tool this is an identity and the rules below never actually fire.
        They exist so INDX does not need this rewritten: there is one bed and one
        chamber, and eight tools can disagree about them.

          bed / chamber_min / chamber_target -> MAX  (the neediest material wins;
              running ABS at PLA's bed temp warps it)
          chamber_max                        -> MIN  (the vent ceiling belongs to the
              most heat-sensitive material loaded; PLA in the chamber caps it at 38
              even if ASA would tolerate 75)
          filtration / abrasive              -> ANY

        Those two chamber rules can collide: ASA wants >=40 and PLA cannot exceed 38,
        so loading both leaves an EMPTY window (min 40 > max 38) -- the materials are
        genuinely not co-printable in one chamber. Rather than emit a nonsense target,
        that is flagged as chamber_conflict for the caller to refuse on. Unreachable
        with one tool; it is here so INDX cannot inherit the silent-nonsense version.
        """
        mats = [n for n in self.loaded if n != NONE]
        out = {'bed': None, 'chamber_min': None, 'chamber_max': None,
               'chamber_target': None, 'filtration': False, 'abrasive': False,
               'chamber_conflict': False}
        for n in mats:
            p = self.params(n)
            for f, op in (('bed', max), ('chamber_min', max),
                          ('chamber_target', max), ('chamber_max', min)):
                v = p.get(f)
                if v is None:
                    continue
                out[f] = v if out[f] is None else op(out[f], v)
            out['filtration'] = out['filtration'] or p['filtration']
            out['abrasive'] = out['abrasive'] or p['abrasive']
        lo, hi = out['chamber_min'], out['chamber_max']
        if lo is not None and hi is not None and lo > hi:
            out['chamber_conflict'] = True
            # Keep the target inside the ceiling so nothing downstream commands a
            # temperature the coolest-running material cannot survive.
            out['chamber_target'] = hi
        return out

    def get_status(self, eventtime):
        tool = self.active_tool()
        name = NONE if tool == NO_TOOL else self.loaded[tool]
        p = self.params(name) if name != NONE else dict(FIELDS)
        # === STATE FIELDS -- READ THIS TABLE BEFORE KEYING A UI OFF ANY OF THEM ===
        # There are FOUR states, not two, and the obvious-looking field is the wrong one:
        #
        #   situation                | name          | material_unknown | must_declare | has_params
        #   -------------------------|---------------|------------------|--------------|-----------
        #   material loaded          | PLA           | False            | False        | True
        #   cold start/CLEAR_FILAMENT| ---           | True             | False        | False
        #   filament physically gone | ---           | True             | **True**     | False
        #   loaded, no preset for it | PEI-1010-CF   | False            | False        | **False**
        #
        # The last row is the trap, and it already produced a wrong badge in the Mainsail UI:
        # an unrecognised name IS recorded (the printer knows exactly what is loaded and will
        # display it) but we have no parameters for it. So:
        #   * "do we know what is loaded?"          -> material_unknown   (NOT has_parameters)
        #   * "will a print refuse to start?"       -> must_declare_material
        #   * "do we have temps for this material?" -> has_parameters  -- a WEAKER, different
        #     warning; word it "no profile for <name>", never "material not set", because a
        #     name IS present and the useful action is a [filaments <NAME>] section or M865.
        # FILAMENT_CHECK itself branches on `loaded == NONE` and then the swapped flag; it
        # never consults has_parameters at all.
        unknown = (name == NONE)
        st = {
            'active_tool': tool,
            'name': name,                       # active tool's material (LCD reads this)
            # --- the three fields to actually use ---
            'material_unknown': unknown,        # nothing recorded for this tool
            'must_declare_material': (tool != NO_TOOL and self.swapped[tool]),
            'has_parameters': self.known(name),
            # --- DEPRECATED ALIASES, kept so already-shipped consumers do not silently break.
            # Both were misread in practice: `known` reads as "we know what is loaded" but means
            # "we have parameters", and `awaiting_material` reads as "material unknown" but means
            # the much narrower "it was removed and nobody said what replaced it". A consumer
            # keying on either alone is wrong in at least one of the four states above. Prefer
            # the three fields above; these two may be removed once no consumer uses them.
            'known': self.known(name),                                    # -> has_parameters
            'awaiting_material': (tool != NO_TOOL and self.swapped[tool]),  # -> must_declare_material
            'loaded': list(self.loaded),        # per-tool, INDX-ready
            'available': sorted(set(list(PRESETS) + list(self.overrides))),
            'tools': self.tools,
            # Active tool's parameters, flattened for easy Jinja access.
            'nozzle': p['nozzle'],
            'load_temp': p['nozzle'],           # Prusa loads at the material's nozzle temp
            'preheat': p['preheat'],            # probe / MBL temp
            'heatbreak': p['heatbreak'],
            'is_flexible': p['is_flexible'],
            # A material CHANGE needs far more purge than a reload: the old material is
            # still in the melt zone and the hot end holds ~roughly this much. 20mm (the
            # standard prime) leaves a PLA/PETG blend in the first cm of the next print.
            'material_changed': self._material_changed(tool),
            'purge_len': (PURGE_LEN_CHANGE if self._material_changed(tool)
                          else PURGE_LEN_NORMAL),
            'mvs': round(self._effective_mvs(tool), 2),
            # Ready to drop straight into a `G1 E.. F{...}` -- mm/min, not mm/s.
            'purge_feedrate': int(round(self.purge_rate(tool) * 60.)),
            # Use this one whenever the material being loaded is NOT known.
            'safe_purge_feedrate': int(round(self.safe_purge_rate() * 60.)),
        }
        st.update(self._reduce())               # bed / chamber_* / filtration / abrasive
        return st

    # --- commands ------------------------------------------------------------------
    def cmd_SET_FILAMENT(self, gcmd):
        raw = gcmd.get('TYPE')
        tool = self._resolve_tool(gcmd)
        name, base = self._normalise(raw)
        if name == NONE:
            self._set(tool, NONE)
            gcmd.respond_info("Tool %d: material cleared" % (tool,))
            return
        self._set(tool, name)
        p = self.params(name)
        if base is None and name not in self.overrides:
            gcmd.respond_info(
                "Tool %d: '%s' remembered, but it matches no known material -- no "
                "temperatures to offer. Add [filament %s] or use M865 to give it "
                "parameters. Known: %s"
                % (tool, name, name, ", ".join(sorted(PRESETS))))
            return
        via = "" if base == name else " (parameters from %s)" % (base,)
        gcmd.respond_info(
            "Tool %d: %s%s -- nozzle %dC, bed %dC, probe %dC%s"
            % (tool, name, via, p['nozzle'], p['bed'], p['preheat'],
               ", chamber %dC" % p['chamber_target']
               if p['chamber_target'] is not None else ""))

    def _material_changed(self, tool):
        """True when the material now recorded differs from the last real one this tool
        held -- i.e. a genuine change rather than a reload of the same spool/material."""
        if tool == NO_TOOL:
            return False
        cur, prev = self.loaded[tool], self.prev_material[tool]
        if cur == NONE or prev == NONE:
            return False
        _, cb = self._normalise(cur)
        _, pb = self._normalise(prev)
        # PETG-CF after PETG is not a material change worth a long purge.
        if cb is not None and cb == pb:
            return False
        return cur != prev

    def cmd_FILAMENT_PURGED(self, gcmd):
        """Acknowledge that a load has purged for the material change, so the next load
        goes back to the normal prime length. Called by LOAD_FILAMENT after its purge."""
        tool = self._resolve_tool(gcmd)
        self.prev_material[tool] = NONE

    def cmd_CLEAR_FILAMENT(self, gcmd):
        """Explicit 'I do not know' -- and the escape hatch out of the swapped state.

        Clearing by hand resets to never-known, so the next print may adopt its
        material from the gcode again. That is deliberate: an automatic clear (the
        sensor saw filament leave) must NOT be adoptable, but a person saying "start
        over" is a considered instruction, not a guess.
        """
        tool = self._resolve_tool(gcmd)
        was_swapped = self.swapped[tool]
        self._set(tool, NONE)
        gcmd.respond_info(
            "Tool %d: material cleared%s"
            % (tool, " (the next print may adopt its material from the gcode)"
               if was_swapped else ""))

    def cmd_QUERY_FILAMENT(self, gcmd):
        tool = self._resolve_tool(gcmd, allow_none=True)
        if tool == NO_TOOL:
            gcmd.respond_info("No tool selected")
            return
        name = self.loaded[tool]
        if name == NONE:
            gcmd.respond_info(
                "Tool %d: nothing recorded. SET_FILAMENT TYPE=<material> to tell it."
                % (tool,))
            return
        p = self.params(name)
        gcmd.respond_info("Tool %d: %s\n%s" % (tool, name, self._describe(p)))

    def _describe(self, p):
        ch = ("chamber %s/%s/%s (min/target/max)"
              % tuple('-' if p[f] is None else p[f]
                      for f in ('chamber_min', 'chamber_target', 'chamber_max')))
        flags = [f for f in BOOLEAN if p[f]]
        return ("  nozzle %dC  preheat %dC  bed %dC  heatbreak %dC\n  %s%s"
                % (p['nozzle'], p['preheat'], p['bed'], p['heatbreak'], ch,
                   ("\n  " + ", ".join(flags)) if flags else ""))

    def cmd_LIST_FILAMENTS(self, gcmd):
        lines = ["Loaded:"]
        for i in range(self.tools):
            lines.append("  tool %d: %s%s"
                         % (i, self.loaded[i],
                            "" if i != self.active_tool() else "   <- active"))
        red = self._reduce()
        if red['bed'] is not None:
            lines.append("Machine targets from loaded material(s): bed %s, chamber %s"
                         % (red['bed'], red['chamber_target']))
        lines.append("Known materials:")
        for n in sorted(set(list(PRESETS) + list(self.overrides))):
            p = self.params(n)
            lines.append("  %-6s nozzle %3d  bed %3d  probe %3d%s"
                         % (n, p['nozzle'], p['bed'], p['preheat'],
                            "  (custom)" if n in self.overrides else ""))
        gcmd.respond_info("\n".join(lines))

    def cmd_FILAMENT_CHECK(self, gcmd):
        """The gcode-vs-loaded gate -- Prusa's check_correct_filament_type
        (marlin_print_preview.cpp:241), called from PRINT_START.

        Three outcomes, chosen so this can never make things worse than not knowing:
          nothing recorded -> ADOPT the gcode's material and say so. The printer
              teaches itself on the first print after a swap; no user action needed,
              and it is strictly more information than we had before.
          match            -> silent.
          mismatch         -> ABORT by default (this is the whole point: PETG gcode
              on loaded PLA). ON_MISMATCH=WARN downgrades it.
        """
        tool = self._resolve_tool(gcmd)
        # Applied in the finally below, NOT here: _set() drops a stale MVS when the
        # material changes, so recording it first would let the adopt path wipe it.
        mvs = self._sane_mvs(gcmd.get('MVS', None))
        try:
            self._check(gcmd, tool)
        finally:
            if mvs is not None:
                self.mvs[tool] = mvs

    def _check(self, gcmd, tool):
        raw = gcmd.get('MATERIAL', None)
        if raw is None:
            return                                   # slicer hasn't been updated yet
        want, _ = self._normalise(raw)
        if want == NONE:
            return
        policy = gcmd.get('ON_MISMATCH', 'ABORT').upper()
        have = self.loaded[tool]
        if have == NONE:
            # TWO DIFFERENT KINDS OF "NOTHING RECORDED", and conflating them was a real
            # hole (caught in review by the slicer chat, 2026-08-30):
            #
            #   never known   -- cold start, no filament event ever seen. Adopting the
            #                    gcode's material is strictly more information than we
            #                    had, and cannot contradict anything.
            #   swapped       -- the sensor saw filament LEAVE, so something physically
            #                    changed and nobody said what went back in. Adopting
            #                    here would rubber-stamp the guess at precisely the
            #                    moment a mismatch is most likely: pull PLA, insert PLA
            #                    again, send an ASA file -> adopt ASA -> heat to 269C
            #                    with PLA in the hotend. That is the exact accident
            #                    this whole command exists to prevent, and prints 2..n
            #                    would have been protected while print 1 was not.
            #
            # The sensor is presence-only (HX717 ch B) and can never identify the new
            # material, so after a swap the printer must be TOLD, not left to infer.
            if self.swapped[tool]:
                msg = ("FILAMENT UNKNOWN on tool %d: filament was changed and the "
                       "printer was not told what went in. The gcode is sliced for "
                       "%s -- if that is what you loaded, run SET_FILAMENT TYPE=%s "
                       "and restart the print."
                       % (tool, want, want))
                if policy == 'WARN':
                    self._set(tool, want)
                    gcmd.respond_info(msg + "  [ON_MISMATCH=WARN -- adopting anyway]")
                    return
                raise gcmd.error(msg)
            self._set(tool, want)
            gcmd.respond_info(
                "Filament: nothing was recorded for tool %d -- adopting '%s' from the "
                "gcode. If that is wrong, cancel and SET_FILAMENT TYPE=<material>."
                % (tool, want))
            return
        if have == want:
            return
        # PETG-CF loaded vs PETG in the gcode is not a mismatch worth aborting on.
        _, hb = self._normalise(have)
        _, wb = self._normalise(want)
        if hb is not None and hb == wb:
            gcmd.respond_info("Filament: gcode says %s, loaded is %s -- same base "
                              "material, continuing." % (want, have))
            return
        msg = ("FILAMENT MISMATCH on tool %d: the gcode was sliced for %s but %s is "
               "loaded. Load %s (or SET_FILAMENT TYPE=%s if the printer is wrong), "
               "then restart the print."
               % (tool, want, have, want, want))
        if policy == 'WARN':
            gcmd.respond_info(msg + "  [ON_MISMATCH=WARN -- continuing anyway]")
            return
        raise gcmd.error(msg)

    # --- Prusa M865 compatibility ---------------------------------------------------
    def cmd_M865(self, gcmd):
        """M865: manage filament parameters -- src/marlin_stubs/M865.cpp.

        Supported: S<name> select, I<ix> select what tool ix has loaded, L<ix> set
        tool ix's loaded material, R reset to defaults, and the parameter letters
        T/P/B/H/C/D/E/F/A/G. Prusa quotes names (S"PETG"); Klipper's parser has no
        quoting, so the raw command line is parsed here and both forms are accepted.

        Not supported: U<ix> (user filament slots) and X (ad-hoc pending type) --
        those index Prusa's fixed-size EEPROM slot arrays. Here a material is just a
        name, so `M865 S<name> T<temp>` covers what they were for.
        """
        line = gcmd.get_commandline()
        def opt(letter):
            m = re.search(r'(?:^|\s)%s"?([^"\s]*)"?' % (letter,), line, re.I)
            return m.group(1) if m else None

        sel = opt('S')
        idx = opt('I')
        lslot = opt('L')
        name = None
        if idx is not None:
            tool = int(idx)
            if tool < 0 or tool >= self.tools:
                raise gcmd.error("M865 I%d: out of range 0..%d" % (tool, self.tools - 1))
            name = self.loaded[tool]
        elif sel:
            name, _ = self._normalise(sel)
        elif lslot is not None:
            raise gcmd.error("M865 L needs a material: M865 L<tool> S<name>")
        if name is None:
            gcmd.respond_info("M865: no filament selected. "
                              "Use S<name>, or I<tool> for what a tool has loaded.")
            return
        if name == NONE:
            raise gcmd.error("M865: no material to act on (tool is empty)")

        reset = bool(re.search(r'(?:^|\s)R(?:\s|$)', line, re.I))
        prev = dict(self.overrides.get(name, {}))
        ovr = {} if reset else dict(prev)
        touched = False
        for letter, field in M865_FIELDS.items():
            v = opt(letter)
            if v is None or v == '':
                continue
            ovr[field] = bool(int(v)) if field in BOOLEAN else int(float(v))
            touched = True
        if touched or reset:
            if ovr:
                self.overrides[name] = ovr
            else:
                self.overrides.pop(name, None)
            for field, val in ovr.items():
                self._persist(self._var_ovr(name, field),
                              str(int(val)) if field in BOOLEAN else str(val))
            # R drops fields, so any previously persisted one that survives into no
            # override must be tombstoned -- otherwise it comes straight back on the
            # next klippy start and `M865 R` silently does nothing across a restart.
            for field in prev:
                if field not in ovr:
                    self._persist(self._var_ovr(name, field), 'None')

        if lslot is not None:
            tool = int(lslot)
            if tool < 0 or tool >= self.tools:
                raise gcmd.error("M865 L%d: out of range 0..%d" % (tool, self.tools - 1))
            self._set(tool, name)
        gcmd.respond_info("M865 %s%s\n%s"
                          % (name,
                             "" if lslot is None else " -> tool %s" % (lslot,),
                             self._describe(self.params(name))))


def load_config(config):
    return Filaments(config)


def load_config_prefix(config):
    # [filament <NAME>] sections are consumed by [filaments] itself; this exists so
    # Klipper does not reject them as an unknown section type.
    return config.get_printer().load_object(config, 'filaments')
