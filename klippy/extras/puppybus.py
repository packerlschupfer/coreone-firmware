# PuppyBus master (host side) — talk to the Prusa xBuddy extension (STM32H503)
# over the main MCU's RS-485 link (USART6 + DE on PB7). The MCU is a dumb blocking
# transceiver (src/stm32/puppybus.c); all framing, CRC and protocol state live here.
#
# Frame format (Prusa puppy bootloader, see Buddy src/puppies/BootloaderProtocol):
#   request : [addr][cmd][data...][crc16_lo][crc16_hi]
#   response: [addr][status][len][data...][crc16_lo][crc16_hi]
# CRC is CRC16-IBM/Modbus (poly 0xA001, init 0xFFFF), little-endian on the wire.
#
# Copyright (C) 2026
# This file may be distributed under the terms of the GNU GPLv3 license.
import hashlib
import logging
import struct


def crc16_ibm(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# Bootloader commands (subset)
BL_GET_PROTOCOL_VERSION = 0x00
BL_SET_ADDRESS = 0x01
BL_GET_HARDWARE_INFO = 0x03
BL_START_APPLICATION = 0x05
BL_WRITE_FLASH = 0x06       # data=[off32_BE][bytes]; H5 needs CONSECUTIVE writes from 0
BL_FINALIZE_FLASH = 0x07    # commits the last partial page; returns [erase_count]
BL_READ_FLASH = 0x08        # data=[off32_BE][len]; returns [len bytes] of the app region
BL_COMPUTE_FINGERPRINT = 0x0f  # bootloader hashes salt(LE 4B)+whole app -> internal fp

# WRITE_FLASH frame = addr+cmd+off(4)+data+crc(2); the F427 txrx buffer is 48 B, so
# cap flash data at 40 B/chunk. Offsets are relative to the app (FLASH_APP_OFFSET) —
# WRITE_FLASH physically cannot reach the bootloader sectors.
FLASH_CHUNK = 40

# Bus addresses
ADDR_DEFAULT = 0x00     # DYNAMICALLY-addressed puppies (toolhead Dwarf) boot here
ADDR_FIRST = 0x0A       # first assigned bootloader address (dynamic puppies)
MODBUS_OFFSET = 0x1A    # address once the app is running
# The xBuddy-extension (H503) is a FIXED-address puppy. Its bootloader is built with
# FIXED_ADDRESS=17 + DISABLE_WATCHDOG (Prusa-Bootloader-Puppy/CMakeLists.txt, BOARD
# xbuddy_extension), so it listens at 0x11 (NOT 0x00) and waits in the bootloader
# forever (no watchdog, no auto-jump) until it gets START_APPLICATION. Talk to it here.
ADDR_XBE_BOOTLOADER = 0x11   # = 17, the xBuddy-extension bootloader's fixed address

STATUS_NAMES = {
    0x00: "OK", 0x01: "FAILED", 0x02: "NOT_SUPPORTED",
    0x03: "INVALID_TRANSFER", 0x04: "INVALID_CRC", 0x05: "INVALID_ARGUMENTS",
}


class PuppyBus:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        ppins = self.printer.lookup_object('pins')
        de_params = ppins.lookup_pin(config.get('de_pin', 'PB7'))
        self.mcu = de_params['chip']
        self.de_pin = de_params['pin']
        self.pwr_pin = ppins.lookup_pin(config.get('pwr_pin', 'PG2'))['pin']
        self.reset_pin = ppins.lookup_pin(config.get('reset_pin', 'PG8'))['pin']
        self.baud = config.getint('baud', 230400)
        self.de_tx_level = config.getint('de_tx_level', 1, minval=0, maxval=1)
        self.read_timeout = config.getint('read_timeout_us', 20000,
                                          minval=100, maxval=100000)
        self.gap = config.getint('gap_us', 2000, minval=100, maxval=20000)
        self.txrx_cmd = None
        self.precharge_cmd = None
        self.power_cmd = None
        self.mcu.register_config_callback(self._build_config)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('PUPPY_PRECHARGE', self.cmd_PUPPY_PRECHARGE,
                               desc="Soft-start power the extension port (PG2)")
        gcode.register_command('PUPPY_POWER', self.cmd_PUPPY_POWER,
                               desc="Set extension-port power: ON=0 cuts PG2, ON=1 forces it on")
        gcode.register_command('PUPPY_DEBUG', self.cmd_PUPPY_DEBUG,
                               desc="Report USART6 registers + TX self-test")
        gcode.register_command('PUPPY_PININFO', self.cmd_PUPPY_PININFO,
                               desc="Report PC6/PC7 GPIO mode + alt-function routing")
        gcode.register_command('PUPPY_DISCOVER', self.cmd_PUPPY_DISCOVER,
                               desc="Atomic reset+probe of the H503 bootloader")
        gcode.register_command('PUPPY_PROTO_VERSION',
                               self.cmd_PUPPY_PROTO_VERSION,
                               desc="Read the puppy bootloader protocol version")
        gcode.register_command('PUPPY_HWINFO', self.cmd_PUPPY_HWINFO,
                               desc="Read the puppy bootloader hardware info")
        gcode.register_command('PUPPY_RAW', self.cmd_PUPPY_RAW,
                               desc="Send a raw PuppyBus frame: ADDR= CMD= DATA=hex")
        gcode.register_command('PUPPY_READ_FLASH', self.cmd_PUPPY_READ_FLASH,
                               desc="Read the H503 app flash via the bootloader")
        gcode.register_command('PUPPY_FLASH_VERIFY', self.cmd_PUPPY_FLASH_VERIFY,
                               desc="Prove RS-485 flashing: write a test pattern + read back")
        gcode.register_command('PUPPY_FLASH', self.cmd_PUPPY_FLASH,
                               desc="Upload a firmware .bin to the H503 over RS-485: FILE=")
        gcode.register_command('PUPPY_START', self.cmd_PUPPY_START,
                               desc="Validate (salted fingerprint) + boot the H503 app: FILE=")

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_puppybus de_pin=%s baud=%d pwr_pin=%s de_tx_level=%d reset_pin=%s"
            % (self.de_pin, self.baud, self.pwr_pin, self.de_tx_level, self.reset_pin))
        self.txrx_cmd = self.mcu.lookup_query_command(
            "puppybus_txrx write=%*s read_timeout=%u gap=%u",
            "puppybus_response read=%*s")
        self.reset_probe_cmd = self.mcu.lookup_query_command(
            "puppybus_reset_probe reset_assert=%u reset_run=%u reset_us=%u"
            " boot_us=%u write=%*s read_timeout=%u gap=%u",
            "puppybus_reset_response read=%*s")
        self.precharge_cmd = self.mcu.lookup_command(
            "puppybus_precharge precharge_ms=%u")
        self.power_cmd = self.mcu.lookup_command("puppybus_power on=%u")
        self.debug_cmd = self.mcu.lookup_query_command(
            "puppybus_debug",
            "puppybus_debug_result pclk=%u brr=%u cr1=%u sr0=%u txe=%u tc=%u"
            " fault_ok=%u gpiog=%u gpioc=%u")
        self.pininfo_cmd = self.mcu.lookup_query_command(
            "puppybus_pininfo", "puppybus_pininfo_result moder=%u afrl=%u")

    def cmd_PUPPY_DEBUG(self, gcmd):
        p = self.debug_cmd.send([])
        exp_brr = int(round(42000000.0 / self.baud))
        pg2 = (p['gpiog'] >> 2) & 1   # ext_pwr_enable
        pg8 = (p['gpiog'] >> 8) & 1   # ext_reset
        pc13 = (p['gpioc'] >> 13) & 1  # ext_shutdown
        gcmd.respond_info(
            "USART6: pclk=%d brr=%d (exp ~%d) cr1=0x%04x sr0=0x%04x txe=%d tc=%d -> %s\n"
            "EXT pins: ext_fault=%s (1=OK,0=FAULT)  ext_pwr(PG2)=%d  "
            "ext_reset(PG8)=%d  ext_shutdown(PC13)=%d"
            % (p['pclk'], p['brr'], exp_brr, p['cr1'], p['sr0'], p['txe'], p['tc'],
               "TX OK" if (p['txe'] and p['tc']) else "TX NOT clocking!",
               p['fault_ok'], pg2, pg8, pc13))

    def cmd_PUPPY_PININFO(self, gcmd):
        p = self.pininfo_cmd.send([])
        moder, afrl = p['moder'], p['afrl']
        pc6_mode = (moder >> 12) & 3   # PC6 (TX): expect 2 (alt-function)
        pc7_mode = (moder >> 14) & 3   # PC7 (RX): expect 2
        pc6_af = (afrl >> 24) & 0xf    # PC6 AF: expect 8 (USART6)
        pc7_af = (afrl >> 28) & 0xf    # PC7 AF: expect 8
        ok = (pc6_mode == 2 and pc6_af == 8 and pc7_mode == 2 and pc7_af == 8)
        gcmd.respond_info(
            "PC6(TX): mode=%d (exp 2=AF) af=%d (exp 8)   "
            "PC7(RX): mode=%d af=%d   -> %s"
            % (pc6_mode, pc6_af, pc7_mode, pc7_af,
               "ROUTING OK (signal is on PC6)" if ok else "ROUTING WRONG!"))

    def cmd_PUPPY_DISCOVER(self, gcmd):
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        cmd = gcmd.get_int('CMD', 0x00, minval=0, maxval=255)  # 0 = GET_PROTOCOL_VERSION
        data = bytes.fromhex(gcmd.get('DATA', '')) if gcmd.get('DATA', '') else b''
        reset_assert = gcmd.get_int('RESET_ASSERT', 1, minval=0, maxval=1)
        reset_run = gcmd.get_int('RESET_RUN', 0, minval=0, maxval=1)
        reset_ms = gcmd.get_int('RESET_MS', 2, minval=0, maxval=100)
        boot_ms = gcmd.get_int('BOOT_MS', 5, minval=0, maxval=500)
        rt = gcmd.get_int('READ_TIMEOUT_US', 50000, minval=100, maxval=200000)
        frame = bytes([addr, cmd]) + bytes(data)
        crc = crc16_ibm(frame)
        msg = list(frame) + [crc & 0xff, (crc >> 8) & 0xff]
        p = self.reset_probe_cmd.send([reset_assert, reset_run, reset_ms * 1000,
                                       boot_ms * 1000, msg, rt, self.gap])
        resp = bytes(p['read'])
        info = ("PUPPY_DISCOVER addr=0x%02x cmd=0x%02x (assert=%d run=%d reset=%dms "
                "boot=%dms): " % (addr, cmd, reset_assert, reset_run, reset_ms, boot_ms))
        if not resp:
            gcmd.respond_info(info + "NO RESPONSE")
            return
        info += "%d bytes: %s" % (len(resp), resp.hex())
        if len(resp) >= 5 and resp[0] == addr:
            status, dlen = resp[1], resp[2]
            body = resp[3:3 + dlen]
            crc_calc = crc16_ibm(resp[0:3 + dlen])
            crc_rx = (resp[3 + dlen] | (resp[4 + dlen] << 8)
                      if len(resp) >= 5 + dlen else None)
            info += "\n  -> status=0x%02x len=%d data=%s crc=%s" % (
                status, dlen, body.hex(), "ok" if crc_rx == crc_calc else "BAD")
            if cmd == 0 and dlen == 2:
                info += " protocol=0x%04x" % ((body[0] << 8) | body[1])
        gcmd.respond_info(info)

    def cmd_PUPPY_PRECHARGE(self, gcmd):
        ms = gcmd.get_int('MS', 15, minval=1, maxval=200)
        self.precharge_cmd.send([ms])
        gcmd.respond_info("PuppyBus: extension port precharged + powered (PG2 high)")

    def cmd_PUPPY_POWER(self, gcmd):
        # ON=0 cuts PG2 (lets PUPPY_REFLASH power-cycle a running H503 into its bootloader
        # without a FIRMWARE_RESTART); ON=1 forces it on (PUPPY_PRECHARGE soft-starts instead).
        on = gcmd.get_int('ON', minval=0, maxval=1)
        self.power_cmd.send([on])
        gcmd.respond_info("PuppyBus: extension port power %s (PG2 %s)"
                          % ("ON" if on else "OFF", "high" if on else "low"))

    # --- wire layer ---------------------------------------------------------
    def txrx(self, frame, read_timeout=None, gap=None):
        # frame = bytes(addr, cmd, data...) WITHOUT crc; we append crc16
        rt = self.read_timeout if read_timeout is None else read_timeout
        gp = self.gap if gap is None else gap
        crc = crc16_ibm(frame)
        msg = list(frame) + [crc & 0xff, (crc >> 8) & 0xff]
        params = self.txrx_cmd.send([msg, rt, gp])
        return bytes(params['read'])

    def bl_command(self, addr, cmd, data=b'', read_timeout=None):
        # Returns (status, data_bytes) or raises on transport error.
        resp = self.txrx(bytes([addr, cmd]) + bytes(data), read_timeout)
        if len(resp) == 0:
            raise self.printer.command_error(
                "PuppyBus: no response from addr 0x%02x cmd 0x%02x" % (addr, cmd))
        if len(resp) < 5:
            raise self.printer.command_error(
                "PuppyBus: short response (%d bytes): %s" % (len(resp), resp.hex()))
        r_addr, status, dlen = resp[0], resp[1], resp[2]
        body = resp[3:3 + dlen]
        crc_rx = resp[3 + dlen] | (resp[3 + dlen + 1] << 8) \
            if len(resp) >= 3 + dlen + 2 else None
        crc_calc = crc16_ibm(resp[0:3 + dlen])
        if r_addr != addr:
            raise self.printer.command_error(
                "PuppyBus: addr mismatch (got 0x%02x want 0x%02x), raw %s"
                % (r_addr, addr, resp.hex()))
        if crc_rx is None or crc_rx != crc_calc:
            raise self.printer.command_error(
                "PuppyBus: CRC mismatch (rx 0x%04x calc 0x%04x), raw %s"
                % (crc_rx or 0, crc_calc, resp.hex()))
        return status, body

    # --- flash layer (bootloader WRITE/FINALIZE/READ) -----------------------
    def read_flash(self, addr, offset, length):
        d = bytes([(offset >> 24) & 0xff, (offset >> 16) & 0xff,
                   (offset >> 8) & 0xff, offset & 0xff, length])
        return self.bl_command(addr, BL_READ_FLASH, d)

    def write_flash_chunk(self, addr, offset, chunk):
        d = bytes([(offset >> 24) & 0xff, (offset >> 16) & 0xff,
                   (offset >> 8) & 0xff, offset & 0xff]) + bytes(chunk)
        # generous timeout: a chunk that crosses an 8 KB page triggers an erase
        return self.bl_command(addr, BL_WRITE_FLASH, d, read_timeout=300000)

    def finalize_flash(self, addr):
        # commits the final partial page (an erase) -> allow plenty of time
        return self.bl_command(addr, BL_FINALIZE_FLASH, b'', read_timeout=2000000)

    # --- gcode --------------------------------------------------------------
    def cmd_PUPPY_PROTO_VERSION(self, gcmd):
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        try:
            status, body = self.bl_command(addr, BL_GET_PROTOCOL_VERSION)
        except self.printer.command_error as e:
            gcmd.respond_info(str(e))
            return
        sname = STATUS_NAMES.get(status, "0x%02x" % status)
        if len(body) == 2:
            ver = (body[0] << 8) | body[1]
            gcmd.respond_info("PuppyBus addr 0x%02x: status=%s protocol=0x%04x"
                              % (addr, sname, ver))
        else:
            gcmd.respond_info("PuppyBus addr 0x%02x: status=%s data=%s"
                              % (addr, sname, body.hex()))

    def cmd_PUPPY_HWINFO(self, gcmd):
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        try:
            status, b = self.bl_command(addr, BL_GET_HARDWARE_INFO)
        except self.printer.command_error as e:
            gcmd.respond_info(str(e))
            return
        sname = STATUS_NAMES.get(status, "0x%02x" % status)
        if len(b) >= 11:
            hw_type = b[0]
            hw_rev = (b[1] << 8) | b[2]
            bl_ver = (b[3] << 24) | (b[4] << 16) | (b[5] << 8) | b[6]
            app_size = (b[7] << 24) | (b[8] << 16) | (b[9] << 8) | b[10]
            gcmd.respond_info(
                "PuppyBus addr 0x%02x: status=%s hw_type=%d hw_rev=%d "
                "bl_ver=0x%08x app_size=%d"
                % (addr, sname, hw_type, hw_rev, bl_ver, app_size))
        else:
            gcmd.respond_info("PuppyBus addr 0x%02x: status=%s data=%s"
                              % (addr, sname, b.hex()))

    def cmd_PUPPY_RAW(self, gcmd):
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        cmd = gcmd.get_int('CMD', minval=0, maxval=255)
        data_hex = gcmd.get('DATA', '')
        data = bytes.fromhex(data_hex) if data_hex else b''
        resp = self.txrx(bytes([addr, cmd]) + data)
        gcmd.respond_info("PuppyBus raw response (%d bytes): %s"
                          % (len(resp), resp.hex() if resp else "<none>"))

    def cmd_PUPPY_READ_FLASH(self, gcmd):
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        offset = gcmd.get_int('OFFSET', 0, minval=0)
        length = gcmd.get_int('LEN', 32, minval=1, maxval=FLASH_CHUNK)
        try:
            status, body = self.read_flash(addr, offset, length)
        except self.printer.command_error as e:
            gcmd.respond_info(str(e))
            return
        gcmd.respond_info("READ_FLASH off=0x%x len=%d status=%s: %s"
                          % (offset, length,
                             STATUS_NAMES.get(status, "0x%02x" % status), body.hex()))

    def cmd_PUPPY_FLASH_VERIFY(self, gcmd):
        # Prove the RS-485 flash path end-to-end: overwrite the start of the app
        # slot with a known pattern and read it back. This CORRUPTS the app's first
        # page (restore by re-flashing); erase_count > 0 confirms a real flash erase.
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        n = gcmd.get_int('BYTES', 128, minval=32, maxval=240)
        pattern = bytes(((i * 7 + 0x5a) & 0xff) for i in range(n))
        try:
            _, before = self.read_flash(addr, 0, 32)
            off = 0
            while off < n:
                chunk = pattern[off:off + FLASH_CHUNK]
                st, _ = self.write_flash_chunk(addr, off, chunk)
                if st != 0x00:
                    gcmd.respond_info("WRITE_FLASH off=%d FAILED status=0x%02x"
                                      % (off, st))
                    return
                off += len(chunk)
            st, fbody = self.finalize_flash(addr)
            erase_count = fbody[0] if fbody else -1
            _, after = self.read_flash(addr, 0, 32)
        except self.printer.command_error as e:
            gcmd.respond_info("flash verify error: " + str(e))
            return
        ok = (after == pattern[:32])
        gcmd.respond_info(
            "PUPPY_FLASH_VERIFY (%d bytes over RS-485):\n"
            "  before (app vectors) = %s\n"
            "  after                = %s\n"
            "  expected pattern     = %s\n"
            "  finalize status=0x%02x erase_count=%d  ->  %s"
            % (n, before.hex(), after.hex(), pattern[:32].hex(), st, erase_count,
               "PASS - RS-485 flash write CONFIRMED" if ok else "FAIL"))

    def cmd_PUPPY_FLASH(self, gcmd):
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        path = gcmd.get('FILE')
        try:
            data = open(path, 'rb').read()
        except Exception as e:
            gcmd.respond_info("cannot read %s: %s" % (path, e))
            return
        total = len(data)
        # Catch handshake. An auto-chainloading bootloader lingers only a few seconds, and the
        # first RS-485 frame after a fresh boot can be dropped by the xBuddy RX-pulldown framing
        # glitch. Probe GET_HARDWARE_INFO until it answers before streaming: the retries absorb
        # that first-frame glitch, and each received probe resets the bootloader's idle timer so
        # it stays in the bootloader instead of auto-jumping. Harmless on a wait-forever
        # bootloader (the first probe just succeeds immediately). CATCH_S=0 disables it.
        reactor = self.printer.get_reactor()
        catch_s = gcmd.get_float('CATCH_S', 2.5, minval=0.)
        deadline = reactor.monotonic() + catch_s
        while True:
            try:
                self.bl_command(addr, BL_GET_HARDWARE_INFO, b'', read_timeout=40000)
                break
            except self.printer.command_error:
                if reactor.monotonic() >= deadline:
                    gcmd.respond_info("PUPPY_FLASH: no bootloader caught at 0x%02x within "
                                      "%.1fs (power-cycle the puppy and retry)" % (addr, catch_s))
                    return
        gcmd.respond_info("PUPPY_FLASH: uploading %d bytes (%s) to addr 0x%02x "
                          "over RS-485..." % (total, path, addr))
        try:
            off = 0
            while off < total:
                chunk = data[off:off + FLASH_CHUNK]
                st, _ = self.write_flash_chunk(addr, off, chunk)
                if st != 0x00:
                    gcmd.respond_info("WRITE_FLASH FAILED at off=%d status=0x%02x"
                                      % (off, st))
                    return
                off += len(chunk)
                if off % 16384 < FLASH_CHUNK:
                    gcmd.respond_info("  ... %d / %d (%d%%)"
                                      % (off, total, off * 100 // total))
            st, fbody = self.finalize_flash(addr)
            ec = fbody[0] if fbody else -1
            _, head = self.read_flash(addr, 0, 32)
        except self.printer.command_error as e:
            gcmd.respond_info("flash error: " + str(e))
            return
        gcmd.respond_info(
            "PUPPY_FLASH done: %d bytes, finalize status=0x%02x erase_count=%d; "
            "readback[0:32] %s file"
            % (total, st, ec, "==" if head == data[:32] else "!="))

    def cmd_PUPPY_START(self, gcmd):
        # Make the bootloader validate + jump to the flashed app. The bootloader
        # hashes salt (uint32, native LE) then the whole app region; we mirror that
        # from the .bin (verified == flash). On the wire the salt is big-endian.
        addr = gcmd.get_int('ADDR', ADDR_XBE_BOOTLOADER, minval=0, maxval=255)
        path = gcmd.get('FILE')
        salt = gcmd.get_int('SALT', 0x6b6c6970)   # arbitrary 32-bit salt ('klip')
        try:
            app = open(path, 'rb').read()
        except Exception as e:
            gcmd.respond_info("cannot read %s: %s" % (path, e))
            return
        fp = hashlib.sha256(struct.pack('<I', salt) + app).digest()
        salt_be = struct.pack('>I', salt)
        try:
            st_c, _ = self.bl_command(addr, BL_COMPUTE_FINGERPRINT, salt_be,
                                      read_timeout=3000000)
            st_s, body = self.bl_command(addr, BL_START_APPLICATION, salt_be + fp,
                                         read_timeout=1000000)
        except self.printer.command_error as e:
            gcmd.respond_info("PUPPY_START error: " + str(e))
            return
        match = body[0] if body else -1
        gcmd.respond_info(
            "PUPPY_START salt=0x%08x app=%d B: compute=0x%02x start=0x%02x "
            "fingerprint_match=%d\n  -> %s"
            % (salt, len(app), st_c, st_s, match,
               "MATCH - H503 validated + JUMPING to klipper-puppy over RS-485"
               if match == 1 else
               "NO MATCH - bootloader parks (check salt/app vs flash)"))


def load_config(config):
    return PuppyBus(config)
