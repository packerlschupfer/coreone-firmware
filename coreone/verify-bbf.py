#!/usr/bin/env python3
# Verify a Core One Klipper BBF is contract-valid + Prusa-bootloader-installable.
# Catches silent breakage (wrong vector base, stale SHA, wrong printer type) before
# you waste a flash cycle. Run standalone or via pack-bbf.sh's self-verify gate.
#
# BBF layout (utils/pack_fw.py, bbf_version 2):
#   [64B signature][32B SHA256][480B header][firmware][optional TLV...]
#   header = [4B fw_len][10B version][1B board][1B printer_type]
#            [1B bbf_version][1B printer_subversion][1B printer_version][461B pad]
#   SHA256 = sha256(header + firmware)   # == pack_fw.py's bin_data_fw_only (excludes sig/SHA/TLV)
import sys, hashlib, struct

SIG, SHA, HDR = 64, 32, 480
HDR_OFF, FW_OFF = SIG + SHA, SIG + SHA + HDR

def main(path):
    bbf = open(path, 'rb').read()
    if len(bbf) < FW_OFF + 8:
        print("BBF too small"); return 1
    sig = bbf[0:SIG]
    sha_emb = bbf[SIG:SIG+SHA]
    hdr = bbf[HDR_OFF:FW_OFF]
    fw_len = struct.unpack('<I', hdr[0:4])[0]
    ptype, bbfver, psub, pver = hdr[15], hdr[16], hdr[17], hdr[18]
    fw = bbf[FW_OFF:FW_OFF + fw_len]
    sha_calc = hashlib.sha256(bbf[HDR_OFF:FW_OFF + fw_len]).digest()  # header + firmware
    msp, rst = struct.unpack('<II', fw[:8]) if len(fw) >= 8 else (0, 0)

    checks = [
        ("printer_type == 7 (COREONE)",        ptype == 7),
        ("bbf_version == 2",                    bbfver == 2),
        ("fw_len within file & non-empty",      0 < fw_len and FW_OFF + fw_len <= len(bbf)),
        ("descriptor SHA == sha256(hdr+fw)",    sha_emb == sha_calc),
        ("firmware vectors @ 0x08020200",       0x08020200 <= (rst & ~1) < 0x08040000
                                                and 0x20000000 <= msp < 0x20030000),
    ]
    bad = [n for n, ok in checks if not ok]
    for n, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {n}")
    if bad:
        print("BBF INVALID ->", ", ".join(bad)); return 1
    signed = set(sig) != {0}
    print(f"BBF OK: COREONE v{pver}.{psub}, firmware {fw_len} B @0x08020200, "
          f"{'SIGNED' if signed else 'unsigned (dev, bootloader-accepted)'}, "
          f"sha256(hdr+fw)={sha_emb.hex()[:16]}…")
    return 0

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: verify-bbf.py <file.bbf>"); sys.exit(2)
    sys.exit(main(sys.argv[1]))
