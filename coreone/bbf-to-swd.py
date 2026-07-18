#!/usr/bin/env python3
# Convert a Core One Klipper BBF -> the raw [descriptor][firmware] flash image to
# SWD-write at 0x08020000 (Strategy B, all-SWD bootloader coexist).
#
# A BBF stores:   [64B sig][32B SHA][480B header][firmware]
# The IN-FLASH layout @0x08020000 is simply the BBF MINUS the 64B signature:
#   [32B SHA][480B header] = 512B descriptor, then firmware @0x08020200.
# (Verified 2026-06-12 by reading back what the bootloader itself wrote: the SHA is
# at descriptor offset 0x000, the header at 0x020 -- NOT header-then-SHA, which was a
# wrong guess in the original reverse-engineering and got the first SWD flash rejected
# with #31608.) The signature is NOT flashed (unsigned accepted via the appendix/PA13).
import sys, hashlib, struct

SIG, SHA, HDR = 64, 32, 480
SHA_OFF, HDR_OFF, FW_OFF = SIG, SIG + SHA, SIG + SHA + HDR  # 64, 96, 576

def main(bbf_path, out_path):
    bbf = open(bbf_path, 'rb').read()
    sha = bbf[SHA_OFF:SHA_OFF + SHA]              # 32B SHA   (bbf[64:96])
    hdr = bbf[HDR_OFF:FW_OFF]                     # 480B header (bbf[96:576])
    fw_len = struct.unpack('<I', hdr[0:4])[0]
    fw = bbf[FW_OFF:FW_OFF + fw_len]             # firmware  (bbf[576:576+fw_len])

    descriptor = sha + hdr                       # in-flash order: SHA THEN header = 512B (= BBF minus sig)
    image = descriptor + fw                      # [512B descriptor][firmware] @0x08020000

    ptype = hdr[15]
    sha_calc = hashlib.sha256(hdr + fw).digest()
    msp, rst = struct.unpack('<II', fw[:8])
    checks = [
        ("descriptor is 512 B",               len(descriptor) == 512),
        ("printer_type == 7 (COREONE)",       ptype == 7),
        ("BBF SHA == sha256(hdr+fw)",         sha == sha_calc),
        ("flash desc SHA @0x000 correct",     image[0:32] == sha_calc),
        ("firmware vectors @0x08020200",      0x08020200 <= (rst & ~1) < 0x08040000
                                              and 0x20000000 <= msp < 0x20030000),
    ]
    for n, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {n}")
    if not all(ok for _, ok in checks):
        print("REFUSING to emit a bad flash image"); return 1
    open(out_path, 'wb').write(image)
    print(f"OK -> {out_path}")
    print(f"  {len(image)} B = 512B descriptor + {fw_len}B firmware, write to 0x08020000")
    print(f"  vectors MSP=0x{msp:08X} reset=0x{rst:08X}  sha256(hdr+fw)={sha.hex()[:16]}…")
    return 0

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: bbf-to-swd.py <in.bbf> <out-flash-image.bin>"); sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
