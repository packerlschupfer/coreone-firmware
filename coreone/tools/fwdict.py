#!/usr/bin/env python3
"""Read Klipper's embedded data dictionary out of a raw firmware .bin.

    tools/fwdict.py out/klipper.bin [more.bin ...]
    tools/fwdict.py --all firmware.bin        # dump every dictionary key

🚨 WHY THIS EXISTS -- IT SETTLES "WHICH BUILD IS THIS ARTIFACT?" FROM THE ARTIFACT
ALONE, with no surviving ELF, .map or build log. The dictionary is stored
zlib-COMPRESSED, so `strings` finds nothing, and every provenance question we spent a
week on is answered by two of its keys:

  version         the git describe of the tree that built it -- AND, for any non-clean
                  build, a second-resolution timestamp plus the build host
  build_versions  the exact gcc and binutils that compiled it

🔑 THE REPRODUCIBILITY TRAP IT EXPOSES. klipper's scripts/buildcommands.py:

      elif 'dirty' in version:
          cleanbuild = False
      if not cleanbuild:
          btime = time.strftime("%Y%m%d_%H%M%S")
          hostname = socket.gethostname()
          version = "%s-%s-%s" % (version, btime, hostname)

A DIRTY TREE STAMPS THE WALL CLOCK INTO THE BINARY. Such a build cannot byte-reproduce
-- not against another machine, not against itself one second later. So a hash
comparison between two dirty builds measures the clock, not the inputs, and chasing it
is chasing nothing.

That is exactly the shape of our H503 story. h5/build-h5.sh `git apply`s klipper-h5.patch
and friends into a pristine klipper clone, so EVERY overlay H503 build is dirty and
timestamped:

    v0.13.0-691-g04705ae1c-dirty-20260612_100155-bnice

while the fork carries that support COMMITTED and builds clean in-tree at a tag, so both
MCUs in the running printer report a bare `v0.13.0-core-one.1` with no stamp. Same
firmware family, opposite reproducibility, and the version string says which you have.

⚠ So: BEFORE comparing firmware hashes, run this on both files. If either version carries
`-dirty-<ts>-<host>`, the hashes were never going to match and the difference is not a
finding. Checking a version STRING for equality is not enough either -- a moved release
tag makes two different builds report the same `mcu_version`, which is why the timestamp
field and build_versions matter.
"""
import json, sys, zlib

ZLIB_CMF = 0x78
ZLIB_FLG = (0x01, 0x5e, 0x9c, 0xda)
MAX_INFLATE = 4 << 20
KEYS = ('app', 'version', 'build_versions', 'license')


def find_dicts(blob):
    """Every embedded JSON dictionary, as (offset, parsed). Scans for a zlib header
    and tries to inflate; a stream that does not yield JSON is simply not one."""
    out = []
    for i in range(len(blob) - 2):
        if blob[i] != ZLIB_CMF or blob[i + 1] not in ZLIB_FLG:
            continue
        try:
            txt = zlib.decompressobj().decompress(blob[i:], MAX_INFLATE)
        except Exception:
            continue
        if not txt.startswith(b'{'):
            continue
        try:
            out.append((i, json.loads(txt.decode('utf-8', 'replace'))))
        except Exception:
            pass
    return out


def main(argv):
    show_all = '--all' in argv
    paths = [a for a in argv if not a.startswith('-')]
    if not paths:
        sys.stderr.write(__doc__.split('\n\n')[1] + "\n")
        return 2
    rc = 0
    for path in paths:
        try:
            blob = open(path, 'rb').read()
        except OSError as e:
            print("== %s\n   CANNOT READ: %s" % (path, e))
            rc = 1
            continue
        found = find_dicts(blob)
        print("== %s  (%d bytes)" % (path, len(blob)))
        if not found:
            # Not an error: a bootloader image or a combined .bin may carry none.
            print("   no embedded dictionary found")
            rc = 1
            continue
        for off, d in found:
            print("   dictionary at offset 0x%06x" % off)
            for k in (sorted(d) if show_all else KEYS):
                if k in d and not isinstance(d[k], (dict, list)):
                    print("     %-15s %s" % (k, d[k]))
            ver = str(d.get('version', ''))
            if 'dirty' in ver:
                print("     ⚠ DIRTY BUILD -- the trailing -<timestamp>-<host> is stamped"
                      " from the wall clock, so this artifact CANNOT byte-reproduce."
                      " Do not read a hash difference against it as a finding.")
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
