#!/usr/bin/env bash
# Pack the built Klipper bin into an unsigned Core One BBF the Prusa bootloader
# can install via USB. See ../docs/klipper_flash_procedure.md (strategy C).
#   COREONE => --printer-type 7 --printer-version 1 --printer-subversion 0
# pack_fw.py needs the `ecdsa` package. Prefer a repo-root .venv; fall back to
# system python3. Set up once with:
#   python3 -m venv .venv && .venv/bin/pip install -r coreone/requirements.txt
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUDDY="$(cd "$HERE/.." && pwd)"
BIN="${1:-$HERE/out/klipper.bin}"
VERSION="${FW_VERSION:-1.0.0+1}"
PY="$BUDDY/.venv/bin/python"
[ -x "$PY" ] || PY=python3

if ! "$PY" -c 'import ecdsa' 2>/dev/null; then
  echo "ERROR: $PY lacks the 'ecdsa' package that utils/pack_fw.py needs." >&2
  echo "  Set up a venv at the repo root:" >&2
  echo "    python3 -m venv .venv && .venv/bin/pip install -r coreone/requirements.txt" >&2
  exit 1
fi

"$PY" "$BUDDY/utils/pack_fw.py" "$BIN" \
    --version "$VERSION" \
    --printer-type 7 --printer-version 1 --printer-subversion 0 \
    --no-sign

# pack_fw.py writes <bin-basename>.bbf next to the input
BBF="${BIN%.bin}.bbf"
echo "BBF: $BBF"

# Self-verify the packed BBF against the bootloader contract (vector base @0x08020200,
# SHA over header+firmware, printer type 7). Fails loud rather than letting a silently
# bad BBF (wrong variant built, stale SHA) reach a flash attempt.
echo "=== verifying BBF ==="
python3 "$HERE/verify-bbf.py" "$BBF"
