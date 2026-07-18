#!/usr/bin/env bash
# Out-of-band SOFT ABORT — the ONLY way to interrupt a macro that holds the gcode mutex.
#
# WHY THIS EXISTS. `SOFT_ABORT` as a GCODE COMMAND is itself queued gcode, so while a long
# macro is running it sits behind that macro and never executes. soft_cancel.py says so in its
# own registration ("Mutex-bound helpers (only useful when idle / for clearing the flag)...
# out-of-band callers should use the soft_cancel/abort webhook") — but nothing in the tree ever
# called that webhook, so in practice the out-of-band path did not exist.
#
# Found the hard way 2026-09-26: LOADCELL_PA_PROBE was started at the wrong Z, `SOFT_ABORT` was
# sent via Moonraker, it queued, the sweep kept extruding, and it took an emergency stop (which
# shuts the MCU down and needs a FIRMWARE_RESTART) to stop a mess that a soft abort should have
# handled cleanly.
#
# The webhook is dispatched from the reactor and NOT under the gcode mutex — the same path
# emergency_stop uses — so it can flip the flag while a blocking macro still holds the lock.
# Klippy's UDS protocol is one JSON object per message, terminated by 0x03.
#
#   ./soft-abort.sh              # abort
#   ./soft-abort.sh --info       # list endpoints (proves the socket works, changes nothing)
set -euo pipefail

SOCK="${KLIPPY_UDS:-$HOME/printer_data/comms/klippy.sock}"
METHOD="soft_cancel/abort"
[ "${1:-}" = "--info" ] && METHOD="info"

[ -S "$SOCK" ] || { echo "no klippy socket at $SOCK (set KLIPPY_UDS=)" >&2; exit 1; }

python3 - "$SOCK" "$METHOD" <<'PY'
import json, socket, sys
sock_path, method = sys.argv[1], sys.argv[2]
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(5.0)
s.connect(sock_path)
s.sendall(json.dumps({"id": 1, "method": method, "params": {}}).encode() + b"\x03")
buf = b""
try:
    while b"\x03" not in buf:
        chunk = s.recv(4096)
        if not chunk:
            break
        buf += chunk
except socket.timeout:
    print("timed out waiting for a reply (the abort may still have landed)", file=sys.stderr)
    sys.exit(2)
for raw in buf.split(b"\x03"):
    if not raw.strip():
        continue
    msg = json.loads(raw)
    if msg.get("id") == 1:
        # klippy answers {"id":1,"result":...} or {"id":1,"error":...}
        if "error" in msg:
            print("ERROR:", msg["error"], file=sys.stderr); sys.exit(1)
        print("ok:", json.dumps(msg.get("result"))[:300])
        sys.exit(0)
print("no reply to id=1; raw:", buf[:200], file=sys.stderr)
sys.exit(3)
PY
