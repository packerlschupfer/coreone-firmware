#!/usr/bin/env bash
# Run EVERY gate in this directory. One command, so the set cannot drift from whoever is running it.
#
# WHY THIS EXISTS
# ---------------
# There was no runner, so each session improvised `for t in *_test.py`. That glob SILENTLY OMITS
# golden.py -- which is not a helper but the original and arguably most important gate here: it
# compares the MCU-side shaping evaluator against a host reference to within a few ULP across a
# dozen timelines, and it exits non-zero on failure (`sys.exit(main())`).
#
# On 2026-10-31 I ran `python3 golden.py`, saw rc=0, and concluded it "asserts nothing and exits 0
# silently" -- reading the ABSENCE OF A FAILURE SIGNAL as the absence of a check. rc=0 meant it
# PASSED. I then reported the suite as 16/16 and later 17/17 and told a peer their count of 18 was
# wrong. It was not; mine was. A glob that quietly drops a gate is exactly the kind of defence that
# stops protecting you without saying so (memory: defences_must_be_tested_independently).
#
# So: this script enumerates gates EXPLICITLY as "golden.py plus *_test.py", prints the count it
# intends to run before running it, and fails if that count is zero.
#
#   ./run_all.sh            # run everything, summarise
#   ./run_all.sh -q         # only print failures and the summary
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

QUIET=0
[ "${1:-}" = "-q" ] && QUIET=1

GATES=(golden.py)
for t in *_test.py; do
    [ -e "$t" ] || continue
    GATES+=("$t")
done

n=${#GATES[@]}
if [ "$n" -le 1 ]; then
    echo "run_all: found no *_test.py next to golden.py -- wrong directory?" >&2
    exit 2
fi
echo "shaper_golden: running $n gate(s)  (golden.py + $((n - 1)) *_test.py)"
echo

pass=0
fail=0
failed=()
for g in "${GATES[@]}"; do
    out=$(timeout 600 python3 -I "$g" 2>&1)
    rc=$?
    if [ $rc -eq 0 ]; then
        pass=$((pass + 1))
        [ "$QUIET" = 0 ] && printf '  PASS  %s\n' "$g"
    else
        fail=$((fail + 1))
        failed+=("$g")
        printf '  FAIL  %s (rc=%d)\n' "$g" "$rc"
        echo "$out" | tail -12 | sed 's/^/        /'
    fi
done

echo
echo "=================================================="
printf 'shaper_golden: %d/%d passed' "$pass" "$n"
if [ "$fail" -gt 0 ]; then
    printf ', %d FAILED\n' "$fail"
    printf '  failing:'
    printf ' %s' "${failed[@]}"
    echo
    exit 1
fi
echo " -- all gates green"
