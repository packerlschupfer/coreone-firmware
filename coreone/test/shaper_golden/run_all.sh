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
# README COVERAGE
# ---------------
# On 2026-11-02 the README's gate table was found to document 6 of 22 gates, and to be stale
# ASYMMETRICALLY between the overlay and the fork (one copy carried a row the other lacked), so
# neither copy could be used to repair the other. A file that calls itself the map of the suite
# while being wrong about two thirds of it is worse than no map: it gets trusted once.
#
# The fix is not writing the rows -- rows rot. It is this check. A list maintained by hand beside
# a list maintained by the filesystem WILL diverge, so the hand-kept one has to be gated by the
# filesystem one. Same lesson as the `*_test.py` glob above, one directory up the stack.
#
# It keys on gate FILENAMES, which are identical in the overlay and the fork (only the enclosing
# directory differs, and that is resolved from this script's own location). So it needs no layout
# tolerance, and it deliberately has none: the one README row that differed between the two trees
# was not a layout seam but a stale copy, and tolerating it would have blinded the check to the
# only drift we had evidence of. A path that differs because the tree differs is a seam; a fact
# one copy got wrong is a defect.
#
#   ./run_all.sh            # run everything, summarise
#   ./run_all.sh -q         # only print failures and the summary
#   ./run_all.sh --docs     # the README coverage check alone, no gates
#
# Exit: 0 green | 1 a gate failed | 2 no gates found | 3 gates green but README drifted
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

QUIET=0
DOCS_ONLY=0
case "${1:-}" in
    -q)     QUIET=1 ;;
    --docs) DOCS_ONLY=1 ;;
    '')     ;;
    *)      echo "run_all: unknown argument '$1' (want -q or --docs)" >&2; exit 2 ;;
esac

# --- README coverage -------------------------------------------------------
# Every *_test.py must be named in backticks somewhere in README.md, and every *_test.py named
# there must exist. golden.py is exempt from the table: it is the oracle, described in prose.
check_readme_coverage() {
    local readme=README.md f n
    if [ ! -f "$readme" ]; then
        echo "  DRIFT  README.md is missing entirely" >&2
        return 1
    fi
    local -a undocumented=() stale=()
    for f in *_test.py; do
        [ -e "$f" ] || continue
        grep -qF -- "\`$f\`" "$readme" || undocumented+=("$f")
    done
    while read -r n; do
        [ -n "$n" ] || continue
        [ -e "$n" ] || stale+=("$n")
    done < <(grep -oE '`[A-Za-z0-9_]+_test\.py`' "$readme" | tr -d '`' | sort -u)

    if [ ${#undocumented[@]} -eq 0 ] && [ ${#stale[@]} -eq 0 ]; then
        return 0
    fi
    if [ ${#undocumented[@]} -gt 0 ]; then
        printf '  DRIFT  %d gate(s) have NO README row:\n' "${#undocumented[@]}" >&2
        printf '           %s\n' "${undocumented[@]}" >&2
    fi
    if [ ${#stale[@]} -gt 0 ]; then
        printf '  DRIFT  %d README row(s) name a gate that does not exist:\n' "${#stale[@]}" >&2
        printf '           %s\n' "${stale[@]}" >&2
    fi
    echo "         Add or remove the row in README.md's gate table. This check exists because" >&2
    echo "         that table had silently drifted to 6 of 22 (see the header)." >&2
    return 1
}

if [ "$DOCS_ONLY" = 1 ]; then
    if check_readme_coverage; then
        echo "shaper_golden: README documents every gate, and no row is stale"
        exit 0
    fi
    exit 3
fi

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

# Gates green is not the whole bar: the README must still describe the set that just ran.
if ! check_readme_coverage; then
    echo "shaper_golden: gates GREEN but the README gate table has DRIFTED -- not clean" >&2
    exit 3
fi
[ "$QUIET" = 0 ] && echo "shaper_golden: README documents all $((n - 1)) *_test.py gate(s)"
exit 0
