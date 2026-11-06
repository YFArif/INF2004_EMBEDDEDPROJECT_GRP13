#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_decoder_test.sh -- S1-U11: host test of the dashboard decoder
# app_program/subsystem1/dashboard/s1_decode.py against the layout and the
# golden vectors of docs/S1_binary_frame.md.
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_decoder_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem1/run_decoder_test.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_decoder.txt
#
# Stages; the run stops at the first stage that fails:
#   1  golden.py --check: the 18 vectors in codec_vectors.c, which the
#      decoder test reads, still agree with the document's 10.2 table and
#      with golden.py's own packing.
#   2  decoder_test.py: the 18 golden vectors decode to exactly the values
#      msg_codec_get_*() must return; malformed input (empty, every type
#      one byte short and one byte long, versions 0, 2 and 15, types 0, 6
#      and 15); padding ignored; count reconstruction (10.1); lost frames
#      from seq; describe(); the field table against msg_layout.h; the
#      command line.
#   3  Four mutations, each on a copy of s1_decode.py in a temporary
#      folder; each must make stage 2 fail. Each mutation matches on text
#      and the copy is compared with the original ("mutation did not
#      apply" if nothing changed; exactly one changed line is required).
#
# The last line is "S1-U11: PASS (...)" with exit status 0, or
# "S1-U11: FAIL (...)" with exit status 1. Nothing in the repository is
# changed: Python runs with -B (no byte-code files), and the mutated
# copies go to a temporary folder that is removed at the end.
#
# Environment: PYTHON (default python3), TMPDIR.

set -u

TEST_DIR=testing/subsystem1
DEC_DIR=app_program/subsystem1/dashboard
DOC=docs/S1_binary_frame.md
PYTHON=${PYTHON:-python3}

if [ ! -f "$DEC_DIR/s1_decode.py" ] || [ ! -f "$TEST_DIR/decoder_test.py" ]
then
    echo "run_decoder_test.sh: run this from the repository root" >&2
    exit 2
fi

WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u11.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

fail ()
{
    echo "S1-U11: FAIL (stage $1)"
    exit 1
}

echo "S1-U11 run_decoder_test.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
DIRTY=$(git status --porcelain -- "$DEC_DIR" "$TEST_DIR" 2>/dev/null \
        | grep -v '^?? .*logs/' | head -n 1)
if [ -n "$DIRTY" ]
then
    COMMIT="$COMMIT (tested files have uncommitted changes)"
fi
echo "commit: $COMMIT"
echo "python: $("$PYTHON" --version 2>&1)"

# ---------------------------------------------------------------- stage 1
echo "== stage 1: golden.py --check"
if ! "$PYTHON" -I -B "$TEST_DIR/golden.py" --check "$DOC" \
        "$TEST_DIR/codec_vectors.c"
then
    fail "1: golden.py --check"
fi
echo "stage 1: ok"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: decoder_test.py"
"$PYTHON" -I -B "$TEST_DIR/decoder_test.py" >"$WORK/run.out" 2>&1
RC=$?
cat "$WORK/run.out"
if [ "$RC" -ne 0 ] || ! tail -n 1 "$WORK/run.out" | grep -q '^OK$'
then
    fail "2: decoder test, exit status $RC"
fi
TESTS=$(grep -E '^Ran [0-9]+ tests? in' "$WORK/run.out" | awk '{print $2}')
echo "stage 2: ok"

# ---------------------------------------------------------------- stage 3
echo "== stage 3: mutations"
CAUGHT=0
MISSED=0

# mutate ID SED_SCRIPT DESCRIPTION -- a copy of s1_decode.py changed by
# SED_SCRIPT must make decoder_test.py fail.
mutate ()
{
    dir="$WORK/$1"
    mkdir -p "$dir"
    sed "$2" "$DEC_DIR/s1_decode.py" >"$dir/s1_decode.py"

    if cmp -s "$DEC_DIR/s1_decode.py" "$dir/s1_decode.py"
    then
        echo "$1 $3: mutation did not apply"
        MISSED=$((MISSED + 1))
        return
    fi

    changed=$(diff "$DEC_DIR/s1_decode.py" "$dir/s1_decode.py" \
              | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $3: changed $changed lines, expected 1"
        MISSED=$((MISSED + 1))
        return
    fi

    S1_DECODE_DIR="$dir" "$PYTHON" -I -B "$TEST_DIR/decoder_test.py" \
        >"$dir/out" 2>&1
    if [ "$?" -eq 0 ]
    then
        echo "$1 $3: NOT caught (the test passed)"
        MISSED=$((MISSED + 1))
    else
        echo "$1 $3: caught ($(tail -n 1 "$dir/out"))"
        CAUGHT=$((CAUGHT + 1))
    fi
}

mutate M1 's/int\.from_bytes(data, "big")/int.from_bytes(data, "little")/' \
    "bit stream read little-endian"
mutate M2 's/return raw - (1 << (width - 1))/return raw/' \
    "signed fields without the offset subtracted"
mutate M3 's/if version != LAYOUT_VERSION:/if version > LAYOUT_VERSION:/' \
    "version 0 accepted"
mutate M4 's/len(data) != LENGTHS/len(data) < LENGTHS/' \
    "longer messages accepted"

echo "stage 3: $CAUGHT/4 mutations caught"
if [ "$CAUGHT" -ne 4 ] || [ "$MISSED" -ne 0 ]
then
    fail "3: mutations"
fi

echo "S1-U11: PASS (golden.py --check ok, $TESTS decoder tests passed," \
     "$CAUGHT/4 mutations caught)"
exit 0
