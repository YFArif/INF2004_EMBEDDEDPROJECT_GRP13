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
#   1  golden.py --check: the 25 vectors in codec_vectors.c (21, plus
#      the 4 MAP vectors of S1-U24), which the decoder test reads, still
#      agree with the document's 10.2 table and with golden.py's own
#      packing.
#   2  decoder_test.py: the 25 golden vectors decode to exactly the values
#      msg_codec_get_*() must return; malformed input (empty, every type
#      one byte short and one byte long, versions 0, 2 and 15, types 0, 8
#      and 15); padding ignored; count reconstruction (10.1); lost frames
#      from seq; describe(); the field table against msg_layout.h; the
#      command line; the counter baseline restart (rebase_live). The class
#      MapMessages is S1-U24 part B: the MAP message, its ack's names and
#      cmd_rtt.map_bytes().
#   3  Nine mutations, each on a copy of s1_decode.py in a temporary
#      folder; each must make stage 2 fail. M1 to M6 are S1-U11's; N1 to
#      N3 are S1-U24 part B's. Each mutation matches on text and the copy
#      is compared with the original ("mutation did not apply" if nothing
#      changed; exactly one changed line is required).
#   4  S1-U24's drift guard: build code39_dump.c with S3's own
#      app_program/subsystem3/s3_code39.c (strict warnings as errors) and
#      compare s3_code39_char(i), i = 0 to 63, with the CODE39 copies in
#      s1_decode.py and cmd_rtt.py: the 44 symbols, then '?'. Then two
#      guard mutations, G1 and G2, each swapping two symbols in a copy of
#      one of those tables, must each make the guard fail.
#
# The last line is "S1-U11: PASS (...)" with exit status 0, or
# "S1-U11: FAIL (...)" with exit status 1. Nothing in the repository is
# changed: Python runs with -B (no byte-code files), and the mutated
# copies go to a temporary folder that is removed at the end.
#
# Environment: PYTHON (default python3), CC (default cc), TMPDIR.

set -u

TEST_DIR=testing/subsystem1
DEC_DIR=app_program/subsystem1/dashboard
DOC=docs/S1_binary_frame.md
PYTHON=${PYTHON:-python3}
CC=${CC:-cc}
S3_SRC=app_program/subsystem3/s3_code39.c
WARN="-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
WARN="$WARN -Wshadow -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes"

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
mutate M5 's/self\._full\.pop("live_" + name, None)/pass/' \
    "rebase_live() keeps the counter baseline"
mutate M6 's/self\._last_seq\.clear()/pass/' \
    "rebase_robot() keeps the seq baselines"

# S1-U24 part B: the MAP message.
mutate N1 's/LIVE: 14, MAP: 5}/LIVE: 14, MAP: 6}/' \
    "MAP length 6"
mutate N2 's/("letter_left", 14, 6, UNSIGNED),/("letter_left", 20, 6, UNSIGNED),/' \
    "LEFT read at RIGHT's offset"
mutate N3 's/shown\["opcode_name"\] = "MAP"/shown["opcode_name"] = "INVALID"/' \
    "an ack with opcode 0 named INVALID again"

echo "stage 3: $CAUGHT/9 mutations caught (M1 to M6, N1 to N3)"
if [ "$CAUGHT" -ne 9 ] || [ "$MISSED" -ne 0 ]
then
    fail "3: mutations"
fi

# ---------------------------------------------------------------- stage 4
echo "== stage 4: drift guard (S3's Code 39 table, S1-U24)"
if ! $CC $WARN -Werror -O1 -I"$DEC_DIR/.." "$TEST_DIR/code39_dump.c" \
        "$S3_SRC" -o "$WORK/code39_dump" 2>"$WORK/dump.err"
then
    cat "$WORK/dump.err"
    fail "4: build code39_dump"
fi
S3_HEX=$("$WORK/code39_dump")
echo "cc: $($CC --version 2>&1 | head -n 1)"

# guard DECODER_DIR CMD_RTT_DIR -- exit 0 if both CODE39 copies match S3.
guard ()
{
    "$PYTHON" -I -B - "$S3_HEX" "$1" "$2" <<'EOF'
import sys
hex_text, dec_dir, cmd_dir = sys.argv[1:4]
sys.path.insert(0, cmd_dir)
sys.path.insert(0, dec_dir)
import cmd_rtt
import s1_decode
s3 = bytes.fromhex(hex_text).decode("ascii")
table, rest = s3[:44], s3[44:]
bad = 0
if (len(s3) != 64) or (rest != "?" * 20):
    print("guard: S3 gives %d symbols, then %r" % (len(s3), rest))
    bad += 1
for name, copy in (("s1_decode.CODE39", s1_decode.CODE39),
                   ("cmd_rtt.CODE39", cmd_rtt.CODE39)):
    diff = [i for i in range(44) if (i >= len(copy)) or (copy[i] != table[i])]
    if (len(copy) != 44) or diff:
        print("guard: %s differs from S3 at indices %s" % (name, diff))
        bad += 1
    else:
        print("guard: %s matches S3 on all 44 indices" % name)
sys.exit(1 if bad else 0)
EOF
}

if ! guard "$DEC_DIR" "$TEST_DIR"
then
    fail "4: drift guard"
fi
echo "guard: s3_code39_char() gives '?' for 44 to 63"

GUARD_CAUGHT=0
# guard_mutate ID FILE_DIR FILE DESCRIPTION -- a copy of FILE with '-' and
# '.' swapped in its CODE39 table must make the guard fail.
guard_mutate ()
{
    dir="$WORK/$1"
    mkdir -p "$dir"
    sed 's/^CODE39 = "\(.*\)-\.\(.*\)"$/CODE39 = "\1.-\2"/' "$2/$3" \
        >"$dir/$3"
    changed=$(diff "$2/$3" "$dir/$3" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $4: changed $changed lines, expected 1"
        return
    fi
    if [ "$3" = s1_decode.py ]
    then
        guard "$dir" "$TEST_DIR" >"$dir/out" 2>&1
    else
        guard "$DEC_DIR" "$dir" >"$dir/out" 2>&1
    fi
    if [ "$?" -eq 0 ]
    then
        echo "$1 $4: NOT caught (the guard passed)"
    else
        echo "$1 $4: caught ($(grep -m 1 differs "$dir/out"))"
        GUARD_CAUGHT=$((GUARD_CAUGHT + 1))
    fi
}

guard_mutate G1 "$DEC_DIR" s1_decode.py "s1_decode.CODE39 with - and . swapped"
guard_mutate G2 "$TEST_DIR" cmd_rtt.py "cmd_rtt.CODE39 with - and . swapped"
echo "stage 4: $GUARD_CAUGHT/2 guard mutations caught"
if [ "$GUARD_CAUGHT" -ne 2 ]
then
    fail "4: guard mutations"
fi

echo "S1-U11: PASS (golden.py --check ok, $TESTS decoder tests passed," \
     "$CAUGHT/9 mutations caught, drift guard ok, $GUARD_CAUGHT/2 guard" \
     "mutations caught)"
exit 0
