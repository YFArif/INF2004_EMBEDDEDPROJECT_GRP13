#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_codec_test.sh -- S1-U05: host test of the minimum-bit message codec
# (app_program/subsystem1/msg_layout.h, msg_codec.h, msg_codec.c) against
# docs/S1_binary_frame.md.
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_codec_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem1/run_codec_test.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_msg_codec.txt
#
# Stages; the run stops at the first stage that fails:
#   1  Build codec_test_main.c, codec_vectors.c and msg_codec.c with strict
#      warnings as errors, UBSan and ASan.
#   2  Run the test; it must end with "RESULT: all passed".
#   3  golden.py --check: golden.py's own packing, the document's 10.2
#      table and codec_vectors.c agree on all 18 vectors.
#   4  Ten mutations, each on a copy of the codec in a temporary folder;
#      every one must be caught. Four must stop the build at a
#      compile-time check; six must build and make the test fail. Each
#      mutation matches on text, not line numbers, and the mutated copy is
#      compared with the original: "mutation did not apply" if nothing
#      changed, and exactly one changed line is required.
#   5  If arm-none-eabi-gcc is on the PATH, compile msg_codec.c for the
#      target with the profile's code-generation flags; any warning fails.
#      Otherwise the stage is skipped.
#
# The last line is "S1-U05: PASS (...)" with exit status 0, or
# "S1-U05: FAIL (...)" with exit status 1. Nothing in the repository is
# changed: every build output and mutated copy goes to a temporary
# directory that is removed at the end.
#
# Environment: CC (default cc), PYTHON (default python3), TMPDIR.

set -u

TEST_DIR=testing/subsystem1
CODEC_DIR=app_program/subsystem1
DOC=docs/S1_binary_frame.md

if [ ! -f "$CODEC_DIR/msg_codec.c" ] || [ ! -f "$TEST_DIR/codec_test_main.c" ]
then
    echo "run_codec_test.sh: run this from the repository root" >&2
    exit 2
fi

CC=${CC:-cc}
PYTHON=${PYTHON:-python3}
WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u05.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

WARN="-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
WARN="$WARN -Wshadow -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes"
SAN="-fsanitize=undefined,address -fno-sanitize-recover=all"
SAN="$SAN -fno-omit-frame-pointer"
TEST_SRC="$TEST_DIR/codec_test_main.c $TEST_DIR/codec_vectors.c"

fail ()
{
    echo "S1-U05: FAIL (stage $1)"
    exit 1
}

echo "S1-U05 run_codec_test.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
DIRTY=$(git status --porcelain -- "$CODEC_DIR" "$TEST_DIR" "$DOC" \
        2>/dev/null | grep -v '^?? .*logs/' | head -n 1)
if [ -n "$DIRTY" ]
then
    COMMIT="$COMMIT (tested files have uncommitted changes)"
fi
echo "commit: $COMMIT"
echo "cc: $($CC --version 2>&1 | head -n 1)"
echo "python: $($PYTHON --version 2>&1)"

# ---------------------------------------------------------------- stage 1
echo "== stage 1: build (strict warnings as errors, UBSan/ASan)"
if ! $CC $WARN -Werror -O1 -g $SAN -DTM_MQTT=1 -I"$CODEC_DIR" $TEST_SRC \
        "$CODEC_DIR/msg_codec.c" -o "$WORK/s1u05" 2>"$WORK/build.err"
then
    cat "$WORK/build.err"
    fail "1: build"
fi
echo "stage 1: ok, no warnings"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: run"
"$WORK/s1u05" >"$WORK/run.out" 2>&1
RC=$?
cat "$WORK/run.out"
CHECKS=$(sed -n 's/^RESULT: all passed (\([0-9]*\) checks)$/\1/p' \
         "$WORK/run.out")
if [ "$RC" -ne 0 ] || [ -z "$CHECKS" ]
then
    fail "2: test, exit status $RC"
fi
echo "stage 2: ok"

# ---------------------------------------------------------------- stage 3
echo "== stage 3: golden.py --check"
if ! "$PYTHON" -I "$TEST_DIR/golden.py" --check "$DOC" \
        "$TEST_DIR/codec_vectors.c"
then
    fail "3: golden.py --check"
fi
echo "stage 3: ok"

# ---------------------------------------------------------------- stage 4
echo "== stage 4: mutations"
CAUGHT=0
MISSED=0

# mutate ID KIND FILE SED_SCRIPT DESCRIPTION
#   KIND is "compile" (the build must stop at a compile-time check) or
#   "run" (the mutant must build and the test must fail).
mutate ()
{
    dir="$WORK/$1"
    mkdir -p "$dir"
    cp "$CODEC_DIR/msg_layout.h" "$CODEC_DIR/msg_codec.h" \
       "$CODEC_DIR/msg_codec.c" "$dir/"
    sed "$4" "$CODEC_DIR/$3" >"$dir/$3"

    if cmp -s "$CODEC_DIR/$3" "$dir/$3"
    then
        echo "$1 $5: mutation did not apply"
        MISSED=$((MISSED + 1))
        return
    fi

    changed=$(diff "$CODEC_DIR/$3" "$dir/$3" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $5: changed $changed lines, expected 1"
        MISSED=$((MISSED + 1))
        return
    fi

    if [ "$2" = compile ]
    then
        if $CC $WARN -O1 -DTM_MQTT=1 -c "$dir/msg_codec.c" \
                -o "$dir/msg_codec.o" 2>"$dir/err"
        then
            echo "$1 $5: NOT caught (the build succeeded)"
            MISSED=$((MISSED + 1))
        elif grep -q 'msg_codec_check_' "$dir/err"
        then
            echo "$1 $5: caught by a compile-time check"
            CAUGHT=$((CAUGHT + 1))
        else
            echo "$1 $5: build failed, but not at a compile-time check"
            cat "$dir/err"
            MISSED=$((MISSED + 1))
        fi
        return
    fi

    if ! $CC $WARN -O1 -g $SAN -DTM_MQTT=1 -I"$dir" $TEST_SRC \
            "$dir/msg_codec.c" -o "$dir/test" 2>"$dir/err"
    then
        echo "$1 $5: mutant did not build"
        cat "$dir/err"
        MISSED=$((MISSED + 1))
        return
    fi

    # Run in a subshell that records the exit status itself, so that a
    # sanitizer abort is reported only through that status and not as a
    # shell "Abort trap" message.
    ("$dir/test" >"$dir/out" 2>&1; echo "$?" >"$dir/rc") 2>/dev/null
    rc=$(cat "$dir/rc")
    if [ "$rc" -eq 0 ]
    then
        echo "$1 $5: NOT caught (the test passed)"
        MISSED=$((MISSED + 1))
    else
        how=$(grep -m 1 -o -E 'RESULT: [0-9]+ FAILED|AddressSanitizer: [a-z-]+|runtime error' \
              "$dir/out")
        echo "$1 $5: caught by the test ($how)"
        CAUGHT=$((CAUGHT + 1))
    fi
}

mutate C1 compile msg_layout.h \
    's/\(MSG_TEL_ROLL_POS  *\)115U/\1116U/' \
    "MSG_TEL_ROLL_POS 115 -> 116"
mutate C2 compile msg_layout.h \
    's/\(MSG_TEL_BYTES  *\)25U/\126U/' \
    "MSG_TEL_BYTES 25 -> 26"
mutate C3 compile msg_layout.h \
    's/\(MSG_EVENT_ARG_BYTES  *\)2U/\13U/' \
    "MSG_EVENT_ARG_BYTES 2 -> 3"
mutate C4 compile msg_layout.h \
    's/\(MSG_TEL_TURN_BITS  *\)10U/\117U/' \
    "MSG_TEL_TURN_BITS 10 -> 17"
mutate R1 run msg_codec.c \
    's/mask = 0x80U >> (bit_pos % 8U);/mask = 0x01U << (bit_pos % 8U);/' \
    "bit order reversed in put_field"
mutate R2 run msg_codec.c \
    '/^put_modulo (/,/^}/s/put_field(p_buf, pos, bits, value & field_max(bits));/put_unsigned(p_buf, pos, bits, value);/' \
    "put_modulo clamps instead of wrapping"
mutate R3 run msg_codec.c \
    '/^put_unsigned (/,/^}/s/(value > max) ? max : value/value/' \
    "put_unsigned clamp removed"
mutate R4 run msg_codec.c \
    '/^put_signed (/,/^}/s/(1U << (bits - 1U))/(1U << bits)/' \
    "put_signed offset 2^bits instead of 2^(bits-1)"
mutate R5 run msg_codec.c \
    's/\(CDEG_HALF_STEP  *\)25$/\124/' \
    "rounding half step 25 -> 24"
mutate R6 run msg_codec.c \
    '/^header_ok (/,/^}/s/(expected_len == len)/(expected_len == expected_len)/' \
    "header_ok length check always true"

echo "stage 4: $CAUGHT/10 mutations caught"
if [ "$CAUGHT" -ne 10 ] || [ "$MISSED" -ne 0 ]
then
    fail "4: mutations"
fi

# ---------------------------------------------------------------- stage 5
# The profile's code-generation flags, from build_make/pico_rp2040.mk (the
# base CFLAGS, -DNDEBUG -ffunction-sections -fdata-sections, and
# -DTM_MQTT=1 for MQTT=1). Its -I and board -D flags are left out because
# msg_codec.c includes only standard headers.
echo "== stage 5: target compile"
STAGE5=skipped
if command -v arm-none-eabi-gcc >/dev/null 2>&1
then
    ARM="-mcpu=cortex-m0plus -mthumb -ffreestanding -std=gnu11 -O2 -g3"
    ARM="$ARM -mfloat-abi=soft -DNDEBUG -ffunction-sections -fdata-sections"
    ARM="$ARM -DTM_MQTT=1"
    if ! arm-none-eabi-gcc $ARM -Wall -Wextra -Wconversion -Wshadow \
            -pedantic -c "$CODEC_DIR/msg_codec.c" -o "$WORK/msg_codec.o" \
            2>"$WORK/arm.err" || [ -s "$WORK/arm.err" ]
    then
        cat "$WORK/arm.err"
        fail "5: target compile"
    fi
    echo "$(arm-none-eabi-gcc --version | head -n 1)"
    arm-none-eabi-size "$WORK/msg_codec.o" | sed "s|$WORK/||"
    STAGE5=ok
    echo "stage 5: ok, no warnings"
else
    echo "stage 5 skipped (arm-none-eabi-gcc is not on the PATH)"
fi

echo "S1-U05: PASS ($CHECKS checks, 18/18 vectors agree," \
     "$CAUGHT/10 mutations caught, target compile $STAGE5)"
exit 0
