#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_telemetry_test.sh -- S1-U14: host test of telemetry_task's logic
# (app_program/subsystem1/s1_telem.h, s1_telem.c) with a stub in place of
# subsystem5_command_post().
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_telemetry_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem1/run_telemetry_test.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_telemetry.txt
#
# Stages; the run stops at the first stage that fails:
#   1  Build s1_telem_test.c, s1_telem.c and msg_codec.c with strict
#      warnings as errors, UBSan and ASan.
#   2  Run the test; it must end with "RESULT: all passed".
#   3  Eight mutations, each on a copy of s1_telem in a temporary folder;
#      every one must be caught. One must stop the build at a compile-time
#      check; seven must build and make the test fail. Each mutation
#      matches on text, and exactly one line must change.
#   4  If arm-none-eabi-gcc is on the PATH, compile s1_telem.c for the
#      target with the profile's code-generation flags; any warning fails.
#      Otherwise the stage is skipped.
#
# The last line is "S1-U14: PASS (...)" with exit status 0, or
# "S1-U14: FAIL (...)" with exit status 1. Nothing in the repository is
# changed: every build output and mutated copy goes to a temporary
# directory that is removed at the end.
#
# Environment: CC (default cc), TMPDIR.

set -u

TEST_DIR=testing/subsystem1
SRC_DIR=app_program/subsystem1

if [ ! -f "$SRC_DIR/s1_telem.c" ] || [ ! -f "$TEST_DIR/s1_telem_test.c" ]
then
    echo "run_telemetry_test.sh: run this from the repository root" >&2
    exit 2
fi

CC=${CC:-cc}
WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u14.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

WARN="-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
WARN="$WARN -Wshadow -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes"
SAN="-fsanitize=undefined,address -fno-sanitize-recover=all"
SAN="$SAN -fno-omit-frame-pointer"

fail ()
{
    echo "S1-U14: FAIL (stage $1)"
    exit 1
}

echo "S1-U14 run_telemetry_test.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
DIRTY=$(git status --porcelain -- "$SRC_DIR" "$TEST_DIR" 2>/dev/null \
        | grep -v '^?? .*logs/' | head -n 1)
if [ -n "$DIRTY" ]
then
    COMMIT="$COMMIT (tested files have uncommitted changes)"
fi
echo "commit: $COMMIT"
echo "cc: $($CC --version 2>&1 | head -n 1)"

# ---------------------------------------------------------------- stage 1
echo "== stage 1: build (strict warnings as errors, UBSan/ASan)"
if ! $CC $WARN -Werror -O1 -g $SAN -DTM_MQTT=1 -I"$SRC_DIR" \
        "$TEST_DIR/s1_telem_test.c" "$SRC_DIR/s1_telem.c" \
        "$SRC_DIR/msg_codec.c" -o "$WORK/s1u14" 2>"$WORK/build.err"
then
    cat "$WORK/build.err"
    fail "1: build"
fi
echo "stage 1: ok, no warnings"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: run"
"$WORK/s1u14" >"$WORK/run.out" 2>&1
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
echo "== stage 3: mutations"
CAUGHT=0
MISSED=0

# mutate ID KIND FILE SED_SCRIPT DESCRIPTION
#   KIND is "compile" (the build must stop at a compile-time check) or
#   "run" (the mutant must build and the test must fail).
mutate ()
{
    dir="$WORK/$1"
    mkdir -p "$dir"
    cp "$SRC_DIR/s1_telem.h" "$SRC_DIR/s1_telem.c" "$dir/"
    sed "$4" "$SRC_DIR/$3" >"$dir/$3"
    if cmp -s "$SRC_DIR/$3" "$dir/$3"
    then
        echo "$1 $5: mutation did not apply"
        MISSED=$((MISSED + 1))
        return
    fi
    changed=$(diff "$SRC_DIR/$3" "$dir/$3" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $5: changed $changed lines, expected 1"
        MISSED=$((MISSED + 1))
        return
    fi
    if [ "$2" = compile ]
    then
        if $CC $WARN -O1 -DTM_MQTT=1 -I"$dir" -I"$SRC_DIR" \
                -c "$dir/s1_telem.c" -o "$dir/s1_telem.o" 2>"$dir/err"
        then
            echo "$1 $5: NOT caught (the build succeeded)"
            MISSED=$((MISSED + 1))
        elif grep -q 's1_telem_check_' "$dir/err"
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
    if ! $CC $WARN -O1 -g $SAN -DTM_MQTT=1 -I"$dir" -I"$SRC_DIR" \
            "$TEST_DIR/s1_telem_test.c" "$dir/s1_telem.c" \
            "$SRC_DIR/msg_codec.c" -o "$dir/test" 2>"$dir/err"
    then
        echo "$1 $5: mutant did not build"
        cat "$dir/err"
        MISSED=$((MISSED + 1))
        return
    fi
    # Run in a subshell that records the exit status itself, so that a
    # sanitizer abort is reported only through that status.
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

mutate C1 compile s1_telem.h \
    's/\(#define S1_TELEM_PUB_MAX  *\)MSG_HB_BYTES/\1MSG_ACK_BYTES/' \
    "payload room cut to the ack's size"
mutate M1 run s1_telem.c \
    's/ack.result = MSG_ACK_ACCEPTED;/ack.result = MSG_ACK_REJECTED;/' \
    "a queued command acked as rejected"
mutate M2 run s1_telem.c \
    's/if (!b_valid)$/if (false)/' \
    "codec rejects posted to S5"
mutate M3 run s1_telem.c \
    's/^            p_in->command_dropped);/            0U);/' \
    "command_dropped left out of cmd_reject"
mutate M4 run s1_telem.c \
    's/\*p_next_msec += ((late_msec \/ S1_TELEM_HB_PERIOD_MSEC) + 1U)/*p_next_msec = now_msec + ((0U * late_msec) + 1U)/' \
    "relative heartbeat deadline instead of absolute"
mutate M5 run s1_telem.c \
    's/p_ack->topic = MQTT_LINK_TOPIC_ACK;/p_ack->topic = MQTT_LINK_TOPIC_EVENT;/' \
    "ack published on the event topic"
mutate M6 run s1_telem.c \
    's/return ((MQTT_LINK_STATE_CONNECTED == state) && (!b_stale));/return (MQTT_LINK_STATE_CONNECTED == state);/' \
    "heartbeat published while the snapshot is stale"
mutate M7 run s1_telem.c \
    's/p_hb->qos = MQTT_LINK_QOS_0;/p_hb->qos = MQTT_LINK_QOS_1;/' \
    "heartbeat sent at QoS 1"

echo "stage 3: $CAUGHT/8 mutations caught"
if [ "$CAUGHT" -ne 8 ] || [ "$MISSED" -ne 0 ]
then
    fail "3: mutations"
fi

# ---------------------------------------------------------------- stage 4
# The profile's code-generation flags, from build_make/pico_rp2040.mk (the
# base CFLAGS, -DNDEBUG -ffunction-sections -fdata-sections, and
# -DTM_MQTT=1 for MQTT=1). s1_telem.c includes only standard headers and
# S1's own, so no board flags are needed.
echo "== stage 4: target compile"
STAGE4=skipped
if command -v arm-none-eabi-gcc >/dev/null 2>&1
then
    ARM="-mcpu=cortex-m0plus -mthumb -ffreestanding -std=gnu11 -O2 -g3"
    ARM="$ARM -mfloat-abi=soft -DNDEBUG -ffunction-sections -fdata-sections"
    ARM="$ARM -DTM_MQTT=1"
    if ! arm-none-eabi-gcc $ARM -Wall -Wextra -Wconversion -Wshadow \
            -pedantic -I"$SRC_DIR" -c "$SRC_DIR/s1_telem.c" \
            -o "$WORK/s1_telem.o" 2>"$WORK/arm.err" || [ -s "$WORK/arm.err" ]
    then
        cat "$WORK/arm.err"
        fail "4: target compile"
    fi
    echo "$(arm-none-eabi-gcc --version | head -n 1)"
    arm-none-eabi-size "$WORK/s1_telem.o" | sed "s|$WORK/||"
    STAGE4=ok
    echo "stage 4: ok, no warnings"
else
    echo "stage 4 skipped (arm-none-eabi-gcc is not on the PATH)"
fi

echo "S1-U14: PASS ($CHECKS checks, $CAUGHT/8 mutations caught," \
     "target compile $STAGE4)"
exit 0
