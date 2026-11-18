#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_wifi_rejoin_test.sh -- S1-U28: host test of S1's WiFi rejoin policy
# (app_program/subsystem1/s1_wifi_rejoin.h, s1_wifi_rejoin.c), driven with
# a fake clock, one step per pass of the radio task (about every 20 ms).
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_wifi_rejoin_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem1/run_wifi_rejoin_test.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_wifi_rejoin.txt
#
# Stages; the run stops at the first stage that fails:
#   1  Build s1_wifi_rejoin_test.c and s1_wifi_rejoin.c twice, with
#      S1_WIFI_LEAVE_BEFORE_JOIN 0 and 1, with strict warnings as errors,
#      UBSan and ASan.
#   2  Run both builds; each must end with "RESULT: all passed".
#   3  Eleven mutations, each on a copy of s1_wifi_rejoin in a temporary
#      folder; every one must be caught. One must stop the build at a
#      compile-time check; ten must build and make the test fail (a
#      sanitizer abort counts). Each mutation matches on text, and exactly
#      one line must change.
#   4  If arm-none-eabi-gcc is on the PATH, compile s1_wifi_rejoin.c for the
#      target with the profile's code-generation flags; any warning fails.
#      Otherwise the stage is skipped.
#
# The last line is "S1-U28: PASS (...)" with exit status 0, or
# "S1-U28: FAIL (...)" with exit status 1. Nothing in the repository is
# changed: every build output and mutated copy goes to a temporary
# directory that is removed at the end.
#
# Environment: CC (default cc), TMPDIR.

set -u

TEST_DIR=testing/subsystem1
SRC_DIR=app_program/subsystem1

if [ ! -f "$SRC_DIR/s1_wifi_rejoin.c" ] \
   || [ ! -f "$TEST_DIR/s1_wifi_rejoin_test.c" ]
then
    echo "run_wifi_rejoin_test.sh: run this from the repository root" >&2
    exit 2
fi

CC=${CC:-cc}
WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u28.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

WARN="-std=c99 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion"
WARN="$WARN -Wshadow -Wcast-qual -Wstrict-prototypes -Wmissing-prototypes"
SAN="-fsanitize=undefined,address -fno-sanitize-recover=all"
SAN="$SAN -fno-omit-frame-pointer"

fail ()
{
    echo "S1-U28: FAIL (stage $1)"
    exit 1
}

echo "S1-U28 run_wifi_rejoin_test.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
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
echo "== stage 1: build, both leave settings (strict warnings as errors," \
     "UBSan/ASan)"
for LEAVE in 0 1
do
    if ! $CC $WARN -Werror -O1 -g $SAN -DTM_MQTT=1 \
            -DS1_WIFI_LEAVE_BEFORE_JOIN=$LEAVE -I"$SRC_DIR" \
            "$TEST_DIR/s1_wifi_rejoin_test.c" "$SRC_DIR/s1_wifi_rejoin.c" \
            -o "$WORK/s1u28_leave$LEAVE" 2>"$WORK/build$LEAVE.err"
    then
        cat "$WORK/build$LEAVE.err"
        fail "1: build, S1_WIFI_LEAVE_BEFORE_JOIN $LEAVE"
    fi
done
echo "stage 1: ok, no warnings"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: run"
CHECKS=""
for LEAVE in 0 1
do
    "$WORK/s1u28_leave$LEAVE" >"$WORK/run$LEAVE.out" 2>&1
    RC=$?
    cat "$WORK/run$LEAVE.out"
    N=$(sed -n 's/^RESULT: all passed (\([0-9]*\) checks)$/\1/p' \
        "$WORK/run$LEAVE.out")
    if [ "$RC" -ne 0 ] || [ -z "$N" ]
    then
        fail "2: test, S1_WIFI_LEAVE_BEFORE_JOIN $LEAVE, exit status $RC"
    fi
    CHECKS="$CHECKS${CHECKS:+ and }$N"
done
echo "stage 2: ok"

# ---------------------------------------------------------------- stage 3
echo "== stage 3: mutations"
CAUGHT=0
MISSED=0

# mutate ID KIND FILE SED_SCRIPT DESCRIPTION
#   KIND is "compile" (the build must stop at a compile-time check) or
#   "run" (the mutant must build and the test must fail). Exactly one line
#   must change. Mutants are built with S1_WIFI_LEAVE_BEFORE_JOIN 0; the
#   leave groups pass the setting to s1_wifi_rejoin_init() themselves.
mutate ()
{
    dir="$WORK/$1"
    mkdir -p "$dir"
    cp "$SRC_DIR/s1_wifi_rejoin.h" "$SRC_DIR/s1_wifi_rejoin.c" "$dir/"
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
        if $CC $WARN -O1 -DTM_MQTT=1 -I"$dir" \
                -c "$dir/s1_wifi_rejoin.c" -o "$dir/s1_wifi_rejoin.o" \
                2>"$dir/err"
        then
            echo "$1 $5: NOT caught (the build succeeded)"
            MISSED=$((MISSED + 1))
        elif grep -q 's1_wifi_rejoin_check_' "$dir/err"
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
    if ! $CC $WARN -O1 -g $SAN -DTM_MQTT=1 -I"$dir" \
            "$TEST_DIR/s1_wifi_rejoin_test.c" "$dir/s1_wifi_rejoin.c" \
            -o "$dir/test" 2>"$dir/err"
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

mutate C1 compile s1_wifi_rejoin.c \
    's/^    S1_WIFI_REJOIN_WAIT_CAP_MSEC$/    S1_WIFI_REJOIN_WAIT_CAP_MSEC, 8000U/' \
    "a fourth ladder step without S1_WIFI_REJOIN_WAIT_STEPS"
mutate W1 run s1_wifi_rejoin.h \
    's/\(#define S1_WIFI_REJOIN_JOIN_LIMIT_MSEC  *\)10000U/\1UINT32_MAX/' \
    "no time limit on a join"
mutate W2 run s1_wifi_rejoin.c \
    's|^        /\* Status 0, 1 or 2 within the limit: keep waiting. \*/$|        p_rejoin->since_msec = (1 == link_status) ? now_msec : p_rejoin->since_msec;|' \
    "status 1 treated as progress (the join limit restarts on it)"
mutate W3 run s1_wifi_rejoin.c \
    's/^\( *\)S1_WIFI_REJOIN_STABLE_MSEC)))$/\10U)))/' \
    "counter reset on first sight of status 3"
mutate W4 run s1_wifi_rejoin.c \
    's/const uint32_t index = (attempt < last) ? attempt : last;/const uint32_t index = attempt;/' \
    "no cap on the ladder"
mutate W5 run s1_wifi_rejoin.h \
    's/\(#define S1_WIFI_REJOIN_GRACE_MSEC  *\)3000U/\10U/' \
    "grace ignored"
mutate W6 run s1_wifi_rejoin.c \
    's/^    if (S1_WIFI_REJOIN_LINK_DOWN == link_status)$/    if (S1_WIFI_REJOIN_LINK_UP == link_status) { enter(p_rejoin, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_WHY_LINK_UP, now_msec, p_change); } else if (S1_WIFI_REJOIN_LINK_DOWN == link_status)/' \
    "LEAVING counts status 3 as UP"
mutate W7 run s1_wifi_rejoin.h \
    's/\(#define S1_WIFI_REJOIN_LEAVE_LIMIT_MSEC  *\)1000U/\1UINT32_MAX/' \
    "LEAVING with no 1 s limit"
mutate W8 run s1_wifi_rejoin.c \
    's/return ((now_msec - since_msec) >= limit_msec);/return (now_msec >= (since_msec + limit_msec));/' \
    "now >= deadline in place of the subtraction (clock wrap)"
mutate W9 run s1_wifi_rejoin.c \
    's/^        if (S1_WIFI_REJOIN_JOINING == p_rejoin->state)$/        if (false)/' \
    "a refused join request waits for the join limit"
mutate W10 run s1_wifi_rejoin.c \
    '/^step_joining (/,/^}/s/^    else if (link_status < 0)$/    else if (false)/' \
    "a negative status in JOINING waits for the join limit"

echo "stage 3: $CAUGHT/11 mutations caught (C1, W1 to W10)"
if [ "$CAUGHT" -ne 11 ] || [ "$MISSED" -ne 0 ]
then
    fail "3: mutations"
fi

# ---------------------------------------------------------------- stage 4
# The profile's code-generation flags, from build_make/pico_rp2040.mk (the
# base CFLAGS, -DNDEBUG -ffunction-sections -fdata-sections, and
# -DTM_MQTT=1 for MQTT=1). s1_wifi_rejoin.c includes only standard headers
# and its own, so no board flags are needed.
echo "== stage 4: target compile"
STAGE4=skipped
if command -v arm-none-eabi-gcc >/dev/null 2>&1
then
    ARM="-mcpu=cortex-m0plus -mthumb -ffreestanding -std=gnu11 -O2 -g3"
    ARM="$ARM -mfloat-abi=soft -DNDEBUG -ffunction-sections -fdata-sections"
    ARM="$ARM -DTM_MQTT=1"
    if ! arm-none-eabi-gcc $ARM -Wall -Wextra -Wconversion -Wshadow \
            -pedantic -I"$SRC_DIR" -c "$SRC_DIR/s1_wifi_rejoin.c" \
            -o "$WORK/s1_wifi_rejoin.o" 2>"$WORK/arm.err" \
            || [ -s "$WORK/arm.err" ]
    then
        cat "$WORK/arm.err"
        fail "4: target compile"
    fi
    echo "$(arm-none-eabi-gcc --version | head -n 1)"
    arm-none-eabi-size "$WORK/s1_wifi_rejoin.o" | sed "s|$WORK/||"
    STAGE4=ok
    echo "stage 4: ok, no warnings"
else
    echo "stage 4 skipped (arm-none-eabi-gcc is not on the PATH)"
fi

echo "S1-U28: PASS ($CHECKS checks with S1_WIFI_LEAVE_BEFORE_JOIN 0 and 1," \
     "$CAUGHT/11 mutations caught, target compile $STAGE4)"
exit 0

