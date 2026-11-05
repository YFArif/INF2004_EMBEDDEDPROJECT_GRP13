#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_link_test.sh -- S1-U09: host test of app_program/subsystem1/mqtt_link.c
# with the caps taken from msg_layout.h (MQTT_LINK_PAYLOAD_MAX 25,
# MQTT_LINK_COMMAND_MAX 2) and a topic ID in the publish record.
#
# Run from the repository root, with PICO_SDK_PATH pointing at a Pico SDK
# that has lib/lwip checked out:
#
#     sh testing/subsystem1/run_link_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem1/run_link_test.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_topic_id.txt
#
# Stages; the run stops at the first stage that fails:
#   1  Build testing/subsystem1/mqtt_link_host_test.c, which includes
#      mqtt_link.c, with the command given in that file's header. It needs
#      the local app_program/subsystem1/mqtt_config.h.
#   2  Run it; it must end with "RESULT: all passed".
#   3  The compile-time checks. Copy mqtt_link.c and the headers it
#      includes into a temporary folder, with mqtt_config.h made from
#      mqtt_config.example.h (placeholders only), and require a clean
#      compile of the unmutated copy. Then make four mutations of the
#      copied mqtt_link.h or mqtt_link.c, each matched on text and compared
#      with the original ("mutation did not apply" if nothing changed;
#      exactly one changed line is required). Each mutant must fail to
#      compile AND name its check in the compiler output; a failure for
#      any other reason fails the run:
#        L1, L2  either cap changed        -> mqtt_link_check_caps_t
#        L3      a 49-character suffix     -> mqtt_link_check_suffix_t
#        L4      a topic ID with no suffix -> mqtt_link_check_table_t
#   4  The run-time check of the topic mapping. Build and run the host test
#      against the unmutated copy (R0), which must pass, then against a
#      copy whose event suffix is changed to "ack" (R1), which must fail,
#      and only in the "-- topic mapping" section.
#
# The last line is "S1-U09: PASS (...)" with exit status 0, or
# "S1-U09: FAIL (...)" with exit status 1. Nothing in the repository is
# changed: build outputs and copies go to a temporary directory that is
# removed at the end.
#
# Environment: PICO_SDK_PATH (required), CC (default gcc), TMPDIR.

set -u

TEST_DIR=testing/subsystem1
CODEC_DIR=app_program/subsystem1
PORT_DIR=lib/liblwip/sysdepend/pico_rp2040

if [ ! -f "$CODEC_DIR/mqtt_link.c" ] || [ ! -f "$TEST_DIR/mqtt_link_host_test.c" ]
then
    echo "run_link_test.sh: run this from the repository root" >&2
    exit 2
fi

if [ -z "${PICO_SDK_PATH:-}" ] || [ ! -d "$PICO_SDK_PATH/lib/lwip/src/include" ]
then
    echo "run_link_test.sh: PICO_SDK_PATH must point at a Pico SDK with" \
         "lib/lwip checked out" >&2
    exit 2
fi

CC=${CC:-gcc}
WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u09.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

# The flags of the build command in mqtt_link_host_test.c's header.
FLAGS="-std=gnu11 -O0 -w -DTM_MQTT=1"
LWIP_I="-I$PORT_DIR/include -I$PORT_DIR -I$PICO_SDK_PATH/lib/lwip/src/include"

fail ()
{
    echo "S1-U09: FAIL (stage $1)"
    exit 1
}

echo "S1-U09 run_link_test.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
DIRTY=$(git status --porcelain -- "$CODEC_DIR" "$TEST_DIR" 2>/dev/null \
        | grep -v '^?? .*logs/' | grep -v 'mqtt_config\.h$' | head -n 1)
if [ -n "$DIRTY" ]
then
    COMMIT="$COMMIT (tested files have uncommitted changes)"
fi
echo "commit: $COMMIT"
echo "cc: $($CC --version 2>&1 | head -n 1)"

# ---------------------------------------------------------------- stage 1
echo "== stage 1: build the host test"
if ! $CC $FLAGS -I"$CODEC_DIR" $LWIP_I -o "$WORK/link_test" \
        "$TEST_DIR/mqtt_link_host_test.c" 2>"$WORK/build.err"
then
    cat "$WORK/build.err"
    fail "1: build"
fi
echo "stage 1: ok"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: run"
"$WORK/link_test" >"$WORK/run.out" 2>&1
RC=$?
cat "$WORK/run.out"
if [ "$RC" -ne 0 ] || ! grep -q '^RESULT: all passed$' "$WORK/run.out"
then
    fail "2: test, exit status $RC"
fi
echo "stage 2: ok"

# ---------------------------------------------------------------- stage 3
echo "== stage 3: compile-time checks"
BASE="$WORK/base"
mkdir -p "$BASE"
cp "$CODEC_DIR/mqtt_link.c" "$CODEC_DIR/mqtt_link.h" "$CODEC_DIR/mqtt_queue.h" \
   "$CODEC_DIR/msg_layout.h" "$BASE/"
sed 's/^\(#define MQTT_LINK_CFG_SET  *\)0$/\11/' \
    "$CODEC_DIR/mqtt_config.example.h" >"$BASE/mqtt_config.h"
if cmp -s "$CODEC_DIR/mqtt_config.example.h" "$BASE/mqtt_config.h"
then
    echo "could not enable the .example config copy"
    fail "3: config copy"
fi

# compile DIR: compile DIR/mqtt_link.c; compiler output in DIR/err.
compile ()
{
    $CC $FLAGS $LWIP_I -c "$1/mqtt_link.c" -o "$1/mqtt_link.o" 2>"$1/err"
}

if ! compile "$BASE" || [ -s "$BASE/err" ]
then
    cat "$BASE/err"
    echo "the unmutated copy does not compile cleanly"
    fail "3: unmutated copy"
fi
echo "unmutated copy: clean compile"

CAUGHT=0
MISSED=0

# copy_to DIR FILE SED_SCRIPT ID DESCRIPTION -- copy the base files to DIR
# with FILE (mqtt_link.h or mqtt_link.c) changed by SED_SCRIPT. Fails, and
# says why, unless exactly one line changed.
copy_to ()
{
    mkdir -p "$1"
    cp "$BASE/mqtt_link.c" "$BASE/mqtt_link.h" "$BASE/mqtt_queue.h" \
       "$BASE/msg_layout.h" "$BASE/mqtt_config.h" "$1/"
    sed "$3" "$BASE/$2" >"$1/$2"

    if cmp -s "$BASE/$2" "$1/$2"
    then
        echo "$4 $5: mutation did not apply"
        return 1
    fi

    changed=$(diff "$BASE/$2" "$1/$2" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$4 $5: changed $changed lines, expected 1"
        return 1
    fi
    return 0
}

# mutate ID FILE SED_SCRIPT CHECK DESCRIPTION -- the mutated copy must fail
# to compile and name the typedef CHECK.
mutate ()
{
    dir="$WORK/$1"
    if ! copy_to "$dir" "$2" "$3" "$1" "$5"
    then
        MISSED=$((MISSED + 1))
        return
    fi

    if compile "$dir"
    then
        echo "$1 $5: NOT caught (the build succeeded)"
        MISSED=$((MISSED + 1))
    elif grep -q "$4" "$dir/err"
    then
        echo "$1 $5: caught by $4"
        CAUGHT=$((CAUGHT + 1))
    else
        echo "$1 $5: build failed, but $4 is not named"
        cat "$dir/err"
        MISSED=$((MISSED + 1))
    fi
}

mutate L1 mqtt_link.h 's/\(MQTT_LINK_COMMAND_MAX  *\)MSG_CMD_BYTES$/\13U/' \
    mqtt_link_check_caps_t "MQTT_LINK_COMMAND_MAX MSG_CMD_BYTES -> 3U"
mutate L2 mqtt_link.h 's/\(MQTT_LINK_PAYLOAD_MAX  *\)MSG_MAX_BYTES$/\124U/' \
    mqtt_link_check_caps_t "MQTT_LINK_PAYLOAD_MAX MSG_MAX_BYTES -> 24U"
LONG=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
mutate L3 mqtt_link.c "s/^\(#define TOPIC_ACK  *\)\"ack\"\$/\1\"$LONG\"/" \
    mqtt_link_check_suffix_t "TOPIC_ACK \"ack\" -> 49 characters"
mutate L4 mqtt_link.h 's/\(MQTT_LINK_TOPIC_COUNT  *\)3U$/\14U/' \
    mqtt_link_check_table_t "MQTT_LINK_TOPIC_COUNT 3U -> 4U"

echo "stage 3: $CAUGHT/4 mutations caught"
if [ "$CAUGHT" -ne 4 ] || [ "$MISSED" -ne 0 ]
then
    fail "3: mutations"
fi

# ---------------------------------------------------------------- stage 4
echo "== stage 4: run-time check of the topic mapping"

# run_copy ID -- build the host test against $WORK/ID/app_program/subsystem1
# (it includes mqtt_link.c by a path relative to itself) and run it. The
# output goes to $WORK/ID/run.out and the exit status to RUN_RC.
run_copy ()
{
    top="$WORK/$1"
    mkdir -p "$top/testing/subsystem1"
    cp "$TEST_DIR/mqtt_link_host_test.c" "$top/testing/subsystem1/"
    if ! $CC $FLAGS -I"$top/app_program/subsystem1" $LWIP_I \
            -o "$top/test" "$top/testing/subsystem1/mqtt_link_host_test.c" \
            2>"$top/build.err"
    then
        cat "$top/build.err"
        fail "4: $1 build"
    fi
    "$top/test" >"$top/run.out" 2>&1
    RUN_RC=$?
}

mkdir -p "$WORK/R0/app_program"
cp -R "$BASE" "$WORK/R0/app_program/subsystem1"
run_copy R0
if [ "$RUN_RC" -ne 0 ] || ! grep -q '^RESULT: all passed$' "$WORK/R0/run.out"
then
    cat "$WORK/R0/run.out"
    fail "4: R0, the unmutated copy does not pass"
fi
echo "R0 unmutated copy: all passed"

if ! copy_to "$WORK/R1/app_program/subsystem1" mqtt_link.c \
        's/^\(#define TOPIC_EVENT  *\)"event"$/\1"ack"/' \
        R1 'TOPIC_EVENT "event" -> "ack"'
then
    fail "4: R1 mutation"
fi
run_copy R1
SECTIONS=$(awk '/^-- /{ s = $0 } /^  FAIL line /{ print s }' \
           "$WORK/R1/run.out" | sort -u)
if [ "$RUN_RC" -eq 0 ]
then
    echo "R1 TOPIC_EVENT \"event\" -> \"ack\": NOT caught (the test passed)"
    fail "4: R1"
elif [ "$SECTIONS" != "-- topic mapping: full topic of each ID, bad queued ID" ]
then
    cat "$WORK/R1/run.out"
    echo "R1: failed outside the topic mapping section"
    fail "4: R1"
fi
grep '^  FAIL line ' "$WORK/R1/run.out"
echo "R1 TOPIC_EVENT \"event\" -> \"ack\": caught by the topic mapping test"
echo "stage 4: 1/1 mutation caught"

echo "S1-U09: PASS (host test all passed, clean compile of the copy," \
     "$CAUGHT/4 compile-time mutations caught, 1/1 run-time mutation" \
     "caught)"
exit 0
