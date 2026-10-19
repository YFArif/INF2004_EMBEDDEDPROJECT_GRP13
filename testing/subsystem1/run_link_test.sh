#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_link_test.sh -- S1-U07: host test of app_program/subsystem1/mqtt_link.c
# with the caps taken from msg_layout.h (MQTT_LINK_PAYLOAD_MAX 25,
# MQTT_LINK_COMMAND_MAX 2).
#
# Run from the repository root, with PICO_SDK_PATH pointing at a Pico SDK
# that has lib/lwip checked out:
#
#     sh testing/subsystem1/run_link_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem1/run_link_test.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_host_test_mqtt_link_caps.txt
#
# Stages; the run stops at the first stage that fails:
#   1  Build testing/subsystem1/mqtt_link_host_test.c, which includes
#      mqtt_link.c, with the command given in that file's header. It needs
#      the local app_program/subsystem1/mqtt_config.h.
#   2  Run it; it must end with "RESULT: all passed".
#   3  The compile-time check of the caps. Copy mqtt_link.c and the
#      headers it includes into a temporary folder, with mqtt_config.h made
#      from mqtt_config.example.h (placeholders only), and require a clean
#      compile of the unmutated copy. Then make two mutations of the copied
#      mqtt_link.h, each matched on text and compared with the original
#      ("mutation did not apply" if nothing changed; exactly one changed
#      line is required). Each mutant must fail to compile AND name
#      mqtt_link_check_caps_t in the compiler output; a failure for any
#      other reason fails the run.
#
# The last line is "S1-U07: PASS (...)" with exit status 0, or
# "S1-U07: FAIL (...)" with exit status 1. Nothing in the repository is
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
WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u07.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

# The flags of the build command in mqtt_link_host_test.c's header.
FLAGS="-std=gnu11 -O0 -w -DTM_MQTT=1"
LWIP_I="-I$PORT_DIR/include -I$PORT_DIR -I$PICO_SDK_PATH/lib/lwip/src/include"

fail ()
{
    echo "S1-U07: FAIL (stage $1)"
    exit 1
}

echo "S1-U07 run_link_test.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
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
echo "== stage 3: compile-time check of the caps"
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

# mutate ID SED_SCRIPT DESCRIPTION -- mutate the copied mqtt_link.h.
mutate ()
{
    dir="$WORK/$1"
    mkdir -p "$dir"
    cp "$BASE/mqtt_link.c" "$BASE/mqtt_queue.h" "$BASE/msg_layout.h" \
       "$BASE/mqtt_config.h" "$dir/"
    sed "$2" "$BASE/mqtt_link.h" >"$dir/mqtt_link.h"

    if cmp -s "$BASE/mqtt_link.h" "$dir/mqtt_link.h"
    then
        echo "$1 $3: mutation did not apply"
        MISSED=$((MISSED + 1))
        return
    fi

    changed=$(diff "$BASE/mqtt_link.h" "$dir/mqtt_link.h" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $3: changed $changed lines, expected 1"
        MISSED=$((MISSED + 1))
        return
    fi

    if compile "$dir"
    then
        echo "$1 $3: NOT caught (the build succeeded)"
        MISSED=$((MISSED + 1))
    elif grep -q 'mqtt_link_check_caps_t' "$dir/err"
    then
        echo "$1 $3: caught by mqtt_link_check_caps_t"
        CAUGHT=$((CAUGHT + 1))
    else
        echo "$1 $3: build failed, but mqtt_link_check_caps_t is not named"
        cat "$dir/err"
        MISSED=$((MISSED + 1))
    fi
}

mutate L1 's/\(MQTT_LINK_COMMAND_MAX  *\)MSG_CMD_BYTES$/\13U/' \
    "MQTT_LINK_COMMAND_MAX MSG_CMD_BYTES -> 3U"
mutate L2 's/\(MQTT_LINK_PAYLOAD_MAX  *\)MSG_MAX_BYTES$/\124U/' \
    "MQTT_LINK_PAYLOAD_MAX MSG_MAX_BYTES -> 24U"

echo "stage 3: $CAUGHT/2 mutations caught"
if [ "$CAUGHT" -ne 2 ] || [ "$MISSED" -ne 0 ]
then
    fail "3: mutations"
fi

echo "S1-U07: PASS (host test all passed, clean compile of the copy," \
     "$CAUGHT/2 cap mutations caught by mqtt_link_check_caps_t)"
exit 0
