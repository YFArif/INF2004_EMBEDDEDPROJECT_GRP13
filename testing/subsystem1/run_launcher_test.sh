#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_launcher_test.sh -- S1-U23 part A: s1_launch_test.py on the real
# launcher, then two mutations that show the test notices them.
#
# Run from the repository root (macOS or Linux):
#
#     sh testing/subsystem1/run_launcher_test.sh
#
# Stages; the run stops at the first stage that fails:
#   1  python3 s1_launch_test.py on the real files: exit status 0. This
#      includes two end-to-end --simulate runs on a free port.
#   2  Two mutations, each on a copy of s1_launch.py in a temporary folder.
#      Each matches on text, and exactly one line must change. The test,
#      run on the copy with --no-e2e, must exit 1 with the named check
#      failing.
#        M1  ensure_config() writes over an existing mosquitto.conf  (L7)
#        M2  port_in_use() reports a listening port as free          (L8)
#
# The last line is "S1-U23 part A: PASS (...)" with exit status 0, or
# "S1-U23 part A: FAIL (...)" with exit status 1. Nothing in the repository
# or in ~/s1-broker is changed. Stage 1 binds local ports on 127.0.0.1 and
# needs no broker. It takes about 40 seconds.
#
# Environment: TMPDIR.

set -u

TEST_DIR=testing/subsystem1

if [ ! -f "$TEST_DIR/s1_launch.py" ] || [ ! -f "$TEST_DIR/s1_launch_test.py" ]
then
    echo "run_launcher_test.sh: run this from the repository root" >&2
    exit 2
fi

WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u23.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

fail ()
{
    echo "S1-U23 part A: FAIL (stage $1)"
    exit 1
}

# ---------------------------------------------------------------- stage 1
echo "== stage 1: s1_launch_test.py"
python3 -I -B "$TEST_DIR/s1_launch_test.py" >"$WORK/test.out" 2>&1
RC=$?
cat "$WORK/test.out"
if [ "$RC" -ne 0 ]
then
    fail "1: s1_launch_test.py, exit status $RC"
fi
echo "stage 1: ok"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: mutations"

# mutate ID CHECK SED_SCRIPT DESCRIPTION -- the copy's test must exit 1
# with CHECK failing.
mutate ()
{
    copy="$WORK/$1"
    mkdir -p "$copy"
    sed "$3" "$TEST_DIR/s1_launch.py" >"$copy/s1_launch.py"
    changed=$(diff "$TEST_DIR/s1_launch.py" "$copy/s1_launch.py" \
              | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        fail "2: $1 changed $changed lines, expected 1"
    fi
    python3 -I -B "$TEST_DIR/s1_launch_test.py" --module-dir "$copy" \
        --no-e2e >"$copy/test.out" 2>&1
    rc=$?
    if [ "$rc" -ne 1 ] || ! grep -q "^$2 FAIL" "$copy/test.out"
    then
        cat "$copy/test.out"
        fail "2: $1 ($4) not caught, exit status $rc"
    fi
    echo "$1 caught by $2: $4"
}

M1='s/open(path, "x", encoding=/open(path, "w", encoding=/'
M2='/socket.create_connection((host, port)/{n;s/return True/return False/;}'
mutate M1 L7 "$M1" "the config is written over"
mutate M2 L8 "$M2" "a listening port looks free"
echo "stage 2: ok"

echo "S1-U23 part A: PASS (s1_launch_test.py passed, 2/2 mutations caught)"
exit 0
