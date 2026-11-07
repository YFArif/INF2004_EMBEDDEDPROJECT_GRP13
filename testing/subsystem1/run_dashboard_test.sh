#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_dashboard_test.sh -- S1-U16 part A: s1_dashboard.py --check, then probe
# mutations that show A14 notices the heartbeat's tel_drop going wrong.
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_dashboard_test.sh
#
# Stages; the run stops at the first stage that fails:
#   1  python3 s1_dashboard.py --check on the real files: exit status 0.
#   2  Two mutations, each on a copy of the dashboard, its helper and its
#      decoder in a temporary tree. Each matches on text, and exactly one
#      line must change. Each copy's --check must exit 1, with A14 failing
#      and no other step.
#        P1  on_heartbeat() replaces the heartbeat's tel_drop with 0
#        P2  tel_drop_text() returns the plain number at the clamp
#
# The last line is "S1-U16 part A: PASS (...)" with exit status 0, or
# "S1-U16 part A: FAIL (...)" with exit status 1. Nothing in the repository
# is changed. --check binds a local port on 127.0.0.1 and needs no broker.
# It takes about three minutes.
#
# Environment: TMPDIR.

set -u

TEST_DIR=testing/subsystem1
DECODER=app_program/subsystem1/dashboard/s1_decode.py

if [ ! -f "$TEST_DIR/s1_dashboard.py" ] || [ ! -f "$DECODER" ]
then
    echo "run_dashboard_test.sh: run this from the repository root" >&2
    exit 2
fi

WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u16.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

fail ()
{
    echo "S1-U16 part A: FAIL (stage $1)"
    exit 1
}

# ---------------------------------------------------------------- stage 1
echo "== stage 1: --check"
python3 -I -B "$TEST_DIR/s1_dashboard.py" --check >"$WORK/check.out" 2>&1
RC=$?
cat "$WORK/check.out"
if [ "$RC" -ne 0 ]
then
    fail "1: --check, exit status $RC"
fi
echo "stage 1: ok"

# ---------------------------------------------------------------- stage 2
echo "== stage 2: probe mutations"

# mutate ID SED_SCRIPT DESCRIPTION -- the mutated copy's --check must exit 1
# with A14 as its only failing step.
mutate ()
{
    copy="$WORK/$1"
    mkdir -p "$copy/$TEST_DIR" "$copy/$(dirname "$DECODER")"
    cp "$TEST_DIR/cmd_rtt.py" "$copy/$TEST_DIR/"
    cp "$DECODER" "$copy/$DECODER"
    sed "$2" "$TEST_DIR/s1_dashboard.py" >"$copy/$TEST_DIR/s1_dashboard.py"
    if cmp -s "$TEST_DIR/s1_dashboard.py" "$copy/$TEST_DIR/s1_dashboard.py"
    then
        echo "$1 $3: mutation did not apply"
        fail "2: $1"
    fi
    changed=$(diff "$TEST_DIR/s1_dashboard.py" \
              "$copy/$TEST_DIR/s1_dashboard.py" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $3: changed $changed lines, expected 1"
        fail "2: $1"
    fi

    python3 -I -B "$copy/$TEST_DIR/s1_dashboard.py" --check \
        >"$copy/check.out" 2>&1
    rc=$?
    failed=$(sed -n 's/^\(A[0-9]*\) *FAIL:.*/\1/p' "$copy/check.out" \
             | tr '\n' ' ')
    if [ "$rc" -ne 1 ] || [ "$failed" != "A14 " ]
    then
        cat "$copy/check.out"
        echo "$1 $3: NOT caught as expected" \
             "(exit $rc, failed steps: ${failed:-none})"
        fail "2: $1"
    fi
    echo "$1 $3: caught by A14 and no other step"
}

mutate P1 's/"tel_drop": msg\["tel_drop"\], "evt_drop"/"tel_drop": 0, "evt_drop"/' \
    "heartbeat tel_drop ignored"
mutate P2 's/return "%d or more" % TEL_DROP_MAX/return str(value)/' \
    "tel_drop_text returns the plain number"
echo "stage 2: 2/2 probe mutations caught"

echo "S1-U16 part A: PASS (--check passed, 2/2 probe mutations caught)"
exit 0
