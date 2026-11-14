#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_dashboard_test.sh -- S1-U16 part A and S1-U24 part D:
# s1_dashboard.py --check (steps A1 to A14, then D1 to D7, the barcode
# letter map), then probe mutations that each must fail exactly the steps
# named for them.
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_dashboard_test.sh
#
# Stages; the run stops at the first stage that fails:
#   1  python3 s1_dashboard.py --check on the real files: exit status 0.
#   2  Six mutations, each on a copy of the dashboard, its page files
#      (dashboard_ui/), its helper and its decoder in a temporary tree. Each matches on text, and exactly one
#      line must change. Each copy's --check must exit 1, with exactly the
#      steps named failing and no other. P1 and P2 are S1-U16's; N1 to N4
#      are S1-U24 part D's.
#        P1  on_heartbeat() replaces the heartbeat's tel_drop with 0 (A14)
#        P2  tel_drop_text() returns the plain number at the clamp (A14)
#        N1  the server's repeated-letter check removed (D2)
#        N2  no re-send when the status turns online (D3 and D6)
#        N3  no re-send when the heartbeat's uptime goes down (D4)
#        N4  a fourth try of a map allowed (D5)
#
# The last line is "S1-U16 part A and S1-U24 part D: PASS (...)" with exit
# status 0, or "...: FAIL (...)" with exit status 1. Nothing in the
# repository is changed. --check binds a local port on 127.0.0.1 and needs
# no broker. It takes about six minutes.
#
# Environment: TMPDIR.

set -u

TEST_DIR=testing/subsystem1
DECODER=app_program/subsystem1/dashboard/s1_decode.py

if [ ! -f "$TEST_DIR/s1_dashboard.py" ] || [ ! -f "$DECODER" ] \
   || [ ! -f "$TEST_DIR/dashboard_ui/index.html" ]
then
    echo "run_dashboard_test.sh: run this from the repository root" >&2
    exit 2
fi

WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u16.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM

fail ()
{
    echo "S1-U16 part A and S1-U24 part D: FAIL (stage $1)"
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

# mutate ID SED_SCRIPT DESCRIPTION STEPS -- the mutated copy's --check must
# exit 1 with exactly STEPS failing (space-separated, in the order --check
# runs them).
mutate ()
{
    copy="$WORK/$1"
    mkdir -p "$copy/$TEST_DIR" "$copy/$(dirname "$DECODER")"
    cp "$TEST_DIR/cmd_rtt.py" "$copy/$TEST_DIR/"
    cp -R "$TEST_DIR/dashboard_ui" "$copy/$TEST_DIR/"
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
    failed=$(sed -n 's/^\([AD][0-9]*\) *FAIL:.*/\1/p' "$copy/check.out" \
             | tr '\n' ' ')
    if [ "$rc" -ne 1 ] || [ "$failed" != "$4 " ]
    then
        cat "$copy/check.out"
        echo "$1 $3: NOT caught as expected" \
             "(exit $rc, failed steps: ${failed:-none})"
        fail "2: $1"
    fi
    echo "$1 $3: caught by $4 and no other step"
}

mutate P1 's/"tel_drop": msg\["tel_drop"\], "evt_drop"/"tel_drop": 0, "evt_drop"/' \
    "heartbeat tel_drop ignored" "A14"
mutate P2 's/return "%d or more" % TEL_DROP_MAX/return str(value)/' \
    "tel_drop_text returns the plain number" "A14"
mutate N1 's/^    if len(set(letters)) != cmd_rtt.MAP_LETTERS:$/    if False:/' \
    "server's repeated-letter check removed" "D2"
mutate N2 's/^            self.map_trigger("online")$/            pass/' \
    "no map re-send on online" "D3 D6"
mutate N3 's/^                self.map_trigger("restart", time.monotonic() - msg\["uptime_s"\])$/                pass/' \
    "no map re-send when the uptime goes down" "D4"
mutate N4 's/^MAP_TRIES = 3 /MAP_TRIES = 4 /' \
    "a fourth try of a map allowed" "D5"
echo "stage 2: 6/6 probe mutations caught (P1, P2, N1 to N4)"

echo "S1-U16 part A and S1-U24 part D: PASS (--check passed, 6/6 probe" \
     "mutations caught)"
exit 0
