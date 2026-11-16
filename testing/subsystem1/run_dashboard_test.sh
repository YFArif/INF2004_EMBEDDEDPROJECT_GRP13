#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_dashboard_test.sh -- S1-U16 part A, S1-U24 part D and S1-U26 parts A
# and D: s1_dashboard.py --check (steps A1 to A14, then D1 to D7, the
# barcode letter map, then U1 to U4, the page's files, the design tokens
# and the simulator scenarios), then probe mutations that each must fail
# exactly the steps named for them.
#
# Run from the repository root:
#
#     sh testing/subsystem1/run_dashboard_test.sh
#
# Stages; the run stops at the first stage that fails:
#   1  python3 s1_dashboard.py --check on the real files: exit status 0.
#   2  Ten mutations, each on a copy of the dashboard, its page files
#      (dashboard_ui/), its helper and its decoder in a temporary tree.
#      Each matches on text in one file, and exactly one line must change.
#      Each copy's --check must exit 1, with exactly the steps named
#      failing and no other. P1 and P2 are S1-U16's; N1 to N4 are S1-U24
#      part D's; M1 to M4 are S1-U26's.
#        P1  on_heartbeat() replaces the heartbeat's tel_drop with 0 (A14)
#        P2  tel_drop_text() returns the plain number at the clamp (A14)
#        N1  the server's repeated-letter check removed (D2)
#        N2  no re-send when the status turns online (D3 and D6)
#        N3  no re-send when the heartbeat's uptime goes down (D4)
#        N4  a fourth try of a map allowed (D5)
#        M1  the server's list serves another file for /fonts/OFL.txt,
#            through ".." (U1)
#        M2  one changed character in vendor/three/three.core.js (U2)
#        M3  a colour literal in app.css (U3)
#        M4  the obstacle scenario's swerve speeds changed (U4)
#
# The last line is "S1-U16 part A, S1-U24 part D and S1-U26 parts A and D:
# PASS (...)" with exit status 0, or "...: FAIL (...)" with exit status 1.
# Nothing in the repository is changed. --check binds a local port on
# 127.0.0.1 and needs no broker. It takes about seventeen minutes.
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
    echo "S1-U16 part A, S1-U24 part D and S1-U26 parts A and D: FAIL" \
         "(stage $1)"
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

# mutate_in FILE ID SED_SCRIPT DESCRIPTION STEPS -- FILE (under
# testing/subsystem1/) is changed by SED_SCRIPT in a copy of the tree, and
# the copy's --check must exit 1 with exactly STEPS failing (space-
# separated, in the order --check runs them).
mutate_in ()
{
    target="$TEST_DIR/$1"
    shift
    copy="$WORK/$1"
    mkdir -p "$copy/$TEST_DIR" "$copy/$(dirname "$DECODER")"
    cp "$TEST_DIR/cmd_rtt.py" "$TEST_DIR/s1_dashboard.py" "$copy/$TEST_DIR/"
    cp -R "$TEST_DIR/dashboard_ui" "$copy/$TEST_DIR/"
    cp "$DECODER" "$copy/$DECODER"
    sed "$2" "$target" >"$copy/$target"
    if cmp -s "$target" "$copy/$target"
    then
        echo "$1 $3: mutation did not apply"
        fail "2: $1"
    fi
    changed=$(diff "$target" "$copy/$target" | grep -c '^>')
    if [ "$changed" -ne 1 ]
    then
        echo "$1 $3: changed $changed lines, expected 1"
        fail "2: $1"
    fi

    python3 -I -B "$copy/$TEST_DIR/s1_dashboard.py" --check \
        >"$copy/check.out" 2>&1
    rc=$?
    failed=$(sed -n 's/^\([ADU][0-9]*\) *FAIL:.*/\1/p' "$copy/check.out" \
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

# mutate ID SED_SCRIPT DESCRIPTION STEPS -- a mutation of s1_dashboard.py.
mutate ()
{
    mutate_in s1_dashboard.py "$@"
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
mutate M1 's|^    "/fonts/OFL.txt": ("fonts/OFL.txt", "text/plain; charset=utf-8"),$|    "/fonts/OFL.txt": ("fonts/../app.css", "text/plain; charset=utf-8"),|' \
    "/fonts/OFL.txt served from another file through .." "U1"
mutate_in dashboard_ui/vendor/three/three.core.js M2 \
    "s/^const REVISION = '186';\$/const REVISION = '187';/" \
    "one changed character in the vendored three.core.js" "U2"
mutate_in dashboard_ui/app.css M3 \
    's/^  margin: 0; min-height: 100vh; background: var(--bg-page);$/  margin: 0; min-height: 100vh; background: #ffffff;/' \
    "a colour literal in app.css" "U3"
mutate M4 's/^            (2.0, dict(_AVOID, l=76, r=124, cmd=100, closest=(150, 260))),$/            (2.0, dict(_AVOID, l=70, r=130, cmd=100, closest=(150, 260))),/' \
    "the obstacle swerve's wheel speeds changed" "U4"
echo "stage 2: 10/10 probe mutations caught (P1, P2, N1 to N4, M1 to M4)"

echo "S1-U16 part A, S1-U24 part D and S1-U26 parts A and D: PASS" \
     "(--check passed, 10/10 probe mutations caught)"
exit 0
