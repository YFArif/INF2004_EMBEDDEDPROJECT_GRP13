#!/bin/sh
#
# run_motion_test.sh -- S2-U09: host (PC) test of
# app_program/subsystem2/motion_task.c against a simulated robot.
#
# Run from the repository root, in WSL:
#
#     sh testing/subsystem2/run_motion_test.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem2/run_motion_test.sh \
#         | tee testing/subsystem2/logs/YYYY-MM-DD_host_test_motion.txt
#
# Builds testing/subsystem2/motion_host_test.c (which #includes
# motion_task.c unchanged) with strict warnings as errors and the address
# and undefined-behaviour sanitizers, then runs it. The last line is
# "S2-U09: PASS (...)" or "S2-U09: FAIL (...)". Nothing in the
# repository is changed; the binary goes to a temporary directory.
#
# Environment: CC (default gcc), TMPDIR.

set -u

TEST_DIR=testing/subsystem2

if [ ! -f app_program/subsystem2/motion_task.c ] || \
   [ ! -f "$TEST_DIR/motion_host_test.c" ]
then
    echo "run_motion_test.sh: run this from the repository root" >&2
    exit 2
fi

CC=${CC:-gcc}
OUT=$(mktemp -d "${TMPDIR:-/tmp}/s2_host.XXXXXX")
trap 'rm -rf "$OUT"' EXIT

echo "commit: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)$(git diff --quiet 2>/dev/null || echo ' (+ uncommitted changes)')"
echo "compiler: $($CC --version | head -1)"

if ! $CC -std=gnu11 -O1 -g -Wall -Wextra -Wshadow -Werror \
        -Wno-unused-function \
        -fsanitize=address,undefined -fno-sanitize-recover=all \
        -DAPP_TEST=0 -I "$TEST_DIR/host" -include "$TEST_DIR/host/host_s5.h" \
        -o "$OUT/motion_host_test" "$TEST_DIR/motion_host_test.c" -lm
then
    echo "S2-U09: FAIL (does not compile)"
    exit 1
fi

"$OUT/motion_host_test"
STATUS=$?
exit $STATUS
