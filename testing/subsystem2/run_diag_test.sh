#!/bin/sh
set -eu
OUT=$(mktemp "${TMPDIR:-/tmp}/s2_diag.XXXXXX")
trap 'rm -f "$OUT"' EXIT
for mode in 1 2 3
do
gcc -std=gnu11 -O1 -g -Wall -Wextra -Wshadow -Werror \
    -Wno-unused-function -ffunction-sections -fdata-sections \
    -fsanitize=address,undefined -fno-sanitize-recover=all \
    -DAPP_TEST=1 -DS2_MOTOR_DIAG=$mode -I testing/subsystem2/host \
    -Wl,--gc-sections -o "$OUT" testing/subsystem2/motor_diag_host_test.c
"$OUT"
done
