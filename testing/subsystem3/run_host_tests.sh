#!/bin/sh
#
# run_host_tests.sh -- every S3 host (laptop) test, no Pico needed.
#
# Run from the repository root, in WSL:
#
#     sh testing/subsystem3/run_host_tests.sh
#
# To keep the evidence, as testing/README.md asks:
#
#     sh testing/subsystem3/run_host_tests.sh \
#         | tee testing/subsystem3/logs/YYYY-MM-DD_host.txt
#
# Builds the pure-C S3 files (s3_line.c, s3_code39.c, s3_analog.c) with
# strict warnings as errors, then runs the seven test programs. The line
# test and the track simulation also run under the address and
# undefined-behaviour sanitizers. The last line is "S3 HOST: PASS (...)"
# or "S3 HOST: FAIL (...)". Nothing in the repository is changed; the
# binaries go to a temporary directory.
#
# Environment: CC (default gcc), TMPDIR.

set -u

SRC=app_program/subsystem3
TEST_DIR=testing/subsystem3/host

if [ ! -f "$SRC/s3_line.c" ] || [ ! -f "$TEST_DIR/test_line.c" ]
then
    echo "run_host_tests.sh: run this from the repository root" >&2
    exit 2
fi

CC=${CC:-gcc}
OUT=$(mktemp -d "${TMPDIR:-/tmp}/s3_host.XXXXXX")
trap 'rm -rf "$OUT"' EXIT

FLAGS="-std=c99 -O2 -Wall -Wextra -Wshadow -Werror -I$SRC"
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all"

DIRTY=""
if ! git -c core.autocrlf=true diff --quiet HEAD -- "$SRC" testing/subsystem3 \
        2>/dev/null
then
    DIRTY=" (+ uncommitted changes in S3)"
fi
echo "commit: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)$DIRTY"
echo "compiler: $($CC --version | head -1)"

FAILED=""
RAN=0

# build NAME EXTRA_FLAGS SOURCES...
build ()
{
    name=$1
    extra=$2
    shift 2
    # shellcheck disable=SC2086
    if ! $CC $FLAGS $extra -o "$OUT/$name" "$@" -lm
    then
        FAILED="$FAILED $name(build)"
        return 1
    fi
    return 0
}

run ()
{
    name=$1
    echo "===== $name"
    RAN=$((RAN + 1))
    if ! "$OUT/$name"
    then
        FAILED="$FAILED $name"
    fi
}

build code39     ""     "$TEST_DIR/test_code39.c"     "$SRC/s3_code39.c" && run code39
build code39_all ""     "$TEST_DIR/test_code39_all.c" "$SRC/s3_code39.c" && run code39_all
build nav        ""     "$TEST_DIR/test_nav.c"        "$SRC/s3_code39.c" && run nav
build analog     ""     "$TEST_DIR/test_analog.c"     "$SRC/s3_analog.c" \
                                                      "$SRC/s3_code39.c" && run analog
build line       "$SAN" "$TEST_DIR/test_line.c"       "$SRC/s3_line.c" \
                                                      "$SRC/s3_code39.c" && run line
build sim        "$SAN" "$TEST_DIR/sim_track.c"       "$SRC/s3_line.c" && run sim
build probe      "$SAN" "$TEST_DIR/test_probe.c"      "$SRC/s3_probe.c" && run probe

if [ -n "$FAILED" ]
then
    echo "S3 HOST: FAIL ($RAN programs run; failed:$FAILED)"
    exit 1
fi
echo "S3 HOST: PASS ($RAN of 7 programs, all checks passed)"
exit 0
