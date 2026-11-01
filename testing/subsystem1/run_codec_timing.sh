#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
#
# run_codec_timing.sh -- builds the standalone S1-U08 image (on-target
# timing of msg_codec) and reports the codec's flash size.
#
# Run from the repository root, with PICO_SDK_PATH set and
# arm-none-eabi-gcc on the PATH:
#
#     sh testing/subsystem1/run_codec_timing.sh \
#         | tee testing/subsystem1/logs/YYYY-MM-DD_s1u08_build.txt
#
# What it does, all in a temporary copy (the repository is never changed):
#   1  git archive HEAD into a temporary folder (so only committed code is
#      built; the script refuses to run if the tested files have
#      uncommitted changes), with the .example config headers
#      (placeholder WiFi and broker).
#   2  Build NET=lwip MQTT=1 as it is: the baseline image.
#   3  Re-link that image with the 11 public msg_codec functions as
#      --undefined roots: the same image with the codec in it.
#   4  Copy codec_timing_main.c over the copy's app_program/app_main.c
#      (with its header beside it) and build again: the S1-U08 image.
#   5  Compile msg_codec.c alone at -Os, for the record.
# It prints four flash figures (report only, no gate):
#   - the image with the codec against the baseline (step 3 minus step 2);
#   - the placed msg_codec.o sections in the S1-U08 image's map;
#   - the S1-U08 image's own size;
#   - msg_codec.o at -Os.
# The UF2, ELF and map of the S1-U08 image go to
# docs/context/evidence-<date>/s1u08/ (git-ignored), or to $S1U08_OUT.
# Flashing and the serial capture are done by hand; see the end of the run.
#
# Environment: PICO_SDK_PATH (required), S1U08_OUT, TMPDIR.

set -u

TEST_DIR=testing/subsystem1
CODEC_DIR=app_program/subsystem1
ELF_NAME=mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt
ROOTS="msg_codec_put_telemetry msg_codec_get_telemetry"
ROOTS="$ROOTS msg_codec_put_heartbeat msg_codec_get_heartbeat"
ROOTS="$ROOTS msg_codec_put_command msg_codec_get_command"
ROOTS="$ROOTS msg_codec_put_ack msg_codec_get_ack"
ROOTS="$ROOTS msg_codec_put_event msg_codec_get_event"
ROOTS="$ROOTS msg_codec_cdeg_to_hdeg"

fail ()
{
    echo "S1-U08 build: FAIL ($1)"
    exit 1
}

if [ ! -f "$TEST_DIR/codec_timing_main.c" ] || [ ! -f "$CODEC_DIR/msg_codec.c" ]
then
    echo "run_codec_timing.sh: run this from the repository root" >&2
    exit 2
fi

if [ -z "${PICO_SDK_PATH:-}" ] || [ ! -d "$PICO_SDK_PATH/lib/lwip" ]
then
    echo "run_codec_timing.sh: PICO_SDK_PATH must point at a Pico SDK" >&2
    exit 2
fi

for tool in arm-none-eabi-gcc arm-none-eabi-size python3 git make
do
    if ! command -v "$tool" >/dev/null 2>&1
    then
        echo "run_codec_timing.sh: $tool is not on the PATH" >&2
        exit 2
    fi
done

REPO=$(pwd)
DIRTY=$(git status --porcelain -- "$TEST_DIR/codec_timing_main.c" \
        "$TEST_DIR/codec_timing_main.h" "$TEST_DIR/run_codec_timing.sh" \
        "$CODEC_DIR/msg_codec.c" "$CODEC_DIR/msg_codec.h" \
        "$CODEC_DIR/msg_layout.h")
if [ -n "$DIRTY" ]
then
    echo "$DIRTY"
    fail "the tested files have uncommitted changes; commit them first"
fi

DATE=$(date '+%Y-%m-%d')
OUT=${S1U08_OUT:-docs/context/evidence-$DATE/s1u08}
mkdir -p "$OUT" || fail "cannot create $OUT"
if ! git check-ignore -q "$OUT/x.uf2"
then
    fail "$OUT is not git-ignored"
fi

WORK=$(mktemp -d "${TMPDIR:-/tmp}/s1u08.XXXXXX") || exit 2
trap 'rm -rf "$WORK"' EXIT
trap 'exit 1' HUP INT TERM
TREE="$WORK/tree"
mkdir -p "$TREE"

echo "S1-U08 run_codec_timing.sh, $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo "commit: $(git rev-parse --short HEAD)"
echo "arm-none-eabi-gcc: $(arm-none-eabi-gcc --version | head -n 1)"

# ---------------------------------------------------------------- step 1
git archive HEAD | tar -x -C "$TREE" || fail "git archive"
cp "$TREE/config/wifi_secrets.example.h" "$TREE/config/wifi_secrets.h"
sed 's/^\(#define MQTT_LINK_CFG_SET  *\)0$/\11/' \
    "$TREE/$CODEC_DIR/mqtt_config.example.h" >"$TREE/$CODEC_DIR/mqtt_config.h"
if cmp -s "$TREE/$CODEC_DIR/mqtt_config.example.h" \
          "$TREE/$CODEC_DIR/mqtt_config.h"
then
    fail "could not enable the .example MQTT config"
fi
echo "step 1: exported HEAD with placeholder configs"

# build LOG: build NET=lwip MQTT=1 in the copy; fail on any warning.
build ()
{
    (cd "$TREE/build_make" && make NET=lwip MQTT=1 -j8) >"$1" 2>&1 \
        || { tail -n 20 "$1"; fail "build, see above"; }
    if grep -qiE 'warning|error' "$1"
    then
        grep -iE 'warning|error' "$1" | head -n 20
        fail "the build printed warnings"
    fi
}

# size_of ELF: "text data bss" of an ELF.
size_of ()
{
    arm-none-eabi-size "$1" | tail -n 1 | awk '{ print $1, $2, $3 }'
}

# ---------------------------------------------------------------- step 2
build "$WORK/base_build.log"
cp "$TREE/build_make/$ELF_NAME.elf" "$WORK/base.elf"
BASE=$(size_of "$WORK/base.elf")
echo "step 2: baseline image built, no warnings ($BASE text/data/bss)"

# ---------------------------------------------------------------- step 3
(cd "$TREE/build_make" && make -n -B NET=lwip MQTT=1 2>/dev/null) \
    | grep -- '-Wl,-Map' >"$WORK/link_cmd.txt"
python3 -I - "$WORK/link_cmd.txt" "$WORK/rooted_link.sh" $ROOTS <<'EOF'
import re
import sys

cmd = open(sys.argv[1]).read().strip()
roots = sys.argv[3:]
assert len(re.findall(r'-Wl,-Map,"[^"]+\.map"', cmd)) == 1
assert len(re.findall(r'-o "[^"]+\.elf"', cmd)) == 1
cmd = re.sub(r'-Wl,-Map,"[^"]+\.map"', '-Wl,-Map,"codec_rooted.map"', cmd)
cmd = re.sub(r'-o "[^"]+\.elf"', '-o "codec_rooted.elf"', cmd)
cmd = cmd.replace(' -Wl,--gc-sections', ' -Wl,--gc-sections '
                  + ' '.join('-Wl,--undefined=' + r for r in roots), 1)
open(sys.argv[2], 'w').write(cmd + '\n')
EOF
[ -s "$WORK/rooted_link.sh" ] || fail "could not build the rooted link line"
(cd "$TREE/build_make" && sh "$WORK/rooted_link.sh") >"$WORK/rooted.log" 2>&1 \
    || { cat "$WORK/rooted.log"; fail "rooted link"; }
ROOTED=$(size_of "$TREE/build_make/codec_rooted.elf")
echo "step 3: baseline re-linked with the 11 codec functions as roots"

# ---------------------------------------------------------------- step 4
cp "$REPO/$TEST_DIR/codec_timing_main.c" "$TREE/app_program/app_main.c"
cp "$REPO/$TEST_DIR/codec_timing_main.h" "$TREE/app_program/"
build "$WORK/s1u08_build.log"
cp "$TREE/build_make/$ELF_NAME.uf2" "$OUT/s1u08_codec_timing.uf2"
cp "$TREE/build_make/$ELF_NAME.elf" "$OUT/s1u08_codec_timing.elf"
cp "$TREE/build_make/$ELF_NAME.map" "$OUT/s1u08_codec_timing.map"
TEST_IMG=$(size_of "$OUT/s1u08_codec_timing.elf")
PLACED=$(python3 -I - "$OUT/s1u08_codec_timing.map" <<'EOF'
import re
import sys

lines = open(sys.argv[1]).read().split('\n')
start = next(i for i, l in enumerate(lines)
             if l.startswith('Memory Configuration'))
total = {'text': 0, 'data': 0, 'bss': 0}
i = start
while i < len(lines):
    m = re.match(r'^ (\.(text|rodata|data|bss)\S*)(?:\s+0x[0-9a-f]+'
                 r'\s+(0x[0-9a-f]+)\s+(\S+))?\s*$', lines[i])
    if m:
        size, obj = m.group(3), m.group(4)
        if size is None and i + 1 < len(lines):
            n = re.match(r'^\s+0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+(\S+)',
                         lines[i + 1])
            if n:
                size, obj = n.group(1), n.group(2)
                i += 1
        if size and obj.endswith('/msg_codec.o'):
            kind = m.group(2)
            key = 'text' if kind in ('text', 'rodata') else kind
            total[key] += int(size, 16)
    i += 1
print(total['text'], total['data'], total['bss'])
EOF
)
echo "step 4: S1-U08 image built, no warnings"

# ---------------------------------------------------------------- step 5
ARM="-mcpu=cortex-m0plus -mthumb -ffreestanding -std=gnu11 -g3"
ARM="$ARM -mfloat-abi=soft -DNDEBUG -ffunction-sections -fdata-sections"
ARM="$ARM -DTM_MQTT=1"
arm-none-eabi-gcc $ARM -Os -c "$REPO/$CODEC_DIR/msg_codec.c" \
    -o "$WORK/msg_codec_os.o" || fail "-Os compile of msg_codec.c"
OS_SIZE=$(size_of "$WORK/msg_codec_os.o")
echo "step 5: msg_codec.c compiled alone at -Os"

# ---------------------------------------------------------------- report
python3 -I - "$BASE" "$ROOTED" "$PLACED" "$TEST_IMG" "$OS_SIZE" <<'EOF'
import sys

base, rooted, placed, test_img, os_size = (
    [int(x) for x in a.split()] for a in sys.argv[1:6])
d = [rooted[i] - base[i] for i in range(3)]
print("== flash and RAM, report only (text/data/bss, bytes)")
print("1. image with the codec vs baseline: %d/%d/%d vs %d/%d/%d,"
      " difference %+d/%+d/%+d" % tuple(rooted + base + d))
print("2. msg_codec.o sections placed in the S1-U08 image: %d/%d/%d"
      % tuple(placed))
print("3. S1-U08 image (test app instead of app_main.c, for the record):"
      " %d/%d/%d" % tuple(test_img))
print("4. msg_codec.o compiled alone at -Os (for the record): %d/%d/%d"
      % tuple(os_size))
EOF

echo "== S1-U08 image: $OUT/s1u08_codec_timing.uf2"
echo "sha256: $(shasum -a 256 "$OUT/s1u08_codec_timing.uf2" | cut -c1-64)"
echo "To flash: hold BOOTSEL, plug in the Pico W, then"
echo "    cp $OUT/s1u08_codec_timing.uf2 /Volumes/RPI-RP2/"
echo "To capture (start it first; the app waits 5 s, then reports every 30 s):"
echo "    cat /dev/cu.usbmodem* | tee $TEST_DIR/logs/YYYY-MM-DD_s1u08_codec_timing.txt"
echo "S1-U08 build: OK"
exit 0
