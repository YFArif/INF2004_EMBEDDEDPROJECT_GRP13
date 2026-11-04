#!/bin/sh
# Run from WSL at the repository root. Never modifies the v28 sources.
set -eu
export PICO_SDK_PATH="${PICO_SDK_PATH:-$HOME/pico/pico-sdk}"
mode_list="${1:-1 2}"
case "$mode_list" in
    "1 2"|1|2|3) ;;
    *) echo "Use no argument, or 1, 2, or 3 (stationary view)." >&2; exit 1 ;;
esac
cfg=app_program/subsystem3/s3_experiment.h
if ! grep -q '^#define S3_EXPERIMENT_MODE (0)[[:space:]]*$' "$cfg"; then
    echo "Refusing: restore S3_EXPERIMENT_MODE to 0 first." >&2
    exit 1
fi
restore()
{
    sed -i 's/^#define S3_EXPERIMENT_MODE ([123])[[:space:]]*$/#define S3_EXPERIMENT_MODE (0)/' "$cfg"
}
trap restore EXIT
for mode in $mode_list; do
    sed -i "s/^#define S3_EXPERIMENT_MODE (0)[[:space:]]*$/#define S3_EXPERIMENT_MODE ($mode)/" "$cfg"
    log="/tmp/s3_probe_build_$mode.log"
    if ! make -C build_make -j8 > "$log" 2>&1; then
        tail -n 40 "$log"
        exit 1
    fi
    if grep -E 'warning:|error:' "$log"; then
        echo "Compiler diagnostics; image not promoted." >&2
        exit 1
    fi
    if [ "$mode" = 1 ]; then
        name=S3_v29_GP27_RECOVERY
    elif [ "$mode" = 2 ]; then
        name=S3_v29_ARMED_LEFT90
    else
        name=S3_v31_IR_GREY_ALERT
    fi
    cp build_make/mtk3pico_smp1_usb_cdc_wifi.uf2 "build_make/$name.uf2"
    echo "BUILT $name.uf2"
    restore
done
make -C build_make -j8 > /tmp/s3_probe_build_normal.log 2>&1
if grep -E 'warning:|error:' /tmp/s3_probe_build_normal.log; then
    exit 1
fi
echo "Normal firmware restored; experiment mode 0."
