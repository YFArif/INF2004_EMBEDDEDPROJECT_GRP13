# Subsystem 3 (Buddy 3, Sean): line following, barcode, junction turns

S3 follows the black line with two IR sensors, reads Code 39 barcodes,
detects junctions and turns at the next junction as the last barcode says.
Older notes (edge-pair layout, floor experiments) are in the git history.

## Files

| Group | Files | What they do |
|---|---|---|
| Line following | `s3_line.c/.h` | Calibration and the single-edge PD controller (pure C, laptop-tested) |
| | `task_line.c` | Runs `s3_line_step()` every 10 ms, publishes steering to S2 |
| | `drv_adc.c/.h` | 0.5 ms sampler: line sensors and barcode AOUT |
| Barcode | `drv_barcode.c/.h` | DOUT (debounced in the sampler) and AOUT capture, one pass at a time |
| | `s3_analog.c/.h` | Analogue stripe widths, flicker merge |
| | `s3_code39.c/.h` | Code 39 decode (29 stripes, both directions), letter -> command map |
| | `task_barcode.c` | Takes each pass, decodes, publishes the NAV command |
| Track test | `task_drive_test.c` | `DRIVE=1` build: buttons, calibration sweep, logging, junction stop and turn |
| Team interface | `s3_api.c/.h`, `subsystem3.c/.h`, `s3_private.h` | IF-01 / IF-02 to S2 and S5, status, mailboxes |
| | `s3_config.h` | Pins, layout, periods, drive-test settings |
| Course template | `task_consumer.c/.h`, `subsystem3_test.c` | Week template objects still used by `app_main.c` and S2 |

## Sensor layout (single edge)

Looking down, car centred on the line:

```
   white   GP27 |######## 18 mm line ########|   white   GP26
           (on the left edge, grey ~500)            (guard, on white,
                                                    25-30 mm from GP27)
```

- GP27 ("Centre IR") does the steering: grey = straight, darker = left,
  lighter = right.
- GP26 ("Left IR", mounted on the robot's RIGHT) is the guard: it sees
  black only at a junction or when the car is far off.
- Mirrored mount: set `S3_IR_L_SIDE` to `S3_LN_L_ON_LEFT` in `s3_config.h`.

## Track test build

In WSL, from `build_make`, with `PICO_SDK_PATH` set:

```
make DRIVE=1 DRIVE_SPEED=80 S3_BC_VERBOSE=1
```

Output: `mtk3pico_smp1_usb_cdc_wifi_bcverbose_drive80.uf2`. S5 disables
itself in this build (no ultrasonic guard: GP20 stops the car).

1. Power on, car centred on the line. Calibration starts after 1 s: the
   car turns left, right, then back until GP27 is on the left edge.
2. Wait for `CALIBRATION READY`. GP21 tap = next preset, hold 2 s =
   recalibrate.
3. Tap and release GP20: 3 s countdown, then it drives. GP20 again stops.

Presets (GP21, while waiting):

| Preset | Kp / Kd | Junction with no barcode read |
|---|---|---|
| 0 (default) | 25 / 25 | stop, then straight |
| 1 | 25 / 25 | turn left (turn test) |
| 2 | 25 / 25 | turn right (turn test) |
| 3 | 25 / 25 | U-turn (turn test) |
| 4 | 20 / 0 | stop, then straight |

A barcode that is read always wins: A = left, B = right, C = straight,
D = U-turn (map in `s3_code39.c`, settable at run time).

At every junction the car stops. If a barcode is still being read it waits
up to 2 s for the result, then drives 125 mm so the axle is on the
crossing, turns on the spot and follows again.

## Laptop tests

From the repository root, in WSL:

```
sh testing/subsystem3/run_host_tests.sh
```

Unit tests: `test_line`, `test_code39`, `test_code39_all`, `test_nav`,
`test_analog`. `sim_track` is a rough track simulation (car settings, 1x
steering): it still fails sharp 90 deg corners without a barcode turn and
some bump recoveries; the car log is the real check.

## Known limits

- One edge sensor only sees about +-2-3 mm, so steering is twitchy.
- The car cuts curves a little: the sensors are ahead of the wheels.
- Line lost on the guard side never reports LOST (the car keeps turning
  towards the guard), so the sweep recovery does not run there.
- Barcode reads are the weak point: many passes are rejected.
