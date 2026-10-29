# Subsystem 3 (Buddy 3, Sean): integration guide

S3 covers IR line following, line-lost detection (junction detection remains off), Code 39 barcode decoding, and the navigation command.

## Sean edge-pair baseline: isolated speed test (29 Oct)

Current normal controller is Sean's `22fda8a` edge-pair update, not the
older single-edge v24 controller. Use **18 mm sensing-centre spacing**,
one sensor on each boundary of an 18 mm black line: GP27 physical LEFT,
GP26 physical RIGHT, both approximately grey when centred. Preserve the
existing sensor height that gave usable analogue transitions.

Build in WSL from `build_make`, with `PICO_SDK_PATH` set:

```
make -j8 DRIVE=1 DRIVE_SPEED=10
```

Output: `mtk3pico_smp1_usb_cdc_wifi_drive10.uf2`. Use `DRIVE_SPEED=40`
for a separate `_drive40.uf2`. The build profile tracks both mode and speed
to prevent reuse of stale objects. Default `make -j8` remains the normal
integrated image, **not** this slow test; S5's normal mission speed is unchanged.

The test excludes S5 tasks: previously both S3 drive and S5 could read GP20
and issue competing FOLLOW commands, including S5's 300 mm/s request.
**No ultrasonic protection is active in this isolated test.** Stop with
GP20; test in clear space with access to power.

Startup begins calibration automatically, but the sensors still have to
see white and black: sweep the car sideways by hand, wait for READY, then
centre the sensors on their edges. GP20 starts after the countdown and
stops while running. GP21 while running toggles base / 1.5x speed
(10/15 or 40/60 mm/s), on release after at least 60 ms pressed. Steering
limits scale with speed. Every new run starts unboosted. While stopped,
GP21 retains Sean's gain preset selection / hold-to-recalibrate behaviour.
The existing 3 m, prolonged LOST and no-odometry stop guards remain.

S2 now accepts real encoder intervals up to its existing 100 ms stopped
threshold, rather than substituting 20 ms for intervals over 40 ms.
This matters at very low speed; no wheel PID gains were changed.
Host tests check actual model displacement over five seconds and exact
wheel targets at 10/15/10/40/60/40 mm/s, including live FOLLOW updates.
These are simulation checks, not a physical speed calibration. Record
the `S3 DRIVE GO`, `S3 DRIVE SPEED`, and wheel target/measured/duty logs
and time a measured straight distance on the car before claiming accuracy.

Older sections below describe preserved diagnostic/experimental images;
they are not the current edge-pair test layout.

## v31 live WHITE/GREY/BLACK alerts (29 Oct)

Build `S3_v31_IR_GREY_ALERT.uf2` with `sh testing/subsystem3/build_probe.sh 3`.
Same motor-locked viewer and GP20 white / GP21 black reference capture as v30.
Each sensor is classified from the latest ADC snapshot every 20 ms, not
from the half-second mean. Entering grey prints `>>> GREY DETECTED` with
sensor pin, time, raw and normalised value. Other zone transitions print
`IR CHANGE`. No hysteresis is applied: repeated alerts near a threshold
expose fluctuations rather than hide them. USB output can add latency.
Periodic output twice per second shows the current zone/raw/normalised
reading and separately the window mean/min/max.
Thresholds: WHITE <250, GREY 250..750 inclusive, BLACK >750 (0..1000).
Missing/invalid calibration is UNCAL/INVALID, never a guessed colour.
Recapturing a reference invalidates classification until capture finishes.
Grey is intermediate reflectance, not proof of a physical edge.

## v30 stationary IR height check (29 Oct, preserved image)

Flash `build_make/S3_v30_IR_HEIGHT_VIEW.uf2`, built with
`sh testing/subsystem3/build_probe.sh 3` in WSL. Mode 3 starts the
sensor viewer instead of the follower; S5 tasks remain excluded.
The viewer latches S2 E-stop at entry and never clears it or sends
FOLLOW/MOVE/TURN. Neither button starts the motors.

At operating height, place BOTH sensors over white, then tap/release
GP20 and hold still for the next 0.5 s. Repeat over a sufficiently wide
black patch with GP21. References remain fixed until captured again.
Readings print twice per second: GP27 physical LEFT and GP26 physical
RIGHT, approximate 12-bit raw mean/min/max (from existing averaged ADC
snapshots), and normalised `n` from 0 white to 1000 black. `n=-1` is
explicitly invalid, not white. Contrast below 600 counts prints an error.
This viewer does not discard high raw readings: lifting the car can give
misleading dark readings, so keep the wheels/sensors at operating height.

Then position GP27 over the left edge of an 18 mm line and GP26 2 mm
outside its right edge (20 mm sensing-centre spacing). Slide sideways
slowly and compare the readings. Neither exactly 500 at the boundary nor
improved accuracy is guaranteed: this measures response/contrast at the
new height, not absolute position accuracy. Power-cycle into a different
image to drive again. The build script restores normal mode 0.

## v29 isolated GP27 experiment (29 Oct, Arif)

The centred-pair controller and v28 drive test are preserved, not replaced.
`s3_experiment.h` defaults to mode 0 (normal firmware). Mode 1 selects
`S3_v29_GP27_RECOVERY.uf2`; mode 2 selects `S3_v29_ARMED_LEFT90.uf2`.
Build both from the repository root in WSL with
`sh testing/subsystem3/build_probe.sh`. It restores mode 0 and rebuilds normal
firmware afterwards. Do not combine with `S3_DRIVE_TEST=1` or `APP_TEST=1`.

**Experimental wiring/layout:** GP27 is the physical LEFT sensor, over the
18 mm black line. GP26 is RIGHT, on white. Arif specifies 20 mm clear gap
between the black sensor bodies, not centre-to-centre. Both sensors must be
at the same forward position and operating height. The measured GP27 lead
ahead of the drive-wheel axle line is 130 mm. The alignment estimate is
139 mm from the first both-black sample (130 plus half the crossing width).
Optical footprint, lateral offset and braking travel can require calibration.

**Isolation/safety:** these are supervised clear-floor experiments, not
integrated mission firmware. S5's tasks (including ultrasonic protection)
are disabled so S5 cannot also command the motors or read the buttons.
The normal S3 line task is not started. S2 remains the only motor driver;
its encoder control, E-stop latch and stall handling remain intact.
GP20 is start/stop; the physical power switch remains the emergency backup.
There is no overall run timer or 3 m limit. Individual manoeuvres have
8 s timeouts; no forward odometry for 2 s, stopped ADC updates, or task
lateness over 120 ms latch E-stop. Command-send failure also latches E-stop.

**Calibration/start:**

1. Flash image 1 first. Open the serial monitor and wait for SWEEP NOW.
2. Sweep BOTH sensors across white and black at operating height. Once
   `ready=1`, tap/release GP20 to lock calibration. Learning then stops.
3. Place GP27 on black, GP26 on white. Tap/release GP20 again to start
   after a three-second countdown. A GP20 press cancels the countdown.
4. GP20 while running stops. GP21 held while waiting recalibrates.
   E-stop is never automatically cleared; power-cycle after a latched fault.

**Follow/recovery:** straight FOLLOW at 40 mm/s while GP27 remains dark.
Below 400/1000, request STOP on the first sampled loss, wait for S2 idle
and 120 ms settling, then search using relative turns
`+4, -8, +16, -24, +12` degrees at 60 mm/s wheel speed. The intended
headings relative to the loss heading are `+4, -4, +12, -12, 0`.
More than 18 degrees measured deviation stops the search. Three dark
samples (>=650) trigger a brake and a second stationary confirmation
before resuming. Five unsuccessful legs stop with NOT_FOUND.
This is deliberately stop-and-search, not smooth PD edge following:
one fully black sensor does not identify which direction the line moved.

**Armed left turn (image 2):** tap/release GP21 during FOLLOW to arm ONE
left turn. Require at least 100 ms of GP26 white after arming, then both
sensors >=650 for at least 2 mm of travel. Unarmed both-black and
both-black during recovery do not trigger turns. Additional GP21 taps
while already armed do not restart the detector.
Continue straight to the estimated axle alignment; losing GP27 during
alignment stops rather than turning blind. Brake, request S2 TURN +90,
wait for completion, and verify/reacquire the outgoing line using GP27.
The instruction is consumed once and following resumes; press GP21 again
for the next crossing. This tests a button instruction, NOT the barcode/S5
bridge. Darkness is only a crossing candidate under the stated track
assumptions, not a universal proof of a junction.

GP27 is NOT guaranteed to remain black during a pivot. Encoder angle
accuracy, turn placement, search speed and real line reacquisition need
hardware tests. Test on a straight with an artificial loss before trying
one isolated cross. Logs show state, armed flag, both normalised sensors,
distance, heading, completion counter, wheel targets and measured speeds.

## v27 drive test (28 Oct, Arif): quiet, clearly separated test stages

- Flash `build_make/S3_v27_CLEAR_STEPS.uf2` for this test.
- Background `S3 LINE` and `S3 TIME` prints are disabled in drive-test builds only. Normal builds retain them. Waiting output prints only when calibration readiness changes, not whenever a sensor level changes.
- Follow the large banners: **STEP 1 GET READY** (10-second countdown), **STEP 2 SWEEP NOW**, **STEP 3 CALIBRATION READY** (park both sensors on black, then tap and release GP20), **STEP 4 RUNNING** (GP20 stops).
- READY requires both black levels >= 2500 and both white-to-black ranges >= 600, matching the controller's contrast requirement. Readiness is not confirmation that the car is positioned on the line.
- After a run, the stop summary is followed by the waiting prompt. Calibration does not automatically reset after a run. Hold GP21 for 2 seconds when waiting to restart calibration.
- Detailed `S3 LOG` output remains at 100 ms while driving; steering, speed and safety limits are unchanged.

## v26 drive test (28 Oct, Arif): countdown, one-press start, run logs

- **Calibration countdown:** at power-up (and when asked) the serial monitor counts down `S3 CAL 10 ... 1`, then forgets the old levels and prints `SWEEP NOW`: sweep until `S3 DRIVE CAL ... READY`.
- **GP20: tap and release to start** (no long hold), then a 3-second countdown and it drives. Release before the countdown finishes: a held button stops the run. Press GP20 while driving to stop. Pressing START before READY starts a new calibration countdown instead.
- **GP21:** press = next gain preset; hold 2 s = recalibrate (with the countdown).
- **`S3 LOG` every 100 ms while driving:** time, distance, left / right sensor (0 white ... 1000 black) and zone (B / G / W), line state, dv sent to S2, each wheel's target speed, measured speed and PWM duty, encoder ticks and heading.
- **`S3 SUMMARY` + `S3 DIAG` after every run:** % of time both sensors black / one white / both white (longest both-white stretch), steering side changes per metre, mean wheel speed error. `S3 DIAG` lines name the likely cause: SENSORS (both seldom black together: too far apart), LINE BETWEEN SENSORS, OVERSTEER / WEAVE, or MOTORS (wheels not following their set-points).

## v25 (28 Oct, Arif): BOTH SENSORS ON THE LINE (replaces the edge-riding layout)

**Re-mount the two line sensors before flashing v25.** Both now sit over the black line, **12 mm apart centre to centre**, the pair centred on the car (6 mm each side of the car's centre line), same height, side by side:

    white |####   GP27      GP26   ####| white     (car driving up the page)
                  <--- 12 mm --->
          line 18 mm wide; each sensor about 3 mm inside its edge

- **How it steers:** e = (right sensor darkness) − (left sensor darkness). Both black = centred, go straight. The car drifts → the sensor on that side reaches the edge and greys → it steers back, harder the further it drifts. One sensor white + the other black = full lock. Left and right corners use the same rule, so they behave the same.
- **Line lost (both white):** full lock towards the side the line was last seen on (the sensor that stayed black longest). There is no near-edge / far-edge to confuse, which is what made the old layout spin. After 40 mm both white the state is LOST (event raised), still turning that way until a sensor sees the line.
- **Which sensor is where:** on our car GP26 is on the robot's RIGHT and GP27 on its LEFT (`S3_IR_L_SIDE` in `s3_config.h`). If you mount them the other way round, set `S3_LN_L_ON_LEFT`. Wrong setting = the car steers away from the line at once.
- **Gains:** Kp 100, Kd 0 by default (full lock when one sensor is white). Drive-test presets: 0 = 100/0, 1 = 75/0 softer, 2 = 150/0 stronger, 3 = 100/25, 4 = 200/0. All at 40 mm/s, steer max 40, S2 corner slow-down on.
- **Junctions are NOT detected in this layout:** both sensors are black on every part of the line, so a crossing looks like plain line. `junction_event` is always 0. FR3 junction reporting needs another method (e.g. the barcode before each junction); the design report's "sensors more than 18 mm apart" rule no longer applies and must be updated.
- **Host evidence** (`testing/subsystem3/logs/2026-10-28_host_v25.txt`, model only): 6 tracks × 3 lookaheads × 40 and 80 mm/s × 4 mountings (12 mm both ways round, 10 mm, 14 mm) all pass; sharp LEFT and RIGHT 90 give identical errors; start 8–15 mm off the line pulls on; wheel lag 40–120 ms + noise + 8-bit readings stay within 30 mm; 216 of 216 bump/slip runs finish. Not yet run on the car.
- Removed: the straddle rules (OUTER push, far-edge flag, tie / last-seen rules, junction distance trackers and `S3 JDBG` prints). Calibration (incl. the lifted-car guard), 4-sample averaging, the drive test (40 mm/s, no time limit) and the barcode path are unchanged.
## 1. What to copy into the team repo

Copy the whole folder `app_program/subsystem3/` and the folder `testing/subsystem3/host/`. Nothing outside these two folders needs to change. The build compiles every `app_program/*/*.c` (`build_make/mtkernel_3/app_program/subdir.mk`), and `app_main.c` already calls `subsystem3_init()` and `subsystem3_start()`.

v23 (28 Oct, Arif):

- **"Full black reads 60 % grey" fixed.** A friend's `S3 VIEW` log showed tape at 2500–2900 raw but a learned black of ~4050: the car had been lifted during the calibration sweep (no floor = darkest reading). The controller then saw the tape as ~60 %, never "black" (> 750): junctions could not fire, and a Centre fully on the line looked only slightly past the edge, so steering was weak and the car drifted across the line in corners. Calibration now ignores readings ≥ 3700 raw (`S3_IR_CAL_LIFT_RAW`). Test S3-L41. **Still sweep with the wheels on the floor.**
- **Corner slow-down (in Buddy 2's S2 FOLLOW).** Speed drops as steering grows: speed − (|steer| − 10) mm/s, never below 70 % of speed (`S2_CORNER_*` in `subsystem2.h`). Straight-line tracking keeps full speed. Simulator: sharp-left error 45 → 11 mm on our 20 mm layout; every track and bump test passes. Not yet run on the car.

v22 (28 Oct, Arif with Sean's consent; normal driving, fixes the sudden 180s / circles / wrong-way turns seen on the car):

- **Cause found (host model, not yet confirmed on the car).** The Centre rides the line's LEFT edge. If a bump, slip or curve carries it right across the 18 mm line, it comes out on the FAR (right) edge, which also reads half black. When both sensors then go white, `off_line_dir()` rule 3 decides which way to turn using a "Centre went dark" flag, and v20 cleared that flag after only 5 mm of mid grey. A slow exit over the far edge passes that much mid grey, so the car turned RIGHT, away from the line, at full lock, and LOST kept turning: a 180 or a circle.
- **Fix:** the flag (now `c_far`) is cleared only when the outer sensor is darker than WHITE (that can only happen on the near side) or after the Centre sat **40 mm** in mid grey (`S3_LN_FAR_SETTLE_MM`; the far edge is unstable under the PD, so a long settle means the near edge, e.g. the new line after a sharp corner). 5, 10 and 20 mm still fail the new stress test; 40 passes it.
- **New simulator stress test S3-S92:** one bump (heading ±15–35° or slide ±15–30 mm, spread over 0.3 s) on a straight and two S-bends, 3 lookaheads, wheel lag 60 / 120 ms, noise, 4-sample averaging, 8-bit readings: 216 runs. v21: 202 of 216 finish (the 14 failures are 180s / circles). v22: **216 of 216**. All 18 track cases still pass.
- **New unit tests S3-L38 … L40:** a slow far-edge exit turns back LEFT (fails on v21), and the two ways the flag must still clear (settled on a new edge; outer sensor grey) turn RIGHT.
- **Tried and NOT adopted:** blocking all PD steering towards the outer side while the flag is set. It broke the sharp LEFT 90° case (S3-S05), where the Centre legitimately crosses to the new edge.
- **Drive-test preset 1 is now Kp 150 / Kd 50** (was 200 / 50). In a host gain sweep it had the fewest steering sign flips of the gains that still pass every track; 200 was barely different from the default 250. The default is unchanged (250 / 50). The model does not reproduce the visible weave on the car, so use preset 1 vs 0 on the car to decide.
- LOST recovery itself (full lock until the correct edge is found) is unchanged; it now starts in the right direction.

v21 (28 Oct, Arif with Sean's consent; normal driving only, the LOST / recovery logic is unchanged):

- **Fewer files (36 → 23 in this folder).** Small files that had one user were merged into it. Nothing outside S3 changes, and every public function keeps its name:
  - `drv_ir_left` and `drv_ir_centre` (two 40-line drivers) → `drv_adc`, which already owned the ADC and the sampler interrupt that reads them. `drv_adc_ir_read()` returns both sensors from the same sample.
  - `s3_ir_cal` (one sensor's calibration) → `s3_line`, its only user.
  - `s3_nav` (letter → command map) → `s3_code39`, and reformatted to the BARR-C layout used everywhere else.
  - `task_line.h`, `task_barcode.h`, `task_drive_test.h` (one prototype each) → `s3_private.h`.
  - `S3_CHECKLIST.md` and `S3_DATA_REPRESENTATION.md` → `testing/subsystem3/`, with the other test evidence.
  - The host tests give byte-identical output before and after the merge.
- **Line IR oversampling (the one behaviour change).** Each 10 ms reading is now the average of 4 ADC samples taken 2.5 ms apart, instead of one sample. Noise was turning into left/right steering flips. In the simulation (new check S3-S91: wheel lag 40–120 ms, 8-bit readings, noise ±40/±80 raw) steering sign flips drop from 26 % to 17 % of steps and the Centre stays on the edge more often. Readings are still published every 10 ms (NFR1) as 8 bits (Change Requirement 1), and the longest sampler interrupt is expected to stay about the same (not measured on the Pico). Not yet run on the car.
- **Tried and not adopted:** letting S2 change the steering faster than its 30 mm/s per 20 ms ramp. It made no measurable difference in S3-S91, so Buddy 2's code is unchanged.
- **New runner:** `sh testing/subsystem3/run_host_tests.sh` builds and runs all six host programs with `-Werror` (the line test and simulation under sanitizers).

v16 splits the old `s3_bringup.c` into one driver per sensor, a combiner and two tasks, in BARR-C:2018 style (deviations are listed at the top of `subsystem3.h`). The line and barcode logic gives byte-identical host-test output to v15; the on-car behaviour still needs re-checking. The desk-test builds (`S3_TEST` 1 to 4) are removed.

v20 (28 Oct, after the v19 car runs at 20 mm, which tracked 2.1 m but shook):

- **Gains Kp 250 / Kd 50** (were 150 / 400): stronger in the edge band, and the D term no longer turns sensor noise into left/right flips.
- **OUTER push grows with the outer reading**: 600 × (n_outer − 250) / 750 ‰, instead of a fixed 600 once the outer sensor is black.
- **OFF LINE / LOST full lock is 1.25 × steer max** (was 2 ×, a pivot that swept past the line).
- **Drive test:** hold Button 1 for 4 s to recalibrate (no power cycle); `S3 DRIVE CAL` prints both sensors' levels; a start is refused until both black levels are ≥ 2500 raw. Five presets that change only Kp / Kd.
- `S3_DRIVE_TEST` is back to **0** in the committed `s3_config.h` (v19 was committed with 1 by mistake).

v19 (28 Oct, after the v18 car runs):

- **New line rules (straddle layout).** The Centre rides the line's LEFT edge, and the outer sensor (on the robot's right) sits just outside the RIGHT edge, so it is a real steering input.
  - Outer black means the car drifted that way: steer towards the outer sensor.
  - Both white: steer full lock towards the sensor that saw the line last. The direction is decided once and held.
  - After LOST, the car keeps turning until the Centre is back on the CORRECT edge.
- **Steering scaled to speed.** `s3_set_steer_max()`; the drive test sets it equal to the speed. In v18 the full-lock difference was a fixed 200 mm/s, so at 80 mm/s the inner wheel ran backwards, the car hunted from black to white every 200 ms, and a wheel stalled.
- **Corner rule removed.** In the car log it fired on normal hunting and forced right turns.
- **The drive test no longer clears S2's E-stop** (Buddy 2's rule); it reports it and refuses to start.
- **New pins:** outer IR GP26 (ADC0), Centre GP27 (ADC1), barcode AOUT GP28 (ADC2), DOUT GP7. Compile-time checks catch a pin and ADC channel that do not match, or two sensors on one channel.
- **New host track simulation** (`host/sim_track.c`): the real `s3_line.c` drives a model car over a straight, curves, sharp 90° turns and an S-bend.

v18 (28 Oct, after the car tests): the **curve boost is off by default**, because v15b's steering tracked well and v17 spun. There is a new optional **corner rule** for right and sharp turns, `s3_set_line_shape(boost, corner)` sets both at run time, and the drive test is reworked (section 6). Arif's S2 `4690bd9` (right TURN calibration) is merged. Host tests S3-L30 … L32 are new.

v17 (28 Oct) changes on top of v16. All four existing host tests give byte-identical output; two tests are new (S3-L29, `test_code39_all.c`):

- **Priorities** now match the Week 6 report (section 3): `task_line` 8, `task_barcode` 12. Before, they were the template band 16 and 17, which put the 10 ms line task below the 20 ms `motion_task` (9).
- **Sampler interrupt (2 kHz):** a countdown replaces `idx % 20`. The Cortex-M0+ has no divide instruction, so `%` called the library divide 2000 times a second.
- **Line step:** 32-bit maths instead of a 64-bit multiply and divide every 10 ms. This also fixes a real overflow: after a late wake-up (dt of a few µs), `Kd × d_err` overflowed 32 bits and flipped the D term's sign (host test S3-L29; UBSan flags the old code).
- **FR5:** a letter that is not in today's map is decoded and shown in the status, but is no longer sent to `mbf_nav` as a `NONE` command.
- `s3_gains_request_take()` takes `mtx_s3` only when a request is pending, not every 10 ms.
- **`S3_TIMING` (on):** prints measured timings on the car (section 7).
- **Interrupt code in RAM** (`S3_ISR_IN_RAM`, Tribunal 1 Version C): ready, but **off**, because it needs one line in the team linker script (see `drv_adc.h`).
- **New host test `test_code39_all.c`:** every character in both directions at 12 print and speed settings, and the table checked against an independent reference.

| File | Layer | What it does |
|---|---|---|
| `subsystem3.c` / `.h` | build | `subsystem3_init()` → `s3_init()`, `subsystem3_start()` → `s3_start()`. File map and BARR-C deviations. |
| `s3_api.h` / `s3_api.c` | interface | Public interface (other subsystems include `../subsystem3/subsystem3.h`), kernel objects, status, steering word. |
| `s3_config.h` | settings | Priorities, periods, edge side, debug prints, `S3_DRIVE_TEST`. |
| `s3_private.h` | internal | Helpers and task start-up shared inside S3 only. |
| `drv_adc.c` / `.h` | driver | ADC owner, the 2 kHz sampler interrupt, and both line IR sensors: outer ("Left IR") GP26 / ADC0 on the robot's right, Centre GP27 / ADC1. Each 10 ms reading averages 4 samples. |
| `drv_barcode.c` / `.h` | driver | Barcode sensor: DOUT GP7 edge interrupt (on S2's shared handler), AOUT GP28 / ADC2 sample ring, pass detection. |
| `s3_line.c` / `.h` | logic (pure C) | Per-sensor calibration (filter, learn, relax, normalise) and the combiner: PD steering, curve boost, overshoot, junction, lost, search. |
| `s3_code39.c` / `.h` | logic (pure C) | Code 39 decoder (all 43 characters plus `*`, forwards and backwards) and the run-time letter → command map. |
| `s3_analog.c` / `.h` | logic (pure C) | Analogue barcode widths. |
| `task_line.c` | task | 10 ms line task (core 1, priority 8). |
| `task_barcode.c` | task | Hardware set-up, then barcode decode, fusion and publish (core 1, priority 12). |
| `task_drive_test.c` | test | Only with `S3_DRIVE_TEST 1`. |
| `task_consumer.c` / `.h`, `subsystem3_test.c` | template | Template demo objects (`mbx_samples`, `mtx_stats`, `app_stats`). Subsystems 2 and 4 still use them, so **do not delete** these until the template demo is removed. |
| this file | docs | not compiled |
| `testing/subsystem3/` | tests and evidence | `run_host_tests.sh` and `host/*.c` (laptop tests, no Pico), `S3_CHECKLIST.md`, `S3_DATA_REPRESENTATION.md`, `TEST_LOG.md`, `S3_ATTEMPTS_LOG.md`, `logs/`. |

## 2. Wiring and sensor layout (robot, v25)

| Sensor | Signal | Pin | Where on the car |
|---|---|---|---|
| Line IR "Centre" | AOUT | GP27 (ADC1) | over the line, 6 mm LEFT of the car's centre line |
| Line IR "Left IR" in the code | AOUT | GP26 (ADC0) | over the line, 6 mm RIGHT of the car's centre line |
| Barcode IR | AOUT / DOUT | GP28 (ADC2) / GP7 | robot's left, 20 mm beside the line |

- **Spacing:** 12 mm centre to centre (10–14 mm also pass in the model). Closer than about 6 mm: both stay black until the car is well off the line (late, hard corrections). Wider than about 16 mm: a sensor sits on an edge all the time (noisy).
- Both sensors at the same height above the floor, side by side (not one in front of the other), as far in front of the wheels as the bracket allows.
- Mounted the other way round (GP26 on the LEFT)? Set `S3_IR_L_SIDE` to `S3_LN_L_ON_LEFT` in `s3_config.h`. Nothing else changes.
- Calibrate with the wheels on the floor: sweep the car sideways so **each** sensor passes fully over white and black, then put both sensors on the line before START.
## 3. Rules the team must keep

1. **ADC:** S3's sampler interrupt (timer alarm 1) is the only ADC user. Nobody else may open the kernel ADC driver.
2. **GPIO interrupt (IO_IRQ_BANK0, IRQ 13):** Subsystem 2 owns it (IF-11). `drv_barcode_init()` attaches S3's handler with `subsystem2_gpio_isr_register()` (Buddy 2's `testing/subsystem2/s3_gpio_isr.patch`, applied) and never calls `tk_def_int(13)`. **Needs Buddy 2's `buddy2-motion` code in the same build** (older `main` has no `subsystem2_gpio_isr_register` and will not link).
3. S3 task priorities follow the Week 6 report: on core 1, `vehicle_ctrl_task` 7 > `task_line` 8 > `motion_task` 9 > `task_barcode` 12. Shorter period gets higher priority (rate-monotonic).

## 4. How other buddies use S3

| Who | Call |
|---|---|
| Anna (Vehicle Controller) | `tk_rcv_mbf(mbf_nav, &m, tmo)`, then use `m.cmd`, `s3_nav_msg_char(&m)`, `m.reversed`, `m.odom_mm`. Wait on `flg_s3` (only the Vehicle Controller does this). Drive `flg_line`: **set `LF_ENABLE` first, then clear `LF_SEARCH_x`.** |
| Arif (Motion) | `s3_get_steer_sp(&dv, &seq)` every control step. FALSE means don't steer from S3. If `seq` stops changing, the value is stale. Call `s3_set_odometry(fn)` once at start-up. |
| Caleb (MQTT) | `s3_get_line_status(&st)` for telemetry. On the demo day, call `s3_set_nav_map("DCBA")` (letters for LEFT, RIGHT, STRAIGHT, U-TURN; any other letter is decoded but ignored, FR5), and use `s3_set_line_gains(kp, kd)` and `s3_request_recalibrate()` for tuning. |
| Kessler (IMU) | nothing |

`s3_get_line_status`, `s3_set_nav_map` and `s3_set_line_gains` use the mutex, so call them from tasks only, never from an ISR. `s3_get_steer_sp` is lock-free and safe anywhere.

## 5. Check it worked

After you flash the main build, the console shows S3 status lines. Sweep both line sensors over white and black, and `UNCALIBRATED` disappears. Use `testing/subsystem3/S3_CHECKLIST.md` for the full test plan.

## 6. Drive test with Buddy 2's motors (before the Vehicle Controller exists)

1. In `s3_config.h` set `#define S3_DRIVE_TEST (1)` (keep it **0** in `main`), then build and flash. A ready image is `S3_v20_DRIVE_TEST.uf2`.
2. Calibrate: slide the car sideways across the line, with both sensors passing fully over white and black, until the console prints `S3 DRIVE CAL: outer white … black …, Centre white … black … -> READY`. Each sensor keeps its own white and black level. Do not lift the car while calibrating (it reads "black" in the air).
3. Put the car on the line, with the Centre on the line's LEFT edge. Only "both sensors white" is refused.
4. **Button 1 (GP20)** does everything:
   - **short press** (let go within 1.5 s): start after a 3 s countdown;
   - **hold 1.5 s, let go before 4 s**: next preset;
   - **hold 4 s**: recalibrate (forget both levels, then sweep again);
   - **any press while driving**: stop.

   Button 2 (GP21) also steps the preset, if you can press it. A start is refused (with the levels printed) until both black levels are at least 2500 raw (`S3_DRIVE_CAL_BLACK_MIN`).

All presets run at 80 mm/s with steer max 80 mm/s, so they differ only in the gains:

| Preset | Kp | Kd | Try it when |
|---|---|---|---|
| **0 (default)** | 250 | 50 | start here |
| 1 | 200 | 50 | it weaves on straights |
| 2 | 300 | 50 | it is late on curves |
| 3 | 250 | 0 | it is still shaky |
| 4 | 250 | 100 | it overshoots after a curve |

"Steer max" is the wheel-speed difference at full lock while tracking. S2 applies half of it to each wheel, so at steer max = speed the inner wheel never goes below half speed. When both sensors are white or the line is LOST, it is 1.25 × steer max, so the inner wheel still rolls forward at 3/8 of the speed.

The car stops by itself in any of these cases, and prints `S3 DRIVE STOP (reason) after X mm, Y ms. S2: stalls=… e-stops=… discarded=…`:

- Button 1;
- 300 mm **or** 2 s of LINE LOST;
- no odometry change for 2 s ("car not moving");
- an **S2 E-STOP**: a wheel at ≥ 40 % duty with no tick for 300 ms latches it, and S2 then discards every command;
- 3 m travelled;
- No elapsed-time run limit (removed at Arif's request); the distance limit remains.

A latched S2 E-STOP is reported and the run is **refused**: power-cycle the car. Buddy 2's reference says S3 must not clear an E-stop to make tests run.

Status (28 Oct): v15b tracks the line and turns left on the car, but loses right and sharp turns at its fixed 150 mm/s. v17 spun at the start. v18 hunted from black to white and one wheel stalled. v19 with the outer sensor at 20 mm tracked 2.1 m but shook (steering changed sign in 27 % of prints). v20 and v21 pass the host tests and simulation, but have **not run on the car yet**. See `testing/subsystem3/S3_ATTEMPTS_LOG.md`.

## 7. Timing prints (`S3_TIMING 1`)

Every 5 s:

    S3 TIME line step max 42 us, period 9998-10003 us, sampler isr max 9 us

After each barcode pass:

    S3 TIME barcode decode 1830 us (both paths)

(The numbers above only show the format; they are not measured values.)

- **line step:** the longest run from wake-up to the end of `s3_line_step`.
- **period:** the shortest and longest time between wake-ups. 10 000 µs is ideal; a long one means `task_line` was delayed.
- **sampler isr:** the longest sampler interrupt.

These values are the evidence for the Week 6 claims ("line_task about 1 % of core 1", INT-05 interrupt load). Copy them into `TEST_LOG.md`. The timer has 1 µs resolution, so a reading of N means N to N+1 µs.
