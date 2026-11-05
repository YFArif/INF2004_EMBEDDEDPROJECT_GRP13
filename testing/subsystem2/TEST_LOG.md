# Subsystem 2 — test log

**Subsystem:** Motion control · **Student ID:** 2501261 · **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

### Anna-style 400 mm MOVE test v37, 2026-11-05

Build from `build_make` with SDK configured:
`make -j8 TEST=1 MOTOR_DIAG=3`.
Named image: `S2_v37_ANNA_MOVE_400MM.uf2`.
Sequence: +400 mm at 40/60/100 mm/s, then -400 mm at 40/60/100 mm/s.
All forward stages precede all reverse stages. Without repositioning,
the three forward moves accumulate to 1200 mm: provide at least 1.4 m
clear forward space, or reposition while stopped between stages.
Each of the six stages requires a separate GP20 press/release, and waits
for S2's incremented done_count with IDLE before continuing. S2 still
raises EV_MOTION_DONE for Anna; this isolated test observes the count
rather than running Anna's full mission task.

Uses `subsystem2_move(distance, speed)` exactly as Anna's adapter does;
not FOLLOW, not fixed PWM, not a timed distance approximation.
The speed is a requested cruise limit, not a promised constant speed:
MOVE ramps and decelerates near the endpoint. Duration therefore varies
by speed. Gains, distance geometry, turn calibration and normal firmware
behaviour are unchanged. S3/S5 tasks remain disabled in this image.

Keep at least 600 mm clear in the intended travel direction, cable slack,
GP20/power accessible. No ultrasonic obstacle protection.
Mark the same axle reference before and after each move; record signed
tape displacement and lateral drift BEFORE GP20 for the next stage.
Reverse does not guarantee returning to exactly the initial position;
repositioning between stopped stages is allowed and each has a new baseline.
Never run near a table edge. GP20 while moving aborts ALL stages (sampled
about every 100-110 ms); use power if immediate physical intervention is needed.
Timeout 40 s, sampling gap >250 ms, E-stop, enqueue failure or invalid
completion causes explicit abort/brake/E-stop and requires reboot.

Each result prints requested distance/speed, elapsed time, final odometry
after a 500 ms settling wait, wheel distances and encoder-derived heading
change. Buffered samples print only after stopping. These readings cannot
replace a tape measure or independently prove physical straightness.

Evidence: [MOVE protocol host checks](logs/2026-11-05_host_anna_move.txt),
[firmware build](logs/2026-11-05_build_anna_move.txt).
Physical floor test: pending. No new calibration claimed.

### Timed floor distance comparison v36, 2026-11-05

Build: from `build_make`, with `PICO_SDK_PATH` set,
`make -j8 TEST=1 MOTOR_DIAG=2`. Image suffix `_test_timed_floor.uf2`;
named copy `S2_v36_TIMED_FLOOR_40_60_100.uf2`.
Mode 1 retains the lifted-wheel v35 sequence; normal builds are unchanged.

| Stage | FOLLOW speed | Nominal command duration | Ideal distance (speed x time) |
| --- | --- | --- | --- |
| 1 | 40 mm/s | 5 s | 200 mm |
| 2 | 60 mm/s | 5 s | 300 mm |
| 3 | 100 mm/s | 5 s | 500 mm |

This is a fixed-time test, NOT a MOVE-to-distance test. It calls the same
public `subsystem2_follow(speed)` and `subsystem2_stop()` APIs used by S5,
with zero steering refreshed during each run. Floor mode never applies
raw PWM to drive/stop a successful run. Emergency failures retain direct
braking plus E-stop. No PI gains, wheel geometry or turn targets change.
S3/S5 motion producers are excluded: this verifies S2's command path,
not the complete group's mission/barcode/obstacle integration.

Clear at least 1 m of flat floor, support a slack USB cable so it does not
pull, and keep GP20/power accessible. **No ultrasonic protection.**
Use a battery/normal motor supply, not an assumption that USB powers motors.
Keep the same floor/load/battery condition for comparisons.
At each READY, mark the drive-wheel AXLE position beside a tape measure,
align straight, then press/release GP20. Run starts on release.
GP20 during motion aborts ALL stages. Reboot after abort/end.
When stopped, mark the same axle reference and record forward displacement
and sideways drift. Record it before starting the next stage; repositioning
by hand is allowed between runs because each stage records a fresh baseline.
Never use a rotating tyre mark as the car's positional reference.

The timer ends the powered command window at the first sample at/after
5 s (about 5.01 s with the observed 30 ms sample cadence).
STOP is queued to the 20 ms motion loop; `TIMED RESULT` reports actual
stop-request time, observed IDLE time, odometry at request and after a
500 ms settling wait, plus separate final wheel distances.
Timestamp/sample gap over 120 ms aborts explicitly.
Timing includes command latency and startup; final tape/encoder distance
includes braking/coast. Therefore 200/300/500 mm are ideal references,
not guaranteed distances or a newly proven accuracy tolerance.
Compare tape with `encoder_final_mm` to check odometry; compare tape with
speed x time to check total motion behaviour. Repeat runs before recalibrating.

Evidence: [dual-mode host tests](logs/2026-11-05_host_timed_floor.txt),
[floor build](logs/2026-11-05_build_timed_floor.txt).
28/28 dual-mode mock checks and 53/53 motion host checks pass; normal
integrated build also passes after restoring user-approved deleted build
dependencies unchanged from HEAD. No calibration or gains were edited.
Physical floor measurements: pending.

### Lifted-wheel motor diagnostic v35, 2026-11-05

Supersedes v34: hardware log showed 20 ms sleeps returning about every
30 ms. The accumulated ideal-schedule check falsely aborted at about
330 ms. v35 uses actual timer elapsed time to end each stage at roughly
2 s, and independently aborts on an individual sampling gap over 120 ms.
Abort output now identifies button, E-stop, sleep, sampling or command
failure explicitly. Zero ticks during the aborted 10% duty test are not
evidence of a failed motor; the higher-duty tests never ran.

[v35 host evidence](logs/2026-11-05_host_motor_diag_v35.txt): 12/12 scenarios
pass, including the observed 30 ms cadence (67 samples, 2.01 s),
100 ms cadence, closed-loop cadence and timer rollover.
[v35 diagnostic build](logs/2026-11-05_build_motor_diag_v35.txt) and
[normal build](logs/2026-11-05_build_normal_v35.txt) pass. Hardware retest pending.

Build in WSL from `build_make`:
`PICO_SDK_PATH=/home/arif/pico/pico-sdk make -j8 TEST=1 MOTOR_DIAG=1`.
Output: `mtk3pico_smp1_usb_cdc_wifi_test_motor_diag.uf2`, also copied as
`S2_v35_LIFTED_MOTOR_DIAG.uf2`. Normal/test/drive images remain separate.
No wheel gains or motor calibration are changed.

**Wheels must be securely lifted, tyres clear of hands/cables/table.
No S3 steering, S5 mission or ultrasonic protection is active.**
Open USB serial and save the complete log. After the motion-task timing
check, every stage waits for a new GP20 press/release. GP20 during motion
aborts the entire sequence and latches E-stop; reboot to restart.
Each stage runs for approximately 2 s, brakes, then prints buffered 20 ms
samples (normally about 30 ms apart on the observed kernel; use CSV
timestamps). No serial input commands are required. No run begins automatically.

Stages 1-10 alternate LEFT/RIGHT at fixed forward PWM:
100, 150, 200, 300, 400 permille (10, 15, 20, 30, 40%).
Each stage starts from rest rather than ramping a running motor.
This brackets the observed starting duty among the tested values; it does
not measure an exact minimum. Watch physical direction and record any
noise, binding or rubbing: single-phase encoders cannot establish direction.
Fixed-PWM target fields remain zero because PI is deliberately IDLE.
Measured fixed-PWM duty fields are the commanded duty, not a sensor reading.

Stages 11-16 run both wheels under the existing PI controller:
10 mm/s twice, 40 twice, 250 twice. Reports include time to first tick,
target/measured wheel speeds, duty and encoder counts. First-tick delay
includes encoder phase, scheduling/queue latency and motor response; it
must not be treated as a precise motor breakaway-time measurement.
Odometry and status are separate snapshots, potentially one cycle apart.
End or abort leaves motors E-stopped until reboot.

Interpretation: repeatable fixed-PWM asymmetry suggests motor/load/wiring,
but encoder feedback must still be checked. Matching fixed-PWM behaviour
with differing PI transients suggests feedback/control sensitivity.
Serial logs alone cannot conclusively isolate a physical fault; a later
motor/driver swap or loaded test may be needed. Do not tune from one run.
Record battery voltage manually if available; this firmware does not measure it.

- [Host diagnostic tests](logs/2026-11-05_host_motor_diag.txt):
  8/8 mock scenarios pass under ASan/UBSan: fixed PWM, closed loop,
  GP20 abort, late task, FOLLOW failure, STOP failure, STOP timeout,
  pre-existing E-stop. Verifies bounded runs and CSV only after braking.
- [Firmware build](logs/2026-11-05_build_motor_diag.txt): build evidence.
- Hardware results: **pending**, not yet flashed/tested by the assistant.

### Low-speed estimator regression, 2026-10-29 (host only)

Evidence: [speed isolation run](logs/2026-10-29_host_speed_isolation.txt).
The unchanged plant model reproduced 10 mm/s underspeed (5.40 / 4.39 mm/s)
with the old 40 ms tick-interval cutoff. Reusing the existing 100 ms stopped
threshold fixes that estimator inconsistency without changing PID gains.

- H20: live FOLLOW sequence 10 / 15 / 10 / 40 / 60 / 40 mm/s; five-second
  displacement-derived wheel means within 15% and exact requested targets.
  At 10 mm/s: left 10.20, right 10.46 mm/s; at 40: approximately 40-41 mm/s.
- H21: 40/50/90/100 ms tick intervals, with and without timer wrap; genuine
  intervals retained below 100 ms and existing stopped fallback at 100 ms.
- Full S2 motion host suite: **53 passed, 0 failed**.
- Physical speed, battery/load sensitivity and edge tracking at 10 mm/s
  still require hardware testing. This is not a new floor calibration.

### U03 recalibration, 2026-10-28 (hardware validation pending)

Evidence: [90-degree run](logs/2026-10-28_s2_u03%28no180%29.txt) and
[clean 180-degree rerun](logs/2026-10-28_s2_u03%28180%20test%29.txt).
Only the first ten measurements from the former are used; its 180-degree
measurements are superseded by the rerun.

| Command | Physical measurements, degrees (magnitudes) | Result before adjustment |
| --- | --- | --- |
| LEFT 90 | 90, 90, 90, 90, 88 | All within 85-95 |
| RIGHT 90 | 82, 87, 85, 84, 85 | 82 and 84 fail 85-95; mean 84.6 |
| LEFT 180 | 185, 185, 184 | All within 172-188 |

Keep the passing left turn scale at 118 mm. Change the right scale from
110 to 117 mm: `110 * 90 / 84.6 = 117.02`. Under a proportional-response
assumption, the previous right readings would become 87.2, 92.5, 90.4,
89.3 and 90.4 degrees. These are predictions, not measured passes.
Physical track, wheel circumference, encoder resolution and PID are unchanged.
This changes finite TURN commands and spin heading estimation, not FOLLOW
steering. Sean's sensor-based steering still uses `S2_S3_STEER_PERCENT = 50`;
this adjustment does not establish a fix for right curves while following a line.

NFR5 requires 90-degree turns within +/-5 degrees and 180-degree turns
within +/-8 degrees over 10 trials each. The current five trials per
90-degree direction and three U-turns are calibration evidence, not full
acceptance evidence. Revalidate on the floor with the new firmware and
a slack cable; do not infer physical accuracy from encoder-only PASS lines.

Define a test here before running it. Test IDs never change.

| ID     | What is tested                                                                                    | Requirement / interface | Method        | Steps / how                                                                                                                                                                                                               | Expected result                                                                                                                                                                                      |
| ------ | ------------------------------------------------------------------------------------------------- | ----------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| S2-U01 | Encoder ticks per wheel revolution, both wheels                                                   | FR7                     | manual + auto | Set `S2_U01_BENCH` to 1 in `subsystem2_test.c`. `make TEST=1`, flash, open the serial port. Follow the prompts: turn ONE wheel by hand exactly 10 revolutions inside the 40 s window (left first, then right), the other wheel still. Repeat the whole run slow and fast. | `[TEST] S2-U01 PASS` for each wheel; the `INFO` line gives debounced ticks per revolution and the rejected glitch count; repeat runs agree within 1%                                                  |
| S2-U02 | PID step response, FOLLOW 0 -> 250 mm/s with no steering                                          | FR7, NFR4               | auto          | Bench: set `S2_U02_BENCH` to 1, lift the wheels, `make TEST=1`, flash. The CSV lines `[S2-U02] t_ms,speed_l,speed_r,duty_l,duty_r` are the step response (paste into a spreadsheet). Floor version later, at full and low battery. | `[TEST] S2-U02 PASS`: both wheels within +/-25 mm/s (10 %) of 250 mm/s from 0.5 s to 1.5 s                                                                                                          |
| S2-U05 | `MF_ESTOP` -> brake latency                                                                        | NFR6, FR15, IF-05       | auto          | Software part, runs in every `make TEST=1` image with the motors braked: 20 trials of `subsystem2_estop_set()`, latency = timer at the brake minus timer at the set. Hardware part (GPIO toggle + scope on GP8-GP11) still to do. | `[TEST] S2-U05 PASS` twice: 48-52 motion cycles per second, and worst latency <= 20000 us (one motion cycle); the `INFO` lines give the cycle count, the longest cycle (NFR11), and worst/mean latency                                                                                       |
| S2-U06 | Open-loop motor drive: each motor turns its own wheel, forward and reverse, faster at higher duty | FR7                     | manual + auto | Set `S2_U06_BENCH` to 1. Lift the wheels. `make TEST=1`, flash, watch the serial output and the wheels: each motor runs forward, reverse, then forward faster, one at a time. Note whether FORWARD really drives the robot forward. | `[TEST] S2-U06 PASS` for each motor; ticks/s rise with duty; the other wheel stays still; FORWARD matches the robot's forward direction (otherwise set `DRV_MOTOR_INVERT_LEFT/RIGHT` in `drv_motor.h` or swap that motor's wires) |
| S2-U07 | E-stop latch and CLEAR protocol                                                                   | IF-04, IF-05, FR15      | auto          | Runs in every `make TEST=1` image, motors stay braked: set `MF_ESTOP`; send MOVE 100 (must be discarded); send CLEAR while still latched (must be ignored); clear `MF_ESTOP`, send CLEAR then MOVE 0 (both must run). | `[TEST] S2-U07 PASS`; `INFO 4 of 4 steps passed`                                                                                                                                                    |
| S2-U08 | The RP2040 TIMER counts at 1 MHz (all edge times, speeds, the encoder glitch filter and S2-U05 depend on it) | NFR4, NFR6, A4          | auto          | Runs in every `make TEST=1` image: count the timer across a 1000 ms kernel delay. The `INFO` line also shows the WATCHDOG TICK register at boot; `drv_encoder_init()` enforces 12 cycles of the 12 MHz crystal if it is not already set. | `[TEST] S2-U08 PASS`: 970000 ... 1040000 counts                                                                                                                                                      |
| S2-U13 | Steering: a set value is used, then ignored after 100 ms (a stopped line_task must not leave the robot steering) | IF-02, FR1, FR15 | auto          | Runs in every `make TEST=1` image, motors stay braked: `subsystem2_steer_set(40)` must read back 40 at once and not 40 after 150 ms. | `[TEST] S2-U13 PASS`; `INFO fresh value used 1, old value ignored 1` |
| S2-U03 | Turn accuracy on the floor, 90 and 180 degrees                                                    | FR6, NFR5               | manual + auto | `s2_test.ps1 -Test u03` (see `TEST_GUIDE.md` for the tape cross and the pointer). TURN +90 x5, -90 x5, 180 x3, 30 s apart; after each turn the script asks for the protractor reading and saves it as a `[MEASURED]` line. | Auto: `S2-U03 PASS` per turn (own odometry within 2 deg). Manual (the real result): every 90 within +/-5 deg, every 180 within +/-8 deg. Tune `STOP_LEAD_MS`, then the effective `S2_TRACK_MM`, if not |
| S2-U09 | `motion_task.c` against a simulated robot on the PC: speed control at 3 loads and 3 motor lags, MOVE, TURN, STOP, E-stop latch and CLEAR, stall, steering, speed change, command handling, odometry, timer wrap, 60 s run | FR6, FR7, FR15, NFR4, NFR5, NFR6, IF-02, IF-04, IF-05, IF-06 | auto (host) | In WSL, from the repository root: `sh testing/subsystem2/run_motion_test.sh \| tee testing/subsystem2/logs/YYYY-MM-DD_host_test_motion.txt`. Built with `-Werror` and the address and undefined-behaviour sanitizers. | Last line `S2-U09: PASS`. Proves the logic and arithmetic; the gains are proved on the car (S2-U02, S2-U03, S2-U11, S2-U12) |
| S2-U10 | Longest motion cycle and stack use                                                                | NFR11                   | auto          | Runs last in every `make TEST=1` image, so it covers everything run before it (enable the bench or floor tests to include driving). `motion_task` runs on its own pattern-filled 2176-byte stack. | `[TEST] S2-U10 PASS`: longest cycle <= 4000 us (20 % of 20 ms), stack <= 75 %                                                                                                                         |
| S2-U11 | MOVE distance and straightness on the floor                                                       | FR6, IF-06              | manual + auto | `s2_test.ps1 -Test u11` (see `TEST_GUIDE.md`), ~1.6 m x 0.6 m clear. Tape measure on the floor beside the LEFT wheel; reference point = front edge of the left tyre where it touches the floor (any start reading; reverse run starts at 300 mm or more). MOVE 1000 x3 at 300 mm/s, MOVE 1000 at 150 mm/s, MOVE -300. The script asks the start reading and tyre-to-tape gap before each run and the end reading and gap after it, and saves the real distance and sideways drift as `[MEASURED]` lines. | Auto: own odometry within 2 %. Manual: real distance within +/-2 % of the odometry (otherwise correct `S2_TICKS_PER_REV_L/R` by the ratio) and sideways drift under 30 mm per metre |
| S2-U12 | Braking distance, slow speeds, stall detection on the floor                                       | NFR6, FR15, INT-02      | manual + auto | `s2_test.ps1 -Test u12` (see `TEST_GUIDE.md`), ~1 m clear. FOLLOW 300 then STOP, FOLLOW 300 then E-stop, FOLLOW 150 then STOP; FOLLOW 40/60/100/150 for 2 s each; FOLLOW 150, then lift the robot and pinch the LEFT tyre when told. The script asks whether it skidded and whether each slow speed was smooth. | Braking distance from 300 mm/s <= 58 mm (NFR6 budget: 150 mm threshold - 42 mm detection - 50 mm margin); 150 mm/s held within 10 %; the held wheel gives a STALL, brake and `MF_ESTOP`. The slow-speed answers give the slowest smooth speed |

_Method:_ **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date       | Test ID | Commit    | Tester | Result      | Log / evidence          | Notes |
| ---------- | ------- | --------- | ------ | ----------- | ----------------------- | ----- |
| YYYY-MM-DD | S2-U01  | `abc1234` |        | PASS / FAIL | `logs/YYYY-MM-DD_….txt` |       |
| 2026-10-29 | S2-U09 | `1e547e1` + uncommitted v29 tests | Copilot for Arif | PASS (39 checks) | `logs/2026-10-29_host_probe_turns.txt` | Existing motor model plus +4/-8/+16/-24/+12 and +90 degree TURN at 60 mm/s wheel speed, for the GP27 experiment. Host model only; physical low-speed turn accuracy and search recovery still unverified. |
| 2026-10-22 | S2-U09  | `0d5f601` | Arif (run by Copilot in WSL) | PASS | `logs/2026-10-22_host_test_motion.txt` | 32 of 32 checks; simulated robot, gcc 13.3 with ASan/UBSan. Earlier runs during development found the weak PI, the ramp wind-up and the 3 deg TURN overshoot, fixed in `081d9d7` |
| 2026-10-22 | S2-U08  | `de44e54` | Arif | PASS | `logs/2026-10-22_s2_unit.txt` | 1009672 counts in 1000 ms. Power bank on VIN + USB; VIN not measured |
| 2026-10-22 | S2-U05  | `de44e54` | Arif | PASS | `logs/2026-10-22_s2_unit.txt` | 50 cycles/s; E-stop latency worst 176 us, mean 135 us (limit 20000) |
| 2026-10-22 | S2-U07  | `de44e54` | Arif | PASS | `logs/2026-10-22_s2_unit.txt` | 4 of 4 steps |
| 2026-10-22 | S2-U10  | `de44e54` | Arif | PASS | `logs/2026-10-22_s2_unit.txt` | longest cycle 304 us (limit 4000); stack 432 of 2176 B = 19 % (limit 75 %) |
| 2026-10-22 | S2-U06  | `de44e54` | Arif | FAIL | `logs/2026-10-22_s2_u06.txt` | Encoder checks passed (L 1035 / 2042 ticks/s, R 1061 / 2067 at duty 400 / 800, other wheel 0), but the RIGHT wheel turned BACKWARD for a positive duty (seen by eye; the encoders cannot see direction). Fixed by `DRV_MOTOR_INVERT_RIGHT` = 1; re-run needed |
| 2026-10-22 | S2-U02  | `de44e54` | Arif | FAIL | `logs/2026-10-22_s2_u02.txt`, `logs/2026-10-22_s2_u02_2.txt` | Speed control met NFR4 in both runs (within 10 % by 160-200 ms, steady +/-2 mm/s, no overshoot), but the right wheel ran backward (same cause as S2-U06), so the test is not valid. Re-run after the fix |
| 2026-10-22 | S2-U06  | `c6a2824` | Arif | PASS | `logs/2026-10-22_s2_u06_4.txt` | After the right-motor fix: all six direction answers `y` (forward, reverse, fast, each wheel); L 1038 / 2044, R 1043 / 2075 ticks/s at duty 400 / 800; other wheel 0. Wheels lifted; VIN not measured |
| 2026-10-22 | S2-U02  | `c6a2824` | Arif | PASS | `logs/2026-10-22_s2_u02_3.txt` | Both wheels forward (`y`). Left within 10 % of 250 mm/s at 180 ms, no overshoot. Right started ~60 ms later, overshot to 314 mm/s (+26 %) at 300 ms and was inside the band from 420 ms (limit 500 ms); steady 249-251 mm/s on both from 1 s. Watch the right-wheel start in the floor run before tuning. Wheels lifted; VIN not measured |
| 2026-10-23 | S2-U11  | `b9e2dfc` | Arif | FAIL | `logs/2026-10-23_s2_u11_3.txt` | Odometry 999-1000 mm and heading < 1 deg in every run, but the tape gave 1047 / 1050 / 1055 mm (MOVE 1000) and 316.5 mm (MOVE -300, start 0, start reading not typed), and the robot curved right 38 / 62 / 85 mm (reverse 6 mm). Cause: wheel calibration (left 5.4 %, right 3.9 % further than counted; 1.4 % mismatch makes it curve). Fixed in the next commit: per-wheel `S2_WHEEL_CIRC_UM_L/R` = 208620 / 209010 um with 639 ticks each. Tyre against the tape (gap 0). Re-run needed |
| 2026-10-23 | S2-U11  | `949f3a4` | Arif | PASS | `logs/2026-10-23_s2_u11_4.txt` | After the per-wheel calibration: tape 1002 / 1002 / 999 / 1002 mm for MOVE 1000 (within 0.2 %) and 300 mm for MOVE -300; sideways 0 mm (runs 1-2), 10 mm (reverse); runs 3-4 had no gap reading. Left tyre against the tape, start 0. The calibration printout at the end of this log is WRONG (-66 %): a sign bug in `s2_test.ps1` for the reverse run, fixed in the next commit; recomputed from this log it is left +0.19 %, right +0.20 %, so no change is needed. VIN not measured |
| 2026-10-23 | S2-U03  | `fdae9c3` | Arif | FAIL | `logs/2026-10-23_s2_u03_2.txt`, `logs/2026-10-23_s2_u03_4.txt` | Incomplete (8 of 13 turns). Odometry 88.3-89.9 deg every time, but measured: left 90 / 88 / 84 / 80 / 86 (avg 85.6, NFR5 limit 85-95: run 4 fails), right 95 / 90 / 90 (avg 91.7; typed without the minus sign). In u03_2 run 2 a TURN never finished (15 s timeout, L -81 / R 92 mm): ticks were counted backward while a wheel was braked with reverse duty. Fixed in the next commit (tick direction changes only when a wheel has stopped) and per-direction turn calibration S2_TURN_TRACK_LEFT/RIGHT_MM = 118 / 110 mm. Re-run needed |
