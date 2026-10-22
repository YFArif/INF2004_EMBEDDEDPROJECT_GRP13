# Subsystem 2 (motion control) handover

Written 2026-10-15 by Buddy 2 (Arif); updated 2026-10-22 when the drive demo was replaced by `motion_task`. Everything below is what exists today in `app_program/subsystem2/`. `motion_task` builds and its software tests are defined, but it has not run on the car yet.

## 1. Hardware and wiring

| Part                    | Pico pin | Robo Pico connection | Notes          |
| ----------------------- | -------- | -------------------- | -------------- |
| Left motor M1, input A  | GP8      | M1A terminal         | PWM slice 4A   |
| Left motor M1, input B  | GP9      | M1B terminal         | PWM slice 4B   |
| Right motor M2, input A | GP10     | M2A terminal         | PWM slice 5A   |
| Right motor M2, input B | GP11     | M2B terminal         | PWM slice 5B   |
| Left encoder A phase    | GP16     | Grove 4 pin 1        | edge interrupt |
| Right encoder A phase   | GP17     | Grove 4 pin 2        | edge interrupt |

Motor 6-pin PH2.0 connector (pin order from the school example; wire colours are an assumption): 01 blue = encoder GND, 02 red = encoder A, 03 green = encoder B (not used), 04 white = encoder VCC (3V3), 05 black = motor +, 06 yellow = motor -.

- PWM is 20 kHz. Driver truth table (from the Robo Pico datasheet extraction, not checked against the original PDF): A high / B low = forward, A low / B high = backward, both low = brake, both high = coast.
- Power: a power bank on the VIN terminal. If USB is also connected, the motors may run from whichever source has the higher voltage, so note which supply you used when you record speeds.
- Chassis: wheel diameter 63 mm, track width 112 mm (centre of left tyre to centre of right tyre). Wheelbase (axle to caster) is not measured yet.
- Unknown: the motors' rated voltage and the encoder output type. A pull-up is enabled on GP16/GP17 in case the output is open-collector.

## 2. Code

All S2 code follows BARR-C:2018 in the same layout as S5's code. The deviations (kernel types at kernel calls, `-std=gnu11`, template file names, the legacy names below) are listed at the top of `subsystem2.h`.

| File                       | What it does                                                                                                                                                                                                                                                                                                                                                                                   |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `subsystem2.h/.c`          | Public interface and kernel objects. Commands (IF-04, message buffer `mbf_motion`, 8 deep, copies the command, never blocks): `subsystem2_move/turn/follow/stop/clear()` or `subsystem2_motion_send()`. `subsystem2_estop_set/clear/is_set()` (IF-05, `MF_ESTOP` in `flg_motion`), `subsystem2_odom_get()` (IF-06: counts, distance, speeds, each wheel's distance, heading), `subsystem2_status_get()` (state, duties, set-points, counters, worst cycle time), `subsystem2_steer_set()` (IF-02). `motion_task` runs on its own pattern-filled stack so its use can be measured (`subsystem2_stack_used()`). Chassis constants. |
| `motion_task.c/.h`         | Priority 9, core 1, woken every 20 ms by `cyc_motion` and at once by `MF_ESTOP`. States IDLE, FOLLOWING, MOVING, TURNING, ESTOP. Per-wheel PID (feed-forward + PI, D available) with a 1.5 m/s^2 set-point ramp and no integration while ramping (no overshoot), edge-timed speed, MOVE/TURN by encoder distance with slow-down, left/right sync and an early stop of `STOP_LEAD_MS` x current speed, stall detection, odometry with heading. Gains (KP 1.0, KI 0.2 per cycle) were tuned in the S2-U09 simulation and are preliminary until S2-U02 on the car. |
| `drv_motor.c/.h`           | `drv_motor_set(motor, duty)` with duty -1000..+1000 permille (positive = forward, 0 = brake), `drv_motor_brake()`, `drv_motor_coast()`. Both inputs of a motor are written in one register write. `DRV_MOTOR_INVERT_LEFT/RIGHT` flip a wheel in software.                                                                                                                                       |
| `drv_encoder.c/.h`         | Edge interrupt on GP16/GP17, set up by `motion_task` so it runs on core 1. Counts debounced ticks (level confirmed, at least 100 us after the last change) plus raw rise/fall counts, rejected glitches and the shortest gap. Counters are never cleared.                                                                                                                                     |
| `subsystem2_test.c`        | Automatic in every `TEST=1` image (motors stay braked): S2-U08 timer rate, S2-U05 period and E-stop latency (20 trials), S2-U07 latch/CLEAR protocol, S2-U10 cycle time and stack (NFR11). The tests wait up to 30 s for the serial port to be opened. Bench switches, all 0: `S2_U01_BENCH` (ticks per revolution by hand), `S2_U06_BENCH` (open-loop motors), `S2_U02_BENCH` (step response 0 -> 250 mm/s, CSV log). |
| `subsystem2_floor.c`       | Floor tests, `S2_FLOOR_TEST` = 0 (off), 1 (S2-U11 MOVE distance), 2 (S2-U03 turns), 3 (S2-U12 braking, slow speeds, stall). Scripted, with a 10 s countdown before each step to measure and reposition. |
| `testing/subsystem2/motion_host_test.c`, `run_motion_test.sh`, `host/` | S2-U09: `motion_task.c` compiled unchanged on the PC against a simulated robot, with sanitizers. Run it after every change to `motion_task.c`. |
| `task_producer.c/.h`       | The course template's example producer, kept only because the S3/S4 examples still use `sem_slots`/`mpf_samples`. Delete it with them.                                                                                                                                                                                                                                                      |

Commands (`subsystem2_cmd_t {type, value, speed_mms}`): `S2_CMD_STOP` brakes and cancels anything running; `S2_CMD_FOLLOW` drives at `speed_mms` (0 = 300) with `v_left = v - steer`, `v_right = v + steer`, and a new FOLLOW only changes the speed; `S2_CMD_MOVE` drives `value` mm (negative = reverse); `S2_CMD_TURN` turns `value` degrees on the spot (positive = left, wheel speed 0 = 150 mm/s); `S2_CMD_CLEAR` leaves the E-stop. MOVE and TURN end with `EV_MOTION_DONE`. A new command replaces the one running (S2-U09 H14), so send the next MOVE/TURN after `EV_MOTION_DONE`; STOP always takes effect at once. A stall (|duty| >= 40 % and no tick for 300 ms) brakes, sets `MF_ESTOP` and reports `S5_FAULT_STALL` + `EV_FAULT`. The S5 calls are compiled in automatically once S5's `subsystem5.h` defines those names.

E-stop: `MF_ESTOP` wakes `motion_task` at once and it brakes, from any state. While latched, every command is discarded except a CLEAR sent after S5 has cleared `MF_ESTOP`; the commands behind that CLEAR run in order.

Legacy names kept for S5's current `obstacle_task.c`: `flg_motion`, `ODOM`, `s2_odom_get()`. Remove them once Anna switches to `subsystem2_estop_set()` and `subsystem2_odom_get()`.

Conventions: distance in mm, angle in degrees, speed in mm/s. A positive TURN angle and a positive steering correction both mean left. Sean (S3) must still confirm the steering sign and scale.

## 3. What was measured

**Encoders (S2-U01).** Two hand-turned runs per wheel. Run 1 was probably 11 turns, not 10, because run 2 / run 1 comes out at 0.909 on both wheels and the left wheel then agrees to 0.05%.

|       | Rising edges per wheel revolution  |
| ----- | ---------------------------------- |
| Left  | about 639                          |
| Right | about 629 (+/- 1%, hand alignment) |

That is about 0.31 mm per tick. Rising and falling counts matched within 1, so no edges are lost. The shortest gap was 11-16 us, which means there is some glitching; real edges are at least about 350 us apart at 0.4 m/s. I plan to count rising edges only with a minimum gap of about 100 us for odometry. The values above are provisional; a distance-roll check will refine them.

**Motors (S2-U06), both PASS**, wheels in the air:

|       | duty 400 forward        | duty 400 reverse | duty 800 forward |
| ----- | ----------------------- | ---------------- | ---------------- |
| Left  | 1038 ticks/s (321 mm/s) | 1037             | 2074 (642 mm/s)  |
| Right | 1062 ticks/s (334 mm/s) | 1059             | 2077 (653 mm/s)  |

Speed doubles with duty and forward matches reverse. The right motor is about 2% faster at the same duty, so each wheel needs its own control loop. Rough no-load gain is 0.8 mm/s per permille of duty; it will be lower under load. The supply for that run was not recorded.

## 4. Bugs and findings

- **PWM was in reset.** `USE_PTMR` is 0 in `config/config.h`, so the kernel never releases the PWM block and the motors did nothing at first. `drv_motor_init()` releases it itself, so `config.h` is unchanged. It now only releases the reset if it is still held and never asserts it, so it cannot wipe S5's servo/echo PWM slices whichever subsystem starts first. No other repo code uses PWM slices 4 or 5 (A5).
- **No GPIO interrupt code existed in the repo.** `drv_encoder.c` registers the IO_IRQ_BANK0 handler itself, from `motion_task` on core 1. The boot line `encoder: GP16/GP17 edge interrupt on core N` should now say core 1; capture it as evidence for A4. The shared `gpio_isr` file for encoders and barcode (S3, GP6) is not agreed yet; S5's echo uses a PWM slice and does not need it.
- **Encoder glitches.** Edge gaps of 11-16 us were seen in S2-U01. The driver now debounces by checking the pin level, and reports how many edges it rejected.
- **`TWF_BITCLR` would clear `MF_ESTOP`.** It clears every bit in the wait pattern, so `motion_task` clears only its own tick bit.
- **The TIMER tick was already 1 MHz.** S2-U08 read WATCHDOG TICK = 0x360c (12 cycles, enabled) at boot. `drv_encoder_init()` checks and enforces it anyway.
- **Found by the S2-U09 simulation before any floor run:** the first PI gains could not hold 250 mm/s under heavy load within 0.5 s; every TURN overshot by about 3 degrees because the wheels roll on while braking; and the integral wound up while the speed ramped, overshooting by 10-15 %. Fixed with retuned gains, the speed-based early stop and no integration while ramping.
- The `Robo Pico` datasheet text I used came from a Gemini extraction and has not been audited.

## 5. Build and test

From WSL, in `build_make`, with `PICO_SDK_PATH` set (see the main README for setup):

| Command           | Image                                                                   |
| ----------------- | ----------------------------------------------------------------------- |
| `make -j8`        | `mtk3pico_smp1_usb_cdc_wifi.uf2`, normal image with `motion_task`       |
| `make TEST=1 -j8` | `mtk3pico_smp1_usb_cdc_wifi_test.uf2`, runs S2-U08, U05, U07, U10 at boot (plus any bench/floor switch set) |
| `sh testing/subsystem2/run_motion_test.sh` (repo root) | S2-U09 host test, no hardware |

Switching options recompiles everything, which takes several minutes. The `DRIVE=1` demo was removed (2026-10-22); `build_make/pico_rp2040.mk` is back to the template.

Without S5's `vehicle_ctrl_task`, nothing sends commands in the normal image, so the motors stay braked. To see the robot move, use the bench tests (`S2_U02_BENCH`, `S2_U06_BENCH`) or send commands from a test.

**Safety:** if the firmware hangs, PWM keeps its last setting. Keep the wheels off the ground for any new test and be ready to unplug.

## 6. Not done yet

1. Flash `TEST=1` and record S2-U08, U05, U07 and U10 with the commit and the log file; run S2-U09 on the PC and record it.
2. Confirm each wheel's direction (S2-U06) and set `DRV_MOTOR_INVERT_LEFT/RIGHT` if needed.
3. S2-U02 on the bench, then on the floor at full and low battery; adjust the feed-forward and PI gains if needed (re-run S2-U09 after any change).
4. S2-U11 (`S2_FLOOR_TEST` 1): correct `S2_TICKS_PER_REV_L/R` from the taped distances.
5. S2-U03 (`S2_FLOOR_TEST` 2): tune `STOP_LEAD_MS`, then the effective `S2_TRACK_MM`, from the measured angles.
6. S2-U12 (`S2_FLOOR_TEST` 3): braking distance (give it to S5), slowest smooth speed, stall.
7. S2-U04 (scope 20 kHz on GP8-GP11) and the hardware part of S2-U05 (GPIO toggle + scope).
8. With S5: switch to the `subsystem2_*` calls, a power-up mode where obstacle monitoring does not latch `MF_ESTOP` in builds where the motors run, INT-02 (stop in front of a box).
9. With S3: the steering sign/scale and who writes the shared `gpio_isr`. With S4: the ODOM fields and the wheelbase (not measured yet). With S1: which status fields go into telemetry.
10. Report updates: the PWM-reset finding (A5), A4 evidence, the Buddy 2 Section 8.1 row, the power row (power bank, not LiPo), the GP16/GP17 "TBC" pins, and `mbx_motion` -> `mbf_motion`.

Also edited in shared files: `TEAM.md` (claims for GP8-GP11, GP16-GP17 and PWM slices 4/5; S2 member row and `motion_task` priority row) and `testing/subsystem2/TEST_LOG.md`.