# Subsystem 2 (motion control) handover

Written 2026-10-15 by Buddy 2 (Arif). Everything below is what exists today in `app_program/subsystem2/`. Nothing here has run on the car with the final design; it is bench-tested only.

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

| File                | What it does                                                                                                                                                                                                                                                                           |
| ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `drv_encoder.c/.h`  | Edge interrupt on GP16/GP17 (IO_IRQ_BANK0). Counts rising and falling edges per wheel and stamps them with the 1 MHz timer. Also keeps the shortest edge gap and a count of gaps under 100 us.                                                                                         |
| `drv_motor.c/.h`    | `s2_motor_set(motor, duty)` with duty -1000..+1000 permille (positive = forward, 0 = brake), plus `s2_motor_brake()` and `s2_motor_coast()`. `S2_MOTOR_INVERT_L/R` flip a wheel in software.                                                                                           |
| `subsystem2.h/.c`   | Interfaces for the other subsystems: `MOTION_CMD` (MC_STOP, MC_FOLLOW, MC_MOVE, MC_TURN, MC_CLEAR), `mbx_motion`, `flg_motion` with `MF_ESTOP`, `ODOM` with `s2_odom_get/put`, and `s2_steer_set/get`. The objects are created in `subsystem2_init()`. Chassis constants are here too. |
| `subsystem2_test.c` | S2-U01 (encoder ticks per revolution, by hand) and S2-U06 (open-loop motor test). Each has a `*_BENCH` switch at the top; U01 is currently 0 (it adds ~90 s at boot), U06 is 1.                                                                                                        |
| `drive_demo.c`      | Separate demo image: both motors forward for 30 s, then brake (section 5).                                                                                                                                                                                                             |

Conventions: distance in mm, angle in degrees, speed in mm/s. A positive TURN angle and a positive steering correction both mean left. The TURN sign was agreed by me; Sean (S3) and Anna (S5) still need to confirm it, and the steering sign is my assumption.

`motion_task` is NOT written yet. `ODOM` and `steer_sp` exist but nothing updates them, so they read as zero.

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

- **PWM was in reset.** `USE_PTMR` is 0 in `config/config.h`, so the kernel never releases the PWM block and the motors did nothing at first. `s2_motor_init()` now releases it itself, so `config.h` is unchanged. This also settles assumption A5: no other repo code uses PWM slices 4 or 5.
- **No GPIO interrupt code existed in the repo.** `drv_encoder.c` registers the interrupt itself. It runs on the core that calls `s2_enc_init()`. That core prints in the boot line `encoder: GP16/GP17 edge interrupt on core N`, which I have not captured yet. The design report wants it on core 1, so assumption A4 is only partly validated. The shared `gpio_isr` (`app_gpio_irq.c`) shared by the encoder, barcode and ultrasonic is not agreed yet.
- The `Robo Pico` datasheet text I used came from a Gemini extraction and has not been audited.

## 5. Build and test

From WSL, in `build_make`, with `PICO_SDK_PATH` set (see the main README for setup):

| Command            | Image                                                            |
| ------------------ | ---------------------------------------------------------------- |
| `make -j8`         | normal image                                                     |
| `make TEST=1 -j8`  | `mtk3pico_smp1_usb_cdc_wifi_test.uf2`, runs the S2 tests at boot |
| `make DRIVE=1 -j8` | `mtk3pico_smp1_usb_cdc_wifi_drive.uf2`, the drive demo           |

Switching options recompiles everything, which takes several minutes. I added the `DRIVE` option to `build_make/pico_rp2040.mk` (a shared file, off by default).

**Drive demo:** after 8 s it runs both motors forward for 30 s at about 0.3 m/s, holding both wheels at the same encoder rate, then brakes. If a wheel gives no ticks for 1 s it stops. If it curves, change `DEMO_TRIM_R_PERMILLE` in `drive_demo.c` (lower makes the robot curve right, higher curves left). `DEMO_KI` 0 means fixed duty with no feedback. It has not been run on the ground.

**Safety:** if the firmware hangs, PWM keeps its last setting. Keep the wheels off the ground for any new test and be ready to unplug.

## 6. Not done yet

1. Confirm each wheel's direction (FORWARD must drive the robot forward). If a wheel is backward, set `S2_MOTOR_INVERT_L` or `S2_MOTOR_INVERT_R` to 1 in `drv_motor.h`. I have not confirmed this yet.
2. `motion_task`: speed from the first and last edge timestamps in each 20 ms cycle, a per-wheel PID, MOVE and TURN by encoder distance, the `MF_ESTOP` latch with `CLEAR` accepted only after it is cleared, stall detection, and publishing `ODOM`.
3. Tests S2-U02 to S2-U05 from the design report.
4. A distance-roll check to refine ticks per revolution and mm per tick, and a per-wheel calibration.
5. Record the S2-U06 PASS and the S2-U01 results in `TEST_LOG.md`, with the commit and the supply used.
6. Report updates: the PWM-reset finding (A5), the Buddy 2 Section 8.1 row, the power row (power bank, not LiPo), and the GP16/GP17 "TBC" pins.

Also edited in shared files: `TEAM.md` (claims for GP8-GP11, GP16-GP17 and PWM slices 4/5) and `testing/subsystem2/TEST_LOG.md`.
