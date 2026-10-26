# Subsystem 3 (Buddy 3, Sean): integration guide (v15)

S3 covers IR line following, junction and line-lost detection, Code 39 barcode decoding, and the navigation command.

## 1. What to copy into the team repo

Copy the whole folder `app_program/subsystem3/` and the folder `testing/subsystem3/host/`. Nothing outside these two folders needs to change. The build compiles every `app_program/*/*.c` (`build_make/mtkernel_3/app_program/subdir.mk`), and `app_main.c` already calls `subsystem3_init()` and `subsystem3_start()`.

| File | Needed for | Notes |
|---|---|---|
| `s3_api.h` | **everyone** | The public interface. Other subsystems include `../subsystem3/subsystem3.h`, which includes this header. |
| `subsystem3.c` / `.h` | build | `subsystem3_init()` calls `s3_init()`, and `subsystem3_start()` calls `s3_bringup_start()`. |
| `s3_bringup.c` | build | ISRs, line task, barcode task, getters. **`#define S3_TEST` must be 0** for the robot. |
| `s3_line.c` / `.h` | build | calibration, PD steering, junction, lost, search |
| `s3_code39.c` / `.h` | build | Code 39 decoder: all 43 characters plus `*`, forwards and backwards |
| `s3_analog.c` / `.h` | build | analogue barcode path |
| `s3_nav.c` / `.h` | build | letter → command, with the map settable at run time |
| `task_consumer.c`, `subsystem3_test.c` | build | Template demo objects (`mbx_samples`, `mtx_stats`, `app_stats`). Subsystems 2 and 4 still include them, so **do not delete** these until the template demo is removed. |
| `S3_CHECKLIST.md`, `S3_DATA_REPRESENTATION.md`, this file | docs | not compiled |
| `testing/subsystem3/host/*.c` | laptop tests | `gcc`, no Pico needed |

## 2. Wiring (robot, S3_TEST 0)

- Barcode sensor: DOUT → GP6, AOUT → GP26 (ADC0).
- Centre line sensor: AOUT → GP27 (ADC1).
- Outer line sensor (on the robot's **right**): AOUT → GP28 (ADC2).
- Centre follows the **right** edge of the line (`S3_EDGE_SIDE`). This must be the side the outer sensor is on: the overshoot and junction rules assume it.
- Outer-to-Centre spacing (lens middle to lens middle): **20-21 mm**. More than 18 mm (the line) or a drift reads as a junction; much more than 21 mm leaves a blind gap on right curves.

## 3. Rules the team must keep

1. **ADC:** S3's sampler interrupt (timer alarm 1) is the only ADC user. Nobody else may open the kernel ADC driver.
2. **GPIO interrupt (IO_IRQ_BANK0, IRQ 13):** Subsystem 2 owns it (IF-11). `s3_irq_init()` attaches S3's handler with `subsystem2_gpio_isr_register()` (Buddy 2's `testing/subsystem2/s3_gpio_isr.patch`, applied) and never calls `tk_def_int(13)`. **Needs Buddy 2's `buddy2-motion` code in the same build** (older `main` has no `subsystem2_gpio_isr_register` and will not link).
3. S3 tasks use priorities 16–17.

## 4. How other buddies use S3

| Who | Call |
|---|---|
| Anna (Vehicle Controller) | `tk_rcv_mbf(mbf_nav, &m, tmo)`, then use `m.cmd`, `s3_nav_msg_char(&m)`, `m.reversed`, `m.odom_mm`. Wait on `flg_s3` (only the Vehicle Controller does this). Drive `flg_line`: **set `LF_ENABLE` first, then clear `LF_SEARCH_x`.** |
| Arif (Motion) | `s3_get_steer_sp(&dv, &seq)` every control step. FALSE means don't steer from S3. If `seq` stops changing, the value is stale. Call `s3_set_odometry(fn)` once at start-up. |
| Caleb (MQTT) | `s3_get_line_status(&st)` for telemetry. On the demo day, call `s3_set_nav_map("DCBA")`, and use `s3_set_line_gains(kp, kd)` and `s3_request_recalibrate()` for tuning. |
| Kessler (IMU) | nothing |

`s3_get_line_status`, `s3_set_nav_map` and `s3_set_line_gains` use the mutex, so call them from tasks only, never from an ISR. `s3_get_steer_sp` is lock-free and safe anywhere.

## 5. Check it worked

After you flash `S3_RUN_robot.uf2`, the console shows S3 status lines. Sweep both line sensors over white and black, and `UNCALIBRATED` disappears. Use `S3_CHECKLIST.md` for the full test plan.

## 6. Drive test with Buddy 2's motors (before the Vehicle Controller exists)

1. In `s3_bringup.c` set `#define S3_DRIVE_TEST 1` (keep it **0** in `main`), build, flash.
2. Calibrate: slide the car sideways across the line until `UNCALIBRATED` disappears.
3. Put the car on the line (Centre on the line or its right edge). Only "both sensors white" is refused.
4. **Button 1 (GP20)** starts / stops. **Button 2 (GP21)** steps the preset while waiting:

| Preset | Speed | Kp | Kd |
|---|---|---|---|
| 0 | 150 mm/s | 100 | 300 |
| 1 | 120 mm/s | 100 | 300 |
| 2 (default) | 100 mm/s | 100 | 300 |
| 3 | 100 mm/s | 150 | 400 |
| 4 | 80 mm/s | 150 | 400 |

The car stops on Button 1, after 300 mm of LINE LOST, after 3 m, or after 30 s, and prints `S3 DRIVE STOP (reason) after X mm`. Both buttons are only read (GP20 is S5's start button).

Status (26 Oct): steering direction verified on the car; tracking not yet stable (at 150 mm/s every run weaved off within about 0.5 m). Tuning continues with the presets above.
