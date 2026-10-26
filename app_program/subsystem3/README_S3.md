# Subsystem 3 (Buddy 3, Sean): integration guide (v14b)

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
- Centre follows the **right** edge of the line (`S3_EDGE_SIDE`).

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
