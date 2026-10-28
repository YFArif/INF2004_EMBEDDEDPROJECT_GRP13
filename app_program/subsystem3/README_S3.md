# Subsystem 3 (Buddy 3, Sean): integration guide (v20)

S3 covers IR line following, junction and line-lost detection, Code 39 barcode decoding, and the navigation command.

## 1. What to copy into the team repo

Copy the whole folder `app_program/subsystem3/` and the folder `testing/subsystem3/host/`. Nothing outside these two folders needs to change. The build compiles every `app_program/*/*.c` (`build_make/mtkernel_3/app_program/subdir.mk`), and `app_main.c` already calls `subsystem3_init()` and `subsystem3_start()`.

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
| `s3_api.h` / `s3_api.c` | interface | Public interface (other subsystems include `../subsystem3/subsystem3.h`), kernel objects, status, steering word. |
| `s3_config.h` | settings | Priorities, periods, edge side, debug prints, `S3_DRIVE_TEST`. |
| `drv_ir_left.c` / `.h` | driver | Left IR line sensor, GP28 / ADC2 (mounted on the robot's right). |
| `drv_ir_centre.c` / `.h` | driver | Centre line sensor, GP27 / ADC1. |
| `drv_barcode.c` / `.h` | driver | Barcode sensor: DOUT GP6 edge interrupt (on S2's shared handler), AOUT GP26 sample ring, pass detection. |
| `drv_adc.c` / `.h` | driver | ADC owner and the 2 kHz sampler interrupt that feeds the three drivers. |
| `s3_ir_cal.c` / `.h` | logic (pure C) | Calibration of one line sensor: filter, learn, relax, normalise. |
| `s3_line.c` / `.h` | logic (pure C) | Combiner: PD steering, curve boost, overshoot, junction, lost, search. |
| `s3_code39.c` / `.h` | logic (pure C) | Code 39 decoder: all 43 characters plus `*`, forwards and backwards. |
| `s3_analog.c` / `.h` | logic (pure C) | Analogue barcode widths. |
| `s3_nav.c` / `.h` | logic (pure C) | Letter → command, map settable at run time. |
| `task_line.c` / `.h` | task | 10 ms line task (core 1, priority 8). |
| `task_barcode.c` / `.h` | task | Hardware set-up, then barcode decode, fusion and publish (core 1, priority 12). |
| `task_drive_test.c` / `.h` | test | Only with `S3_DRIVE_TEST 1`. |
| `s3_private.h` | internal | Helpers shared inside S3 only. |
| `subsystem3.c` / `.h` | build | `subsystem3_init()` → `s3_init()`, `subsystem3_start()` → `s3_start()`. |
| `task_consumer.c` / `.h`, `subsystem3_test.c` | template | Template demo objects (`mbx_samples`, `mtx_stats`, `app_stats`). Subsystems 2 and 4 still use them, so **do not delete** these until the template demo is removed. |
| `S3_CHECKLIST.md`, `S3_DATA_REPRESENTATION.md`, this file | docs | not compiled |
| `testing/subsystem3/host/*.c` | laptop tests | `gcc`, no Pico needed. Pure-C files only. |

## 2. Wiring and sensor layout (robot, 28 Oct)

| Sensor | Signal | Pin | Where on the car |
|---|---|---|---|
| Centre line IR | AOUT | GP27 (ADC1) | middle of the sensor bar, slightly left of the car's centre |
| Outer line IR ("Left IR" in the code) | AOUT | GP26 (ADC0) | on the bar, to the **right** of the Centre |
| Barcode IR | AOUT / DOUT | GP28 (ADC2) / GP7 | robot's left, 20 mm beside the line |

Seen from above (car driving up the page):

    C |##########| O        C = Centre on the line's LEFT edge (S3_EDGE_SIDE = LEFT)
                            O = outer sensor just OUTSIDE the RIGHT edge

- **Spacing, Centre to outer:** 20–24 mm (line width 18 mm plus 2–6 mm). The outer sensor then turns black after only a few mm of drift to the left. On the car (28 Oct) about 33 mm tracked badly (Centre on the edge 14–21 % of prints) and **20 mm** (centre to centre) worked best; keep 20 mm.
- Both sensors at the same height above the floor.
- If the outer sensor is ever moved to the robot's **left**, set `S3_EDGE_SIDE` to `S3_LN_EDGE_RIGHT` in `s3_config.h`. Nothing else changes.

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

After you flash the main build, the console shows S3 status lines. Sweep both line sensors over white and black, and `UNCALIBRATED` disappears. Use `S3_CHECKLIST.md` for the full test plan.

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
- 30 s elapsed.

A latched S2 E-STOP is reported and the run is **refused**: power-cycle the car. Buddy 2's reference says S3 must not clear an E-stop to make tests run.

Status (28 Oct): v15b tracks the line and turns left on the car, but loses right and sharp turns at its fixed 150 mm/s. v17 spun at the start. v18 hunted from black to white and one wheel stalled. v19 with the outer sensor at 20 mm tracked 2.1 m but shook (steering changed sign in 27 % of prints). v20 passes the host tests and simulation, but has **not run on the car yet**. See `testing/subsystem3/S3_ATTEMPTS_LOG.md`.

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
