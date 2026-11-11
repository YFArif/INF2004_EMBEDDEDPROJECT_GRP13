# Subsystem 5 — test log

**Subsystem:** Obstacle scan, avoidance, vehicle controller  ·  **Student ID:** 2502447  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

**How to run:** `powershell -ExecutionPolicy Bypass -File testing\subsystem5\s5_test.ps1 -Test <auto|bench|stop|avoid>`.
The script sets `S5_TEST_MODE` in `subsystem5_test.c` for one build, flashes,
logs to `logs/<date>_s5_<test>.txt` and asks for the manual measurements.
`testing/subsystem5/TEST_GUIDE.md` gives the set-up of each test.
`[S5] PING servo=… range=…` lines show every ping during scans and every 5th
while monitoring (`LOG_PINGS` in `obstacle_task.c`).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | HC-SR04 range accuracy and steadiness | FR9, NFR7 | auto + manual | `-Test bench`: 20 samples, 100 ms apart, of the live range, box ~30 cm ahead. Compare the printed `S5-U01 mean_mm` with a ruler (asked by the script). Later also at 10 and 50 cm. | `[TEST] S5-U01 PASS`; mean within ±10 mm of the ruler |
| S5-U02 | ECHO voltage at GP3 is safe | A8 | manual | 1. Power the sensor with GP3 disconnected. 2. Put a target 20 cm away and ping it. 3. Measure the ECHO pin (peak). | ≤ 3.3 V (HC-SR04+ on 3.3 V); only then connect GP3 |
| S5-U03 | Servo angles | A9, FR10 | auto + manual | `-Test bench`: `OB_CAL_SWEEP` holds 50, 70, 90, 110, 130° (the sensor's reach; a box on the robot stops it about 40° either side) for 2 s each and logs `CAL angle_deg range_mm` at each. Film it; the script asks for the direction and the worst error. | `[TEST] S5-U03 PASS`; each angle within ±3°; otherwise adjust `PULSE_*_US` / `TRIM_DEG` in `drv_servo.c` |
| S5-U04 | Live scan publishes a profile | FR10, NFR13, A16 | auto + manual | `-Test bench`: `OB_SCAN_REQ`. The script asks for the box's real width to compare with the printed `PROFILE ... width_mm`. Later: 3 boxes at 15/20/30 cm, 10 runs each; fit `S5_EDGE_CORR_DEG`. | `[TEST] S5-U04 PASS`; width error ≤ 10 % |
| S5-U05 | Full bypass on the floor | FR11, FR16, NFR10, NFR13 | auto + manual | `-Test avoid`, 3 runs: robot 1 m before a box on a taped line; START; it stops, scans, bypasses, returns, drives on 1.5 s, stops. The script asks: touched?, offset from the tape, heading, closest distance. | `[TEST] S5-U05 PASS` x3; never touches; back within ~50 mm of the tape |
| S5-U06 | Integer maths, scan logic and profile on a fixed simulated scene | FR10 | auto | Every test image: sin/cos/isqrt checks, then a full scan and profile of a 150 mm box 150 mm ahead with a wall 310 mm to its left. No hardware needed. | `[TEST] S5-U06a/b/c PASS`; width 135–165 mm, left clearance 150–250 mm |
| S5-U07 | obstacle_task obeys OB_RESUME and OB_HOLD | IF-09, flg_obst | auto | Every test image: each request, then the published state 300 ms later. | `[TEST] S5-U07a/b PASS` |
| S5-U08 | Drive at a box: slow, stop, scan, plan (no bypass) | FR9, NFR6, NFR7, FR16 | auto + manual | `-Test stop`, 3 runs: robot 1 m from a box; START; bypass switched off. The script asks: gap at stop, slowed?, touched?, plan on the roomier side? | `[TEST] S5-U08 PASS` x3; gap ≥ 50 mm; never touches |
| S5-U09 | Avoidance planner decisions | FR11 | auto | Every test image: fixed profiles give RIGHT (wall left), LEFT (bigger gap), REVERSE (both narrow), CONTINUE (off path); with the face 150 mm ahead, a 197 mm shift turns out 33–37° on a 330–355 mm leg (rear wheels 160 mm behind the front), returns the same way, no reverse first. | `[TEST] S5-U09 PASS` |
| S5-U10 | vehicle_ctrl_task reaches READY after power-up | Section 5 statechart | auto | Every test image: controller state after the boot wait. | `[TEST] S5-U10 PASS` |
| S5-U11 | Fault flag stress test: no bit lost across the two cores | Tribunal 1 (minimum-bit), flg_fault | auto | Every test image: one task on each core sets and clears its own bit 10,000 times, first in a plain shared word with a slow read-modify-write (deliberately broken), then in an event flag made like `flg_fault` (`tk_set_flg` / `tk_clr_flg`). Prints `S5-U11 lost bits: plain_word event_flag`. | `[TEST] S5-U11a PASS` (the plain word loses bits, so the test can see a race) and `S5-U11b PASS` (the event flag loses none) |
| S5-U12 | Dead sensor stops the robot | Tribunal 1 Version C, NFR6 | manual | Robot following the line; unplug the HC-SR04's ECHO wire. | Motors stop within 3 pings (about 180 ms); state FAULT; `[S5] EVT 0x58` (sensor fault) in the log |
| S5-U13 | S5 RAM before and after minimum-bit packing | Tribunal 1 (Week 13 evidence) | manual | After `make`: `python testing/subsystem5/s5_ram.py build_make/mtk3pico_smp1_usb_cdc_wifi.map`. Compare with the "before" row below. | Static RAM lower than before; packed structs: profile 26 → 12 B, obstacle status 12 → 4 B |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S5-U01, U03, U04 | `c7e29ef` | Anna | not run | `logs/2026-10-22_unit.txt` | Output lost while S2-U06 drove the motors on USB power; mostly "no echo" (sensor tilted, since fixed). Rerun with `-Test bench`. |
| 2026-11-11 | S5-U13 (before) | `15300f7` | map file | baseline | `build_make/mtk3pico_smp1_usb_cdc_wifi.map` (build of 18:09, no source newer) | Before minimum-bit packing: S5 static RAM 410 B (obstacle_task 192, s5_mission 77, subsystem5 61, s5_avoid 45, drv_button 16, s5_ranger 14, drv_servo 4, s5_links 1), flash 15,498 B. Task stacks not included (kernel-allocated). |
