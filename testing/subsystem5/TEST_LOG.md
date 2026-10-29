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
| S5-U03 | Servo angles | A9, FR10 | auto + manual | `-Test bench`: `OB_CAL_SWEEP` holds 30, 60, 90, 120, 150° for 2 s each. Film it; the script asks for the direction and the worst error. | `[TEST] S5-U03 PASS`; each angle within ±3°; otherwise adjust `PULSE_*_US` / `TRIM_DEG` in `drv_servo.c` |
| S5-U04 | Live scan publishes a profile | FR10, NFR13, A16 | auto + manual | `-Test bench`: `OB_SCAN_REQ`. The script asks for the box's real width to compare with the printed `PROFILE ... width_mm`. Later: 3 boxes at 15/20/30 cm, 10 runs each; fit `S5_EDGE_CORR_DEG`. | `[TEST] S5-U04 PASS`; width error ≤ 10 % |
| S5-U05 | Full bypass on the floor | FR11, FR16, NFR10, NFR13 | auto + manual | `-Test avoid`, 3 runs: robot 1 m before a box on a taped line; START; it stops, scans, bypasses, returns, drives on 1.5 s, stops. The script asks: touched?, offset from the tape, heading, closest distance. | `[TEST] S5-U05 PASS` x3; never touches; back within ~50 mm of the tape |
| S5-U06 | Integer maths, scan logic and profile on a fixed simulated scene | FR10 | auto | Every test image: sin/cos/isqrt checks, then a full scan and profile of a 150 mm box 150 mm ahead with a wall 310 mm to its left. No hardware needed. | `[TEST] S5-U06a/b/c PASS`; width 135–165 mm, left clearance 150–250 mm |
| S5-U07 | obstacle_task obeys OB_RESUME and OB_HOLD | IF-09, flg_obst | auto | Every test image: each request, then the published state 300 ms later. | `[TEST] S5-U07a/b PASS` |
| S5-U08 | Drive at a box: slow, stop, scan, plan (no bypass) | FR9, NFR6, NFR7, FR16 | auto + manual | `-Test stop`, 3 runs: robot 1 m from a box; START; bypass switched off. The script asks: gap at stop, slowed?, touched?, plan on the roomier side? | `[TEST] S5-U08 PASS` x3; gap ≥ 50 mm; never touches |
| S5-U09 | Avoidance planner decisions | FR11 | auto | Every test image: fixed profiles give RIGHT (wall left), LEFT (bigger gap), REVERSE (both narrow), CONTINUE (off path); 45° leg for a 197 mm shift is 270–290 mm. | `[TEST] S5-U09 PASS` |
| S5-U10 | vehicle_ctrl_task reaches READY after power-up | Section 5 statechart | auto | Every test image: controller state after the boot wait. | `[TEST] S5-U10 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-22 | S5-U01, U03, U04 | `c7e29ef` | Anna | not run | `logs/2026-10-22_unit.txt` | Output lost while S2-U06 drove the motors on USB power; mostly "no echo" (sensor tilted, since fixed). Rerun with `-Test bench`. |
