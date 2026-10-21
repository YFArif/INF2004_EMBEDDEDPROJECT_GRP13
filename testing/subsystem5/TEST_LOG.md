# Subsystem 5 — test log

**Subsystem:** Obstacle scan, avoidance, vehicle controller  ·  **Student ID:** 2502447  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

**Bench set-up for the `make TEST=1` run:** robot on blocks, wheels free,
a flat box about 30 cm straight ahead of the sensor, nothing else within
1 m. Open the serial monitor within 5 s of power-up: subsystem 5 waits
5 s before its first line. The S5 part of the run takes about 45 s
(S5-U03 alone takes 14 s). `[S5] PING servo=… range=…` lines show every
ping during scans and every 5th while monitoring (`LOG_PINGS` in
`obstacle_task.c`).

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | HC-SR04 range accuracy and steadiness | FR9, NFR7 | auto + manual | `subsystem5_test.c`: 20 samples, 100 ms apart, of the live range. Repeat with the box at 10, 30 and 50 cm and compare the printed `S5-U01 mean_mm` with a ruler. | `[TEST] S5-U01 PASS`; mean within ±10 mm of the ruler at each distance |
| S5-U02 | ECHO voltage at GP3 is safe | A8 | manual | 1. Power the HC-SR04 with GP3 disconnected. 2. Put a target 20 cm away and ping it. 3. Measure the divider output with a scope or multimeter (peak). | ≤ 3.3 V; only then connect GP3 |
| S5-U03 | Servo angles and settle time | A9, FR10 | auto + manual | `subsystem5_test.c` sends `OB_CAL_SWEEP`: the servo holds 30, 60, 90, 120, 150° for 2 s each. Check each with a protractor (30° = robot's right). Repeat at full and low battery. | `[TEST] S5-U03 PASS`; each angle within ±3°; otherwise adjust `PULSE_*_US` / `TRIM_DEG` in `drv_servo.c` |
| S5-U04 | Live scan publishes a profile | FR10, NFR13, A16 | auto + manual | `subsystem5_test.c` sends `OB_SCAN_REQ`. Compare the printed `PROFILE ... width_mm` with the box's measured width. Later: 3 boxes at 15/20/30 cm, 10 runs each; fit `S5_EDGE_CORR_DEG`. | `[TEST] S5-U04 PASS`; width error ≤ 10 % |
| S5-U05 | 20 approaches at 0.3 m/s | FR9, FR16, NFR13 | manual | Needs S2 motion_task and vehicle_ctrl_task. | Detected 100 %, slows below 35 cm, correct side chosen 100 %, every decision logged |
| S5-U06 | Integer maths, scan logic and profile on a fixed simulated scene | FR10 | auto | `subsystem5_test.c`: sin/cos/isqrt checks, then a full scan and profile of a 150 mm box 150 mm ahead with a wall 310 mm to its left. No hardware needed. | `[TEST] S5-U06a/b/c PASS`; width 135–165 mm, left clearance 150–250 mm |
| S5-U07 | obstacle_task obeys OB_HOLD and OB_RESUME | IF-09, flg_obst | auto | `subsystem5_test.c` sends each request and reads the published state 300 ms later. | `[TEST] S5-U07a/b PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
