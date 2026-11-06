# Subsystem 4 — test log

**Subsystem:** IMU motion and terrain monitoring (imu_task, LSM303DLHC on Grove 3)  ·  **Student ID:** 2502114  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | Static pitch vs a reference level | FR8, FR18, A6 | manual | Normal build. Car flat 5 s ("S4 CAL done"). Phone (Measure > Level) on the car; tilt to 3 angles, 3 s each; read `p=` (1/100 deg). | IMU within 1° of the phone (change from flat) |
| S4-U02 | Hump detection and peak height | FR8, NFR8, A17 | manual | Build with `DRIVE_TEST 1` (not committed). Car self-drives 800 mm over the hump. Read "S4 HUMP #n peak=..". Compare with the ruler. | Hump detected; height within ±5 mm or ±20 % |
| S4-U03 | Impact -> MF_ESTOP + EV_IMPACT | FR14, NFR16, A18 | manual | Normal build. Bump the front into a fixed block 5 times; drive over the hump once. | "S4 IMPACT" each bump, brake ≤ 40 ms after the sample; no stop on the hump |
| S4-U04 | Turn rate from the encoders | FR20, A7 | manual | 90° and 180° turns; compare `w=` with video. | Within 10 % |
| S4-U05 | Motion class | FR19 | manual | Stop / accelerate / cruise / turn / hump / bump; read "S4 CLASS". | Correct class, none shorter than 100 ms |
| S4-U06 | Accelerometer identity (CTRL1/CTRL4 read-back) | FR18 | auto | `make TEST=1` | `[TEST] S4-U06 PASS` |
| S4-U07 | Magnetometer identity 'H43' | FR18 | auto | `make TEST=1` | `[TEST] S4-U07 PASS` |
| S4-U08 | Packed terrain fields clamp, never wrap | Tribunal 1 minimum-bit | auto | `make TEST=1` | `[TEST] S4-U08 PASS` |
| S4-U09 | Packed terrain round trip | Tribunal 1 minimum-bit | auto | `make TEST=1` | `[TEST] S4-U09 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-04 | S4-U01 | `bd59c6d` | Kessler | PASS | `logs/2026-11-04_s4_u01_tilt.txt` | Phone on the car read −2° flat, so phone 5/10/15° = 7/12/17° from flat. IMU 7.6 / 12.0 / 17.7°, max error 0.7°. Flat noise ±0.2°. Nose up = +, roll stayed 0°. |
| 2026-11-06 | S4-U02 | `c6375eb` | Kessler | FAIL | `logs/2026-11-06_s4_u02_hump.txt` | 10 mm strip under right wheel. Height from largest roll read 18 / 22 mm (jolt at strip edge adds to roll). |
| 2026-11-06 | S4-U02 | `c6375eb` + avg roll fix | Kessler | PASS | `logs/2026-11-06_s4_u02_hump.txt` | Height from average roll: 9 / 10 / 10 mm on the 10 mm strip, all within ±5 mm. Hump detected 3/3. |
| 2026-11-06 | S4-U03 | `d2187f1` | Kessler | FAIL | `logs/2026-11-06_s4_u03_impact.txt` | Wall hit at 150 mm/s, IMPACT_MG 2500. Hits read 2172 / 2428 mg, no trigger, car kept pushing ~2 s. |
| 2026-11-06 | S4-U03 | `d2187f1` + IMPACT_MG 2000 | Kessler | PASS | `logs/2026-11-06_s4_u03_impact.txt` | 3/3 triggered: 2499 / 2444 / 2069 mg, braked 13.7 / 18.6 / 9.3 ms. Soft push (~1.8 g) does not trigger. |
| 2026-11-06 | S4-U04 | `13e3101` + hd= heading | Kessler | FAIL | `logs/2026-11-06_s4_u04_turn.txt` | Hand pivot on the left wheel. 90° read 17° / 34°, 180° read ~6° / −102°. Right encoder reads ~1/5 of the distance and no direction. Blocked on S2 (Arif). |
| 2026-11-06 | S4-U04 | `6340b47` + S2 TURN test | Kessler | PASS | `logs/2026-11-06_s4_u04_turn.txt` | Motor turn with S2 TURN cmd (150 mm/s). 90° read hd 94 (+4.4 %), 180° read hd 191 (+6 %). Hand test above was invalid: encoders have no direction, S2 takes the sign from the motor cmd. |
