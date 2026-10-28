# Subsystem 3 — test log

**Subsystem:** IR line following, junction / line-lost detection, Code 39 barcode navigation  ·  **Student:** Sean (Buddy 3), 2500316  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | _template example:_ statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-H01 | Line combiner on a laptop: calibration, PD, curve boost, overshoot, junction, lost, search, late wake-up, straddle rules, proportional OUTER push, search dv (S3-L01 … L37, N01) | FR3, FR4, IF-02, IF-07 | auto (host) | `gcc` command at the top of `host/test_line.c` | all `PASS` (33 from v18), `ALL PASSED` |
| S3-H02 | Code 39 decoder: sample `*A*` / `*Z*`, backwards, speed scaling, rejects | FR4 (barcode), NFR3 | auto (host) | `host/test_code39.c` | 8 `PASS` |
| S3-H03 | Every character both directions at ratio 2.0–3.0, ±15 % jitter, −20 / +25 % speed drift; table vs python-barcode (S3-C03 … C05) | FR4, instructor A–Z + both directions | auto (host) | `host/test_code39_all.c` | 3 `PASS`, 0 wrong letters |
| S3-H04 | Letter map: default, set, reject, upper-case; every character both directions (S3-C01, C02, N01 … N04) | FR5, IF-S3f | auto (host) | `host/test_nav.c` | 6 `PASS` |
| S3-H05 | Analogue barcode widths, noise rejection | FR4 | auto (host) | `host/test_analog.c` (link with `-lm`) | 9 `PASS` |
| S3-H06 | Track simulation: the real `s3_line.c` on a model car, 6 tracks × 3 lookaheads × 3 sensor layouts, plus v19 vs v20 gains with sensor noise | FR3 | auto (host) | `host/sim_track.c` (link with `-lm`) | 19 `PASS` (18 tracks + S3-S90 noise comparison) |
| S3-U02 | Line tracking on the car | FR3 | manual | `S3_DRIVE_TEST 1`. Calibrate, then 3 runs each on a 3 m straight and on the course curves, 100 mm/s. | 3 of 3 runs stay on the line to the end; log saved |
| S3-U03 | Barcode capture: DOUT edges per pass | NFR2 | manual | `S3_BC_VERBOSE 1`. Push the car over `*A*` 10 times at a steady speed. | `edges=30` on ≥ 9 of 10 passes (29 elements + the idle gap) |
| S3-U04 | Barcode decode while driving | NFR3, FR4 | manual | `S3_DRIVE_TEST 1`. Per character: 10 passes forwards and 10 with the sheet turned 180°, at 0.15 / 0.25 / 0.35 m/s. Include the 4 letters of the day. | ≥ 9 of 10 correct per character, direction and speed; 0 wrong letters |
| S3-U05 | Junction and line-lost events on the car | FR3, IF-S3c | manual | `S3_JUNC_DEBUG 1`. 10 cross-junction passes; 3 m straight with no junction; lift the car off the line 5 times. | 10 of 10 `JUNCTION`, 0 false on the straight, 5 of 5 `LINE LOST` |
| S3-U06 | Wiring A2 / A3: GP26 driven by one output only; GP28 VBAT jumper open | A2, A3 | manual | Continuity meter; read GP28 with the sensor unplugged. | One source on GP26; GP28 reads ~0 unplugged |
| S3-U07 | S3 timing on the car | Week 6 §3 (line_task ≈ 1 % of core 1), INT-05 | manual | `S3_TIMING 1`, any drive test; copy 3 `S3 TIME` lines. | line step max < 100 µs (1 % of 10 ms); period 9 000 – 11 000 µs; sampler isr max < 20 µs (4 % of core 1 at 2 kHz). Targets UNVERIFIED until measured |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (host)** = laptop test with `gcc`, no Pico needed;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | S3-H01 | `4a8c395` + v17 (uncommitted) | Claude, reviewed by Sean | PASS (30/30) | `logs/2026-10-28_host_v17.txt` | L01 … L28 output byte-identical to `4a8c395`. New L29 fails on `4a8c395` (UBSan: signed overflow in `pd_steer`) and passes on v17 |
| 2026-10-28 | S3-H02 | `4a8c395` + v17 (uncommitted) | Claude, reviewed by Sean | PASS (8/8) | `logs/2026-10-28_host_v17.txt` | |
| 2026-10-28 | S3-H03 | `4a8c395` + v17 (uncommitted) | Claude, reviewed by Sean | PASS (3/3) | `logs/2026-10-28_host_v17.txt` | 10 320 / 10 320 forwards and backwards, 0 wrong. Mutation: 1 bit changed in 'A' → C03 FAIL |
| 2026-10-28 | S3-H04 | `4a8c395` + v17 (uncommitted) | Claude, reviewed by Sean | PASS (6/6) | `logs/2026-10-28_host_v17.txt` | |
| 2026-10-28 | S3-H05 | `4a8c395` + v17 (uncommitted) | Claude, reviewed by Sean | PASS (9/9) | `logs/2026-10-28_host_v17.txt` | |
| 2026-10-28 | S3-H01 | v18 (uncommitted) | Claude, reviewed by Sean | PASS (33/33) | `logs/2026-10-28_host_v18.txt` | New L30–L32 (corner rule). L28 now switches the boost on itself (default 0). L01–L28 output identical to v17 |
| 2026-10-28 | S3-H02 … H05 | v18 (uncommitted) | Claude, reviewed by Sean | PASS (8/8, 3/3, 6/6, 9/9) | `logs/2026-10-28_host_v18.txt` | |
| 2026-10-28 | S3-U02 | v15b `2b04bf7` (`S3_v15b_DRIVE_TEST.uf2`) | Sean | FAIL | `Projects/log/COM4_2026_10_28.16.48.03.873.txt` | Straight and left turns OK. Right / sharp turns lost (Centre dark → white, steered LEFT). Then stood still in FOLLOW for > 6 s. See `S3_ATTEMPTS_LOG.md` 28-A |
| 2026-10-28 | S3-U02 | v17 (`S3_v17_DRIVE_TEST.uf2`) | Sean | FAIL | no log saved | Spun at start ("turning around and around"), too fast, Button 1 did nothing. See `S3_ATTEMPTS_LOG.md` 28-B |
| 2026-10-28 | S3-U02 | v18 (`S3_v18_DRIVE_TEST.uf2`) | Sean | FAIL | `Projects/log/COM4_2026_10_28.17.22.49.291.txt`, `…17.24.04.113.txt` | Hunting (Centre on the EDGE in 10–22 % of prints), corner rule forced right turns, S2 stall after 1101 mm, wrong edge. See `S3_ATTEMPTS_LOG.md` 28-C |
| 2026-10-28 | S3-H01 | v19 (uncommitted) | Claude, reviewed by Sean | PASS (36/36) | `logs/2026-10-28_host_v19.txt` | Straddle rules: L16, L22, L30–L35 rewritten or new; L04, L10 and L33 start from the edge |
| 2026-10-28 | S3-H02 … H05 | v19 (uncommitted) | Claude, reviewed by Sean | PASS (8/8, 3/3, 6/6, 9/9) | `logs/2026-10-28_host_v19.txt` | |
| 2026-10-28 | S3-H06 | v19 (uncommitted) | Claude, reviewed by Sean | PASS (18/18) | `logs/2026-10-28_host_v19.txt` | Wrong edge setting: sharp RIGHT fails (negative check). Not a car test |
| 2026-10-28 | S3-U02 | v19 `c9b4648` (`S3_v19_DRIVE_TEST.uf2`), outer ~33 mm | Sean | FAIL | `Projects/log/COM4_2026_10_28.18.04.15.418.txt` … `…18.12.00.933.txt` | EDGE 14–21 % of prints, S2 stalls. See `S3_ATTEMPTS_LOG.md` 28-E |
| 2026-10-28 | S3-U02 | v19 `c9b4648`, outer 20 mm | Sean | FAIL (best so far) | `Projects/log/COM4_2026_10_28.18.17.28.771.txt` | 2.1 m on track, then LOST. EDGE 52 %, sign flips 27 %. See 28-F |
| 2026-10-28 | S3-H01 | v20 (uncommitted) | Claude, reviewed by Sean | PASS (38/38) | `logs/2026-10-28_host_v20.txt` | New L36 (proportional OUTER), L37 (search dv 5/4). L01–L35 unchanged and passing with the new default gains |
| 2026-10-28 | S3-H02 … H05 | v20 (uncommitted) | Claude, reviewed by Sean | PASS (8/8, 3/3, 6/6, 9/9) | `logs/2026-10-28_host_v20.txt` | |
| 2026-10-28 | S3-H06 | v20 (uncommitted) | Claude, reviewed by Sean | PASS (19/19) | `logs/2026-10-28_host_v20.txt` | 20 mm layout now (was 21). New S3-S90: with noise, v20 gains flip sign 12–18 % vs v19 23–28 %. Not a car test |
