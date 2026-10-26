# Buddy 3 (Sean) – checklist and test plan (firmware v14b)

Tick a box only with evidence: a pasted log in `testing/subsystem3/TEST_LOG.md` and the git commit hash.

## Builds (switches in `s3_config.h`)

| Switch | main | Use |
|---|---|---|
| `S3_DRIVE_TEST` | 0 | 1 = Button 1 starts line following on Buddy 2's motors |
| `S3_LINE_VERBOSE` | 1 | `S3 LINE` status every 200 ms (both sensors: zone, permille, calibration range) + events |
| `S3_JUNC_DEBUG` | 1 | one `S3 JDBG` line per pass of the outer sensor over black |
| `S3_BC_VERBOSE` | 0 | 1 = print every barcode attempt (digital and analogue result, fusion) for barcode tuning |

The v15 desk builds (`S3_TEST` 1 to 4, one sensor on GP26/27/28) were removed in v16: the robot wiring with `S3_LINE_VERBOSE` and `S3_BC_VERBOSE` shows the same numbers.

## A. Barcode (write-up: barcode detection, decoding, navigation command generation)

- [ ] A1 Slit fitted (2 mm gap along the stripes, over both lenses); re-tune trimpot to the new middle
- [ ] A2 Parked test: narrow black – narrow white ≥ 800 counts (paper taped down, car put down gently)
- [ ] A3 Measure a real track barcode: narrow bar = ___ mm, length = ___ mm
- [ ] A4 Decode at constant speed (motors, not hand): **≥ 18/20 correct, 0 wrong** for `*A*`
- [ ] A5 Same for `*B*`, `*C*`, `*D*` (≥ 18/20 each, 0 wrong)
- [ ] A6 Crossed backwards: ≥ 9/10 correct, `(read backwards)` printed
- [ ] A7 Two speeds (e.g. 0.2 and 0.3 m/s): A4 holds at both
- [ ] A8 Command printed matches: A = TURN LEFT, B = TURN RIGHT, C = GO STRAIGHT, D = U-TURN
- [ ] A9 One barcode → exactly one `S3 NAV` line (cooldown works), 10/10
- [ ] A11 Day-of map: Buddy 1's MQTT command calls `s3_set_nav_map("DCBA")`; console shows `S3 NAV MAP set: LEFT=D RIGHT=C STRAIGHT=B U-TURN=A`; a `*D*` pass then gives TURN LEFT — no recompile
- [ ] A12 Invalid map (e.g. "DDBA") → `REJECTED (kept)`, old map still used
- [ ] A13 Confirm with the instructor: the order of the day's string (LEFT, RIGHT, STRAIGHT, U-TURN?) and how it is sent
- [ ] A10 Fusion evidence: count `both agree` / `analog only` / `digital only` over the 20 runs (for the report)

## B. Centre sensor + line following (write-up: calibration, position estimation, algorithm)

- [ ] B1 Calibration: after one sweep over white and the line, `UNCALIBRATED` disappears; `cal C[white..black]` sensible
- [ ] B2 Position: Centre on the RIGHT edge of the line (v14: `S3_EDGE_SIDE` = right) → `nC` ≈ 500 and `steer` ≈ 0 (within ±100)
- [ ] B3 Direction: slide the car left (onto the line) → `steer` positive; right (off the line) → `steer` negative (record the signs you see; if reversed, report before changing anything)
- [ ] B4 Lighting: B2 still holds with room lights off (write-up: "minor lighting changes")
- [ ] B5 With motors (needs Buddy 2/5): follows a straight 1 m line without leaving it, 5/5
- [ ] B6 With motors: completes the curve sections / full lap, 3/3
- [ ] B7 Speed variation: B5 at two speeds

## C. Left sensor + junction / line lost (write-up: junction detection)

- [ ] C1 Calibration as B1 for `cal L[...]`
- [ ] C2 Junction: push across a junction (Left and Centre both on black ≥ 8 mm) → exactly 1 `S3 EVENT JUNCTION`, 10/10
- [ ] C3 No false junction: a straight line, 3 m → 0 junction events
- [ ] C4 Line lost: lift the line away / move off the line ≥ 40 mm → exactly 1 `S3 EVENT LINE LOST`, 5/5
- [ ] C5 Small gap (< 40 mm white) → no LINE LOST, 5/5
- [ ] C7 Drift / hard curve onto the line → NO junction event, 10/10 (v12 junction-lead rule)
- [ ] C8 Overshoot (Left on the line, Centre past it) → status shows OVERSHOOT and steer −1000, 5/5
- [ ] C9 Search (IF-07): set `LF_SEARCH_R`, approach from the right → exactly 1 `LINE FOUND` at the first edge, 5/5
- [ ] C10 Search from the wrong side: set `LF_SEARCH_L`, approach from the left → no FOUND at the first edge, 1 FOUND after crossing, 5/5
- [ ] C11 After FOUND, set `LF_ENABLE` then clear `LF_SEARCH_x` → state stays FOLLOW, 0 extra FOUND (host test L27 covers the logic), 5/5
- [ ] C12 Clear `LF_ENABLE` → status DISABLED, steer 0, no junction/lost events
- [ ] C6 Parked (not moving) on black or white → no events (needs odometry from Buddy 2; with the bench default speed this one is expected to FAIL – record it)

## D. Integration (with other buddies)

- [ ] D1 Copy IF-01, IF-02, IF-07, IF-S3b..f from `s3_api.h` into TEAM.md section 5; team agrees
- [ ] D2 Buddy 2 calls `s3_set_odometry()`; C6 passes after that
- [ ] D3 Vehicle Controller (Anna) receives the 4-byte `s3_nav_msg_t` from `mbf_nav`, uses `s3_nav_msg_char()`, and turns correctly, 10/10
- [ ] D4 Motion (Arif) reads `s3_get_steer_sp()` every control step and stops trusting it when `seq` stops changing (B5/B6)
- [ ] D10 Vehicle Controller sets `flg_line` in the documented order (`LF_ENABLE` first, then clear `LF_SEARCH_x`)
- [ ] D5 Buddy 1 publishes line state + last barcode over MQTT (telemetry)
- [ ] D6 Shared GPIO interrupt (IO_IRQ_BANK0) merged with Buddy 2/5 encoders/ultrasonic (only ONE `tk_def_int` on IRQ 13 is allowed)
- [ ] D7 ADC owner = S3 sampler interrupt; nobody else reads the ADC (TEAM.md section 3). Ask the team whether DEVCNF_USE_ADC can be set to 0 so the kernel ADC driver cannot be opened by mistake
- [ ] D8 Tuning: Buddy 1's MQTT command handler calls `s3_set_line_gains(kp, kd)` / `s3_request_recalibrate()` (IF-S3e); tune from MQTT Explorer or a phone MQTT app — no custom frontend
- [ ] D9 Only the Vehicle Controller waits on `flg_s3`; telemetry uses the counters in `s3_line_status_t`

## E. Evidence for the Tribunal

- [ ] E1 Host tests pass: `test_code39`, `test_analog`, `test_line` (L01–L27, N01), `test_nav` (C01: all 43 Code 39 characters both directions) — every file prints `ALL PASSED: 0 failure(s)` (commands in each file's header)
- [ ] E6 Change Requirement 1 evidence: `S3_DATA_REPRESENTATION.md` (field table, RAM 67,790 → 9,678 B, NAV message 12 → 4 B, status 32 → 16 B); re-run `arm-none-eabi-size` on your own build and paste it into TEST_LOG.md
- [ ] E7 Measure ISR time before/after 8-bit samples (E2 method) — the performance row is UNVERIFIED until then
- [ ] E2 CPU load: time the sampler interrupt (toggle a spare pin, scope) → ___ µs × 2000/s = ___ %
- [ ] E3 Every result above logged with test ID + commit hash
- [ ] E4 Report updated: slit + 0.2 mm height, ADC owner, glitch 2 ms, ratio 1.3, mbf_nav (message buffer, not mailbox), edge side
- [ ] E5 AI usage recorded in report section 8.1
