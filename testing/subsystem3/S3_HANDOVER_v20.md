# Subsystem 3 handover: line following and barcode (Sean, Buddy 3), 28 Oct 2026, v20

INF2004 Autonomous Robotic Car, group 13. Sean owns **Subsystem 3 (S3)**: IR line following and Code 39 barcode navigation, on a Pico W with μT-Kernel 3.0. This page is for the friend taking over. It covers what was done, what works, and what to do next.

## 1. Rules (keep these)

- **No Claude co-author or "Claude-Session" lines in git commits.** The repo is public. This happened once, and the history had to be rewritten and force-pushed.
- **Sean (or you) reviews and pushes. Claude never pushes.**
- **Only the owner edits a subsystem's folder** (TEAM.md). A change to someone else's code goes in its own commit for them to confirm. TEAM.md is shared, so commit it separately.
- **Buddy 2 (Arif) rules**, from MOTOR_S3_REFERENCE.md:
  - S3 must not clear an S2 E-stop just to make tests run.
  - Do not change the right-TURN constant to tune line following.
  - Do not flip the right motor's polarity to fix a sensor problem.

## 2. Where everything is

- Repo: `github.com/YFArif/INF2004_EMBEDDEDPROJECT_GRP13`. On Sean's PC it is `Projects/05_Group_Repo`.
- Branch **`buddy3-cleanup`**. It is **not pushed**. Its contents are in Sean's working tree as uncommitted changes on top of `4a8c395`.
- S3 code: `app_program/subsystem3/`. Tests and logs: `testing/subsystem3/`.
- **Read these first:**
  - `testing/subsystem3/S3_ATTEMPTS_LOG.md`: every firmware tried on the car, what happened, and the log file for each.
  - `app_program/subsystem3/README_S3.md`: wiring, the drive test, buttons and presets.
- Firmware images: `Projects/03_S3_Firmware_v15/`.
  - `S3_v20_DRIVE_TEST.uf2` is the one to test with.
  - `S3_v20_main.uf2` is the main build, with the drive test off.
- Car console logs: `Projects/log/COM4_*.txt`.
- Build: `cd build_make && export PICO_SDK_PATH=<pico-sdk 2.2.0> && make -j8`. Version 2.1.1 fails to link.
  - Drive-test build: set `#define S3_DRIVE_TEST (1)` in `s3_config.h`.
  - **Set it back to 0 before committing.** v19 was committed with 1 by mistake, and v20 fixes that.

**To push (in this order):**
1. `git merge origin/main`
2. Commit TEAM.md on its own.
3. Commit `app_program/subsystem3` + `testing/subsystem3`.
4. Push.

## 3. Hardware (current, 28 Oct)

| Item | Value |
|---|---|
| Outer line IR (called "Left IR" in the code) | GP26 / ADC0. Physically on the robot's **RIGHT**, **20 mm** centre to centre from the Centre sensor |
| Centre line IR | GP27 / ADC1, slightly left of the car's centre |
| Barcode sensor | AOUT GP28 / ADC2, DOUT GP7 |
| Buttons | Button 1 GP20 (does everything in the drive test). Button 2 GP21 (seems not pressable, possibly a hardware fault) |
| Layout ("straddle") | `C |##line##| O`: the Centre rides the line's **LEFT edge** (reads about 500 ‰ = half black) and the outer sensor sits just outside the RIGHT edge (white). `S3_EDGE_SIDE = S3_LN_EDGE_LEFT` in `s3_config.h` |
| Tasks | `task_line` priority 8, 10 ms. `task_barcode` priority 12. Both on core 1. Drive test priority 17, core 0 |

## 4. How the line following works (v20)

- **PD** on the Centre reading: `e = n_C − 500`. **Kp 250, Kd 50** (x100).
- **OUTER rule:** as the outer sensor darkens, the car is drifting left, so steer right by at least 600 × (n_outer − 250) / 750 ‰.
- **Both white:** full lock towards the sensor that saw the line last. The direction is decided once and held.
- **LOST** (40 mm with both sensors white): keep turning until the Centre is back on the correct edge.
  - Search full lock is **1.25 ×** the steer max.
- **Junction:** both sensors black for 8 mm. Steer is 0 and a JUNCTION event is raised.
- **Steering to S2:** a wheel-speed difference `dv`. S2 applies 50 % of it. The drive test sets steer max = speed (80 mm/s), so the inner wheel never reverses.
- **Calibration:** each sensor learns its own white and black levels while you sweep the car sideways across the line. Don't lift the car while calibrating: it reads "black" in the air.

## 5. What was tried on the car (summary of the attempts log)

| Version | Result | Lesson |
|---|---|---|
| v15b | Straight and left turns good. Right and sharp turns lost the line. Too fast (150 mm/s) | One edge sensor can't tell "drifted off" from "line turned away" |
| v17 | Spun at start, Button 1 did nothing (no log) | Probably the curve boost (full lock outside 250–750) |
| v18 | Hunting black↔white, a forced "corner rule" turned the car the wrong way, an S2 wheel stall | Steering not scaled to speed. Corner rule removed |
| v19, ~33 mm spacing | Centre on the edge only 14–21 % of the time, stalls | Too wide |
| **v19, 20 mm** | **Best so far: 2.1 m on track, but very shaky** (steering flipped sign in 27 % of prints) | Kd too high, P too weak, fixed-step rules jerked |
| v19 calibration check | A half sweep counted as "valid" at black 2099 and kept learning while driving | The black level must be checked before starting |
| **v20** | **Not yet run on the car** | Fixes for all of the above (next section) |

## 6. v20 changes (all done, committed locally, files on Sean's PC)

1. Gains Kp 150/Kd 400 → **250/50**.
2. OUTER push grows gradually instead of a fixed jump.
3. Search turn 2× → **1.25×**, so it should stop sweeping past the line.
4. **Hold Button 1 for 4 s = recalibrate** (no power cycle needed).
   - `S3 DRIVE CAL` prints both sensors' white/black levels.
   - The car **refuses to start until both black levels are ≥ 2500** raw.
5. Five presets that change only the gains, all at 80 mm/s:

   | Preset | Kp / Kd | Try it when |
   |---|---|---|
   | 0 (default) | 250 / 50 | start here |
   | 1 | 200 / 50 | it weaves on straights |
   | 2 | 300 / 50 | it is late on curves |
   | 3 | 250 / 0 | it is still shaky |
   | 4 | 250 / 100 | it overshoots after a curve |

**Evidence (host computer only, not the car):**
- Host tests 38/38. Simulation 19/19.
- In the simulation with sensor noise, the steering flipped sign 12–18 % of the time with the new gains, against 23–28 % with the old ones. **The noise level is a guess (UNVERIFIED).**
- The firmware builds with 0 warnings.

## 7. Drive test: how to run it

1. Flash `S3_v20_DRIVE_TEST.uf2` and open the serial console (COM4). Save the log.
2. Hold Button 1 for 4 s. Then slide the car sideways across the line until it prints `S3 DRIVE CAL ... READY`.
3. Put the car so the Centre is on the line's **left edge**.
4. Button 1 controls the run:
   - **short press:** start (3 s countdown);
   - **hold 1.5 s, let go before 4 s:** next preset;
   - **any press while driving:** stop.
5. The car stops by itself in these cases, printing the reason and the S2 counters:
   - LOST for 300 mm or 2 s;
   - not moving for 2 s;
   - an S2 E-stop;
   - 3 m travelled or 30 s elapsed.
6. If an **S2 E-STOP** is latched, power-cycle the car. S3 is not allowed to clear it.

**Compare against the v19 20 mm run** (`COM4_2026_10_28.18.17.28.771.txt`):
- Centre on the edge > 52 % of prints;
- steering sign flips < 27 %;
- more than 2.1 m before LOST.

## 8. Next steps, ranked

1. **(Most important) Run v20 on the car with preset 0**, 3 runs on the same track, and save the logs. Then try other presets based on section 7. Record each run as a new row in `S3_ATTEMPTS_LOG.md` and `TEST_LOG.md` (add rows, never edit old ones).
2. **Barcode on the car** (tests S3-U03, U04):
   - at least 9/10 correct per letter, both directions, 0 wrong letters;
   - include a slow 0.15 m/s run.
   - The decoder is tested A–Z both ways on the host, **but not yet while driving**.
3. Junctions on the car (S3-U05): 10/10 detected, 0 false on a straight.
4. **Integration (not started, waiting on teammates):**
   - **With Anna (Vehicle Controller / S5):**
     - S3 sends `s3_nav_msg_t` into `mbf_nav`, but nothing reads it yet.
     - S5 expects `subsystem5_nav_post()` with commands numbered 0–3, while S3 uses NONE=0, LEFT=1 … UTURN=4. Change S3 to 0–3 (the Tribunal 1 promise was a 2-bit command).
     - S5 also turns after a 150 mm fallback even without a junction, which can turn before the cross-junction.
   - **With Caleb (MQTT / S1):**
     - The barcode letter map is not wired yet: nothing calls `s3_set_nav_map("DCBA")`.
     - S1's command has no free opcode, so use a separate `cmd/map` topic.
   - **With Arif (S2):** optional. One linker line (`*(.time_critical*)` in `.data`) lets `S3_ISR_IN_RAM 1` put the interrupts in RAM.
5. Wiring checks A2/A3 (S3-U06) and the timing prints (S3-U07: copy 3 `S3 TIME` lines).

## 9. Instructor's demo requirements (status)

| Requirement | Status |
|---|---|
| Follow the line, including curves | v20 ready, needs the car test |
| Decode Code 39 A–Z | Done on the host |
| Read the barcode both directions | Done on the host, not yet on the car |
| Store the letter, act at the cross-junction | S3 side done (event + message). Needs S5 integration |
| Letter → action map sent over MQTT, not hard-coded | S3 function exists. Needs S1 integration |
