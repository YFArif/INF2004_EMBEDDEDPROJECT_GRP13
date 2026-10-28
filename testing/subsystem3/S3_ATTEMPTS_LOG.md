# Subsystem 3 — line following: everything tried (attempts log)

Sean (Buddy 3). This file records each firmware that was tried on the car, what changed, what happened, and what was learned from it. Formal test results are kept in `TEST_LOG.md`; this file is the story behind them. Console logs are in `Projects/log/` on Sean's PC (COM4_*.txt).

**Rule for new entries:** add a row, never edit old ones. Say what you saw, give the log file, and mark guesses as UNVERIFIED.

## 1. Car and sensors (fixed facts)

- Centre IR (GP27 / ADC1) rides **one edge** of the 18 mm line. "Half black" (500 ‰) means on track.
- The outer IR (GP28 / ADC2, called "Left" in the code) is physically on the robot's **RIGHT**, about 20–21 mm outside the Centre.
- The ridden edge must be the outer sensor's side, which is the **RIGHT** edge (proven 26 Oct, see row 26-C).
- Steering goes S3 → S2 as a wheel-speed difference. S2 applies 50 % of it (`S2_S3_STEER_PERCENT`).
- **S2 stall guard:** if a wheel runs at ≥ 40 % duty with no encoder tick for 300 ms, S2 latches MF_ESTOP. It then brakes and **discards every command** until the latch is cleared.

## 2. Attempts on the car

| # | Date | Firmware (commit) | What changed | What happened on the car | Evidence | Learned |
|---|---|---|---|---|---|---|
| 26-A | 26 Oct | `S3_v15_DRIVE_TEST` (`512ba05`) | First drive test, right edge, Kp 150 / Kd 400, 150 mm/s | Weaved and lost the line after 429–450 mm, 5 runs | commit `824e2ef` message | 150 mm/s with these gains is unstable on a straight |
| 26-B | 26 Oct | `S3_v15b_DRIVE_TEST` (`2b04bf7`) | Starts only from FOLLOW | See row 28-A (re-tested 28 Oct) | | |
| 26-C | 26 Oct | `824e2ef` (LEFT edge) | Rode the LEFT edge | Left curves blind: the Centre crossed the line, read white and steered the wrong way | `cd0bdc6` message | The ridden edge must be on the outer sensor's side (RIGHT) |
| 26-D | 26 Oct | `eb1fe2c` + `cd0bdc6` | Curve boost (full lock outside 250–750) and back to the RIGHT edge | Not logged separately | | |
| 26-E | 26 Oct | `S3_FINAL5_DRIVE_TEST` (`b38339a`) | Speed presets (default 100 mm/s, Kp 100 / Kd 300) and boost on | GO 150 → LOST after 457 mm. GO 150 → LOST after 62 mm. GO 100 → LOST after 468 mm. Turned away mid-way along a straight | `COM4_2026_10_26.19.09.21.426.txt`, `…19.13.57.722.txt` | Boost on still loses the line. Never stable |
| 27-A | 27 Oct | `S3_v16_*` (`4a8c395`) | Refactor only (drivers / tasks split); host output identical to v15 | Never flashed | | |
| 28-A | 28 Oct | **`S3_v15b_DRIVE_TEST`** (fixed 150 mm/s, Kp 150 / Kd 400, **no boost**) | — | **Line tracing good, left turns good. Right turn and sharp turns lose the line. Too fast.** Once lost, sat still showing `FOLLOW`, `turn LEFT 200 mm/s` for over 6 s | `COM4_2026_10_28.16.48.03.873.txt` | See section 3 |
| 28-B | 28 Oct | `S3_v17_DRIVE_TEST` (v17: priorities 8/12, boost on) | Optimisations, FR5, timing prints | **"Crash at start, keeps turning around and around. Too fast. Button 1 not working."** | No log saved | Most likely cause: the boost (full lock as soon as the Centre leaves 250–750). UNVERIFIED without a log. The priority change is the other candidate |

| 28-C | 28 Oct | `S3_v18_DRIVE_TEST` (presets 0 and 1) | v15b steering at 80 mm/s, corner rule, Button 1 only | **Hunting:** the Centre read BLACK or WHITE in about 85 % of prints and EDGE in only 10–22 of 108–150, flipping about 980 ↔ 8 every 200 ms, with steering swinging from full right to full left. Corner rule fired 5× on normal hunting and forced full right (left turn → went right; "turns 360 out of nowhere"). S2 `STALL ... MF_ESTOP` after 1101 mm. Centre followed the RIGHT edge; Sean expected the LEFT | `COM4_2026_10_28.17.22.49.291.txt`, `…17.23.11.183.txt`, `…17.24.04.113.txt` | (1) Steering was not scaled to speed: full lock = 200 mm/s difference, so at 80 mm/s the inner wheel reversed, causing over-correction and a stall. (2) The corner rule cannot tell a corner from hunting. (3) Riding the edge next to the outer sensor makes it useless |
| 28-D | 28 Oct | **v19** (`S3_v19_DRIVE_TEST.uf2`) | Straddle layout (Centre LEFT edge, outer just outside the RIGHT edge), outer as a steering input, OFF LINE / LOST direction from the last-seen sensor, steer max = speed, corner rule removed, new pins | **Not yet run on the car** | host sim: 18/18 runs PASS | See section 5b |
| 28-E | 28 Oct | v19, outer sensor at ~33 mm from the Centre | — | Centre on the EDGE in only 14–21 % of prints; S2 stalls | `COM4_2026_10_28.18.04.15.418.txt` … `…18.12.00.933.txt` | ~33 mm is too wide: the outer sensor only sees the line after a large drift |
| 28-F | 28 Oct | v19, outer sensor at **20 mm** (centre to centre) | Sean moved the sensor | **Best so far but "damn shaky".** EDGE 52 % of prints, 2.1 m before LOST. Steering changed sign in 27 % of prints; mean \|dv\| inside the band only 20 mm/s; the OUTER rule jumped to a fixed ~58 mm/s. Ended in LOST: the recovery pivot (2× steer) swept past the line, the Centre peaked at 371 / 295 ‰ and never read black | `COM4_2026_10_28.18.17.28.771.txt` | (1) P too weak in the band, (2) Kd 400 turns sensor noise into left/right flips, (3) fixed OUTER step jerks, (4) 2× search is too fast to stop on the line. Calibration black was 3939, so "black" needs > 3100 raw |
| 28-G | 28 Oct | v19, calibration after a power cycle | — | Full sweep: black Centre 3356 / outer 2955; Centre over the line reads 911–956 ‰. Calibration counted as "valid" at black 2099 (range ≥ 600) and kept rising while driving | `COM4_2026_10_28.18.22.26.400.txt` … `…18.22.47.017.txt` | A half sweep passes the "valid" test; the levels must be checked before a start. Lifting the car contaminates black |
| 28-H | 28 Oct | **v20** (`S3_v20_DRIVE_TEST.uf2`) | Kp 150 → 250, Kd 400 → 50; OUTER push proportional to the outer reading; search 2× → 1.25×; hold Button 1 4 s = recalibrate; start refused until both black ≥ 2500; Kp / Kd presets | **Not yet run on the car** | host 38/38, sim 19/19 (incl. new S3-S90) | See section 5c |

## 3. What the 28 Oct v15b log shows

**Right turn.** During the right curve the Centre stayed on the dark side for over 1 s (621–715 ‰), with weak right steering (33–65 mm/s). Then, between two 200 ms prints, it jumped straight to white (8 ‰). The outer sensor stayed white (1–17 ‰) and there was no black phase. This looks like a corner the car ran past (the line turned right under / behind the sensors), not a slow drift.

With "Centre white" the code assumes the car drifted off the right edge, so it steers **left**, away from the line. Left turns work for the same reason: there the line really does go left. This is built into following one edge: white alone does not say which side the line is on.

**"Lost" that never ends.** For over 6 s the Centre reading barely changed, so the car was not moving. LOST is measured in distance travelled (40 mm with both sensors white), so a car that is not moving never reaches LOST and the drive test never stops.

The likely reason the car stopped is the S2 stall guard. At full lock (dv 200 → ±100 mm/s per wheel) one wheel can sit at high duty without ticking for 300 ms. S2 then latches MF_ESTOP and discards everything after it, including the next run's FOLLOW, which would also explain "Button 1 not working". **UNVERIFIED:** the v15b log does not print S2's state. v18 prints `S2: stalls=… e-stops=… discarded=…` at every stop.

**Calibration.** Both sensors were calibrated (Centre 142–4056, outer 140–4028). Calibration is not the cause. Each sensor needs its own white and black level, as in Pololu's QTR library.

## 4. Things checked and ruled out (28 Oct)

- **S2 code:** FOLLOW and steering are correct (dv sign flip, 50 %, 100 ms stale check). Arif's `4690bd9` (28 Oct) changes only on-the-spot TURN calibration (right track 110 → 117 mm) and test logs, not FOLLOW. It has been merged in v18.
- **Something else driving the motors:** nothing in a `TEST=0` build sends FOLLOW or TURN without Button 1. "Turning around and around" does match S2's own floor test S2-U03 (13 on-the-spot turns, started by Button 1), but that only exists in `make TEST=1` builds.
- **Sensor channels in the v16 refactor:** Centre = ADC1, outer = ADC2, the same as v15.
- **Task stacks:** the line task needs about 0.5 KB of its 2 KB.
- **Odometry hand-over:** S2 calls `s3_set_odometry()` at start-up.

## 5. v18 (28 Oct): what it changed (superseded by v19, see 5b)

`S3_v18_DRIVE_TEST.uf2`. Not yet run on the car.

1. **v15b steering by default:** the curve boost is off (`S3_LN_BOOST_X100` 0). It can still be switched on at run time (preset 4).
2. **Slower:** 80 mm/s default, with 60 and 100 as presets.
3. **Corner rule (preset 1 and up):** the Centre was dark (> 600 ‰) for ≥ 10 mm, then both sensors go white within 6 mm → full lock towards the outer sensor (right), and LOST keeps that direction. A slow drift through the band does not trigger it (host test S3-L31). The console prints `S3 EVENT CORNER`.
4. **Button 1 only:** short press = start or stop, hold 1.5 s = next preset.
5. **S2 E-STOP:** a latched E-stop is reported and cleared before each start, and a run ends at once with the reason if it latches while driving.
6. **The run also stops** after 2 s of LOST, or 2 s with no odometry change, so it can never sit for ever.

**Test order (stop at the first that fails, and save the COM4 log):**
1. Preset 0 (80 mm/s, v15b steering), straight line, 3 runs. Must not be worse than v15b.
2. Preset 0, a left turn and a right turn, 3 runs each. Note the stop reason and the S2 counters.
3. Preset 1 (corner rule on), the same right turn, 3 runs. Look for `S3 EVENT CORNER` at the turn.
4. If it still goes wrong, preset 2 (60 mm/s).

If a run ends with `S2 E-STOP`, the problem is the stall guard at full lock. That is a decision for Arif (lower `S3_STEER_MAX_MMS`, or S2's stall threshold), not S3 tuning.

## 5b. v19 (28 Oct): the straddle layout

The wiring was changed by Sean on 28 Oct: outer IR on GP26, Centre on GP27, barcode AOUT on GP28, DOUT on GP7. The photo shows the outer sensor to the RIGHT of the Centre.

    C |##########| O     Centre on the line's LEFT edge; outer just outside the RIGHT edge

| Situation | What the sensors see | v19 steering |
|---|---|---|
| On track | C ≈ 500 (half black), O white | PD ≈ 0 |
| Drift right | C darker | PD left |
| Drift left | C lighter, then O black | PD right, at least 600 ‰ once O is black (OUTER) |
| Both white | — | Full lock towards the sensor that saw the line LAST (outer → right, Centre → left; both together → right). Decided once and held |
| LOST (40 mm both white) | — | Keeps turning (double steer max = pivot) until the Centre is back on the CORRECT edge |
| Cross junction | C and O both black | Steer 0, JUNCTION event |

**Evidence (host, 28 Oct):**
- `host/sim_track.c` drives the real `s3_line.c` on a model car (S2's 50 % steering, ramp, 112 mm track).
- Six tracks: straight 2 m, left and right curves r200, sharp RIGHT 90°, sharp LEFT 90°, S-bend r150.
- Each track is run at three sensor lookaheads (40, 60, 80 mm) and three layouts (21 mm, ~33 mm as in the photo, and the mirror image).
- Result: 18/18 PASS, with the axle at most 25 mm from the line.
- With the edge setting deliberately wrong, the sharp right turn fails (negative check).
- Line host tests: 36/36.

**Limits:** the simulation has no wheel slip, motor lag, sensor noise or stall, and its lookahead and spot size are guesses. It shows that the rules steer the right way and settle; it does not prove the car will.

## 5c. v20 (28 Oct): calmer steering for the 20 mm layout

Cause of the shaking (row 28-F): the PD was weak where it matters (in the band) and noisy where it should be quiet (Kd 400 on raw readings), and two rules stepped instead of growing.

| Change | v19 | v20 | Why |
|---|---|---|---|
| Gains | Kp 150, Kd 400 | **Kp 250, Kd 50** | Stronger in the band, D no longer amplifies noise |
| OUTER rule | fixed 600 ‰ once the outer is black | **600 × (n_outer − 250) / 750**, from the moment the outer leaves white | Grows smoothly: 0 at white, 400 at 750, 600 fully black |
| OFF LINE / LOST full lock | 2 × steer max (pivot) | **1.25 × steer max** | The pivot swept past the line (28-F); the inner wheel now keeps rolling forward |
| Calibration | power-cycle to redo | **hold Button 1 for 4 s**; levels printed as `S3 DRIVE CAL`; start refused until both black ≥ 2500 raw | Half sweeps looked valid (28-G) |
| Presets (all 80 mm/s, steer max 80) | speeds and gains mixed | 0: 250/50, 1: 200/50, 2: 300/50, 3: 250/0, 4: 250/100 | Only the gains change, so runs compare directly |

**Evidence (host only):** new sim test S3-S90 drives an S-bend with uniform sensor noise of ±0 / 40 / 80 raw. v19 gains: steering sign flips 0 / 23 / 28 % of steps, Centre on the edge 59–60 %. v20 gains: 0 / 12 / 18 %, Centre on the edge 95–100 %. The v19 figure at ±80 (28 %) is close to the car's 27 %, which suggests the noise level is about right, but the noise model is a guess (UNVERIFIED). Host tests S3-L36 (OUTER push 200 at outer 500 ‰, 40 at 300 ‰) and S3-L37 (search dv = 5/4 × steer max) are new.

**Test order on the car (save each COM4 log):**
1. Hold Button 1 for 4 s, sweep both sensors across the line until `S3 DRIVE CAL ... READY`. Note the four numbers.
2. Preset 0, the same track as 28-F, 3 runs. Compare: EDGE % of prints (target > 52 %), sign flips (target < 27 %), distance before LOST (target > 2.1 m).
3. If it still weaves on the straight → preset 1. If it is late on curves → preset 2. If it is still shaky → preset 3. If it overshoots after a curve → preset 4.

## 6. Open questions

- What exactly did v17 do? A COM4 log of one v17 run would show whether it was the boost or something else. (Flash `S3_v18_DRIVE_TEST`, preset 4 = v17 steering, to compare.)
- What is the sharpest turn on the track (radius, or 90° corners)? It decides whether one edge sensor plus the corner rule is enough.
- Button 2 cannot be pressed: is that a hardware problem (button or wiring)?
- (28 Oct, v20) How noisy are the sensors at speed? A COM4 log of the Centre held still on the edge for 5 s would give the real noise level for the simulation.

## 7. Sources used for the method

- Pololu QTR library: calibration is stored separately for each sensor (minimum and maximum, then scaled to 0–1000). <https://www.pololu.com/docs/0J18/16>
- "PID Controller for Lego Mindstorms Robots" (edge following): white means one side and black the other. A robot that is too fast loses the line on curves whatever Kp is, so start slow. Lower Kp if it hunts, raise Kd against overshoot. <https://cs.au.dk/~ocaprani/legolab/Danish.dir/WRO2018Regular/LeftEdgeDrive/PID%20Controller%20For%20Lego%20Mindstorms%20Robots.pdf>
