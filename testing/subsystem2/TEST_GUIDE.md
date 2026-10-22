# Subsystem 2: how to run each test

This guide is for running the S2 tests on the real robot so that every result is complete and can be trusted. One script, `s2_test.ps1`, does the build, the flash and the logging. It also asks for your measurements at the right moments, so every run ends as one log file with nothing missing.

## 0. Before the first test

**Tools to have on the bench**

- Tape measure, at least 2 m (a metal one stays straight)
- Masking tape and a marker pen
- A protractor (the bigger the better) and a 30 cm ruler
- A sheet of A4 paper (its corner is an exact 90 degrees)
- A multimeter
- A box or a thick book to lift the robot so the wheels hang free
- The long USB cable and the power bank

**How to start a test**

Open a terminal in VS Code (Terminal > New Terminal, the PowerShell kind) in the project folder `INF2004_EMBEDDEDPROJECT_GRP13`. Type the command for the test, for example:

```powershell
powershell -ExecutionPolicy Bypass -File testing\subsystem2\s2_test.ps1 -Test unit
```

Only the word after `-Test` changes: `unit`, `u06`, `u02`, `u11`, `u03`, `u12`, `u01` or `u09`. Answer the questions it asks, then follow what it prints.

**Close the VS Code Serial Monitor (and PuTTY) first.** Only one program can open the Pico's port. The script opens it itself.

**The red squiggle in `motion_task.c`** (`cannot open source file "sysdepend/TARGET_DIR/sysdef.h"`) is from VS Code's code checker, not the compiler. The real build defines `_PICO_RP2040_` and works. The file `.vscode/c_cpp_properties.json` gives VS Code the same settings. If the squiggle stays, run "C/C++: Reset IntelliSense Database" from the command palette (Ctrl+Shift+P).

## 1. Keep the conditions the same every time

Results can only be compared if they are taken the same way.

| What        | Do this every time                                                                                                                                                                                                                      |
| ----------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Power       | Power bank on the Robo Pico's VIN terminal **and** the USB cable to the laptop. Answer `1` when the script asks. Charge the power bank before a session.                                                                                 |
| VIN voltage | Measure it before each test: multimeter on DC volts, red probe on VIN +, black on VIN - (GND), robot switched on and still. Type it when asked (for example `5.08`). Motor speed depends on it, so a result without it is hard to compare. |
| Floor       | The same surface each time, ideally the track paper. Type what it is when asked.                                                                                                                                                         |
| Cable       | Hold or tape the cable up behind the robot so it never pulls the robot sideways.                                                                                                                                                         |
| Commit      | The code must be committed. The script warns if `app_program/subsystem2` has uncommitted changes. Such a run is only practice and does not count in `TEST_LOG.md`.                                                                       |

## 2. What the script does

1. Shows the commit and asks: tester, power supply, VIN voltage, floor (floor tests only), notes.
2. Prints the setup checklist for the test and waits for Enter.
3. Builds the test image in WSL with that test's switch turned on, then puts the switch back to 0 in the source straight away. The code in git never changes.
4. Tells you to unplug the Pico, then hold BOOTSEL and plug it back in. It copies the image onto the `RPI-RP2` drive by itself.
5. Opens the Pico's serial port as soon as it appears, so nothing is missed. It shows the output (PASS green, FAIL red) and saves every line.
6. **Every step that moves the robot waits for Button 1** (the button marked GP20 on the Robo Pico). The terminal shows a yellow `NEXT:` banner with what the step does and what to watch, and beeps twice. Get the robot ready, then press the button. It beeps `3`, `2`, `1` and a long `GO`: **look at the robot from the first beep**. Nothing moves before you press, so take as long as you need.
7. When a step ends, the script says `Look at the robot now. Question in 3.. 2.. 1..`, then asks its question. The robot is stopped and waiting for the button, so you can answer, measure and reset the robot without hurrying. Each answer is saved as a `[MEASURED]` line.
8. Stops by itself at the end of the S2 tests and prints where the log is: `testing\subsystem2\logs\<date>_s2_<test>.txt`.

**Send me that log file.** It holds the commit, your setup, the Pico's output and your measurements, so nothing else is needed.

If a step goes wrong (the robot hits something, you mis-measure), let the run finish or press Ctrl+C. Then run the same test again; the new log gets a `_2` name. Tell me which run to use.

## 3. The tests, in this order

Every test image runs the automatic `unit` checks first (about 10 s, motors still), so every log contains them too.

### Test 1: `unit` (S2-U08, U05, U07, U10). Motors do not move.

- **Why:** checks the timer, that `motion_task` runs every 20 ms, that the brake reacts to the stop flag within 20 ms, the stop-latch rules, and the CPU time and stack use (NFR11).
- **Setup:** none; the robot can stand anywhere.
- **Takes:** about 15 s after flashing.
- **You do:** nothing.
- **Good result:** `Done: 5 passed, 0 failed`. The `S2-U10 INFO` line gives the longest cycle (limit 4000 us) and the stack use (limit 75 %).

### Test 2: `u06` motor direction (S2-U06). Wheels lifted.

- **Why:** if a wheel turns the wrong way, every MOVE and TURN is wrong. Do this before any floor test.
- **Setup:** put the robot on a box so both wheels hang free. Stand **behind** the robot, its front pointing away from you. "Forward" for a wheel then means the top of the tyre moves **away** from you.
- **Takes:** about 2 minutes (6 runs, each started by Button 1).
- **What happens:** LEFT forward, LEFT reverse, LEFT forward fast, then the same for RIGHT. Each run is 3 s.
- **You do:** press Button 1 for each run and watch that wheel. After each run the script asks:
  1. After FORWARD: did it turn the way that drives the robot **forward**, and did **only** that wheel turn?
  2. After REVERSE: did it turn the **opposite** way?
  3. After FAST: forward again, and faster?
- **Good result:** every `S2-U06` line PASS and `y` to every question. The PASS only proves the encoder counted; the **direction is only checked by your eyes**, because the encoders cannot tell direction. If a wheel turns the wrong way, stop and tell me.

### Test 3: `u02` speed controller (S2-U02). Wheels lifted.

- **Why:** first real check of the speed controller (NFR4: within 10 % of the target in 0.5 s).
- **Setup:** same as `u06`.
- **Takes:** about 15 s after you press Button 1. Both wheels spin for 1.5 s.
- **You do:** press Button 1, then watch both wheels from behind. Afterwards the script asks whether both turned the way that drives the robot **forward**.
- **Good result:** `S2-U02 PASS` and `y`. The `[S2-U02]` lines are the speed every 20 ms, which I use to tune.

### Test 4: `u11` MOVE distance and straightness (S2-U11). On the floor.

- **Why:** everyone uses the robot's distance (Sean's junction and barcode distances, Kessler's hump height, Anna's obstacle depth). This test checks it against a tape measure.
- **What to measure, and why the tyre:** on a straight run every point of the robot moves the same distance, so you can measure any point that is easy to read. The car's long nose and tail make the body hard to line up, so use the **front edge of the LEFT tyre where it touches the floor**. It sits on the floor, so there is no parallax, and it is exactly what the encoders measure. Do **not** count wheel turns instead: that only checks the encoder again and misses wheel slip, which is what this test is for.
- **Setup:**
  1. **Space:** a clear, flat strip **1.6 m long and 0.6 m wide**, plus the robot's own length behind the start, nothing at the far end, and slack in the cable.
  2. Lay the tape measure **beside the left wheel**, not under the car: flat, straight, parallel to the way the robot will drive, with its edge about 2 cm from the outside of the left tyre. Tape it down at both ends and in the middle.
  3. Choose a **start mark**: either a line on the floor that you call 0, or a reading on the tape such as 100 mm (the hook end of a tape measure is not accurate, so do not start on the hook). Put the robot down with the **front edge of the left tyre, where it touches the floor,** on that mark, and both wheels parallel to the tape. Any start reading is fine: the distance is the end reading minus the start reading. Use the same point of the tyre at both ends.
- **Takes:** about 6 minutes (5 runs, each started by Button 1).
- **What happens:** runs 1-3 are MOVE 1000 mm at 300 mm/s, run 4 is MOVE 1000 mm at 150 mm/s, and run 5 is MOVE -300 mm (reverse).
- **You do, for each run:**
  1. **Before pressing the button** the script asks your **start reading** (Enter = 0) and the **gap** between the tape's edge and the outside of the left tyre, in mm. Putting the tyre right against the tape is fine: type `0`. If the robot then drifts left over the tape, type how far the tyre overlaps it as a negative number. Then press Button 1.
  2. **After the run:** look straight down at the same point of the left tyre and type the **reading** there, in mm (for example `1003` if you started at 0).
  3. Type the **gap** again, now at the end position.
  4. The script works out the real distance (reading - start) and the sideways drift (+ = drifted left, - = right) and saves both.
  5. Put the robot back on its start mark.

  **Run 5 (reverse):** the robot goes backward, so it moves **down** the tape numbers. Start it at **300 or more** (for example 400); it ends near 100. If your start line is 0, lay the tape so there is tape behind it, or use a different start mark for this run.
- **Good result:** every `S2-U11` line PASS. Real distances within 1000 +/- 10 mm (reverse: 300 +/- 3 mm) and sideways drift under 20 mm.
- **The script calibrates the wheels for you.** At the end it prints, for example, `left went 0.40 % further than counted` and two lines for `subsystem2.h`:
  `suggested: S2_WHEEL_CIRC_UM_L (208620)   S2_WHEEL_CIRC_UM_R (209010)`.
  If both wheels are within 1 % it says no change is needed. Otherwise put the two suggested numbers into `app_program\subsystem2\subsystem2.h`, commit, and run u11 again. Straightness and distance are fixed together: the robot curves when one wheel's calibration is off by about 1 %, because the controller keeps the *counted* distances of both wheels equal.
- **Why the wheel and not the body, and why the calibration matters:** see section 6.
### Test 5: `u03` turn accuracy (S2-U03). On the floor.

- **What this test is (and is not):** the robot does **not** drive along the tape and does not decide where to turn. It stays in one spot and **spins on the spot**, like a turntable, by the exact angle the test commands. In the finished system the *vehicle controller* (Anna) decides when and which way to turn, for example after a barcode, and sends S2 the command "TURN 90". This test only checks that S2 turns by the angle it is told. No sensors are involved: the robot counts wheel ticks and stops when the wheels have travelled the right arc.
- **Which way:** a **positive** angle is a **left** turn and a **negative** angle is a **right** turn. Left means anticlockwise when you look down on the car from above: stand behind the car and the nose swings to **your left**. The terminal banner says which one is next:
  - `TURN +90 (LEFT)`: nose ends pointing at arm **L**.
  - `TURN -90 (RIGHT)`: nose ends pointing at arm **R**.
  - `TURN 180`: nose ends pointing at arm **B** (backward).
- **Why it matters:** barcode turns (Sean to Anna to you) and Anna's 45-degree bypass turns depend on it (NFR5: 90 degrees within 5 degrees, 180 within 8).
- **Setup:**
  1. Make a `+` on the floor with two 40 cm strips of masking tape, crossing at exactly 90 degrees (lay the A4 corner in the crossing). Label the four arms F (forward), L (left), R (right), B (back). F is the direction the robot faces at the start. Stand behind the robot so L is on your left.
  2. Tape the 30 cm ruler on top of the robot along its centre line, sticking out past the front. This is the **pointer**.
  3. Place the robot so the middle of its axle (halfway between the two wheels) is over the crossing and the pointer lies exactly along arm F. The robot will turn around this point.
- **Takes:** about 8 minutes (13 turns, each started by Button 1).
- **What happens:** 5 turns of +90 (left), 5 of -90 (right), then 3 of 180. Each turn takes about 1 second; the robot stays at the crossing.
- **You do, for each turn:**
  1. Put the robot back on the crossing with the pointer along arm F, then press Button 1. Watch it spin.
  2. When the script asks, hold the protractor with its centre on the crossing and its 0 line along arm F. Read the angle from arm F to the pointer, counting **left as positive, right as negative**.
  3. Type the **total angle turned**. A 90-degree left turn that stopped 3 degrees short: `87`. That went 2 degrees too far: `92`. A right turn that stopped 4 degrees short: `-86`. A 180 that overshot by 5: `185`.
  4. If the robot slid off the crossing, read the pointer's direction, not where it is.
- **Good result:** every `S2-U03` line PASS, and every 90-degree answer within 85 to 95 (or -85 to -95), every 180 within 172 to 188.
- **The script calibrates the turns for you.** At the end it prints the average angle for left, right and 180 turns, and a suggested `S2_TRACK_MM`. If the turns are within 2 % it says no change is needed. Otherwise put the number into `app_program\subsystem2\subsystem2.h`, commit, and run u03 again. A consistent error is usually the effective track width: the tyres scrub a little when they turn on the spot, so the effective width is a bit bigger than the measured 112 mm.
### Test 6: `u12` braking, slow speeds, stall (S2-U12). On the floor.

- **Why:** Anna's 150 mm stop distance only works if the robot brakes from 300 mm/s within 58 mm (NFR6). Her slow mode needs 150 mm/s to hold steady. A stalled wheel must stop everything (FR15).
- **Setup:** 1 m clear in front. Put the robot back at the start before pressing Button 1 for each step. Each step drives about 0.1-0.7 m. Have the box ready for the last step.
- **Takes:** about 5 minutes.
- **What happens and what you do:**
  1. Drive 300 mm/s, then STOP. Did the wheels skid? (y/n)
  2. Drive 300 mm/s, then emergency stop. Did the wheels skid? (y/n)
  3. Drive 150 mm/s, then STOP. Did the wheels skid? (y/n)
  4. Drive at 40, 60, 100, then 150 mm/s, 2 s each, one step each. After each, did it move smoothly, without stopping or jerking? (y/n)
  5. **Stall:** put the robot on the box with both wheels free, then press Button 1. After `GO` both wheels turn slowly and it prints `HOLD the LEFT tyre now`. Pinch the **left** tyre firmly between finger and thumb so it stops. Within about 1 s both motors stop by themselves and `STALL` appears. Let go. The script asks whether both wheels stopped.
- **Good result:** every `S2-U12` line PASS: braking distance from 300 mm/s of at most 58 mm, 150 mm/s held within 10 %, stall detected. Give the braking distance to Anna too.

### Optional: `u01` encoder ticks per wheel revolution (S2-U01)

The calibration from 15 October is good enough, and `u11` checks it better. Do this only if I ask. Put a tape mark on each tyre. At the `NEXT: S2-U01 turn the LEFT wheel` banner, press Button 1. After `GO`, turn the left wheel slowly by hand exactly 10 times (count the mark) within 40 s, keeping the right wheel still. Then do the same for the right wheel. Type how many turns you really made.

### PC only: `u09` simulation (S2-U09)

Runs `motion_task.c` against a simulated robot on the PC. No Pico is needed. Run it after any change to `motion_task.c`.

## 4. Chassis measurements (once, by hand)

(The wheel calibration does not need a hand measurement: u11 works it out, see Test 4.)

Write these down and send them with the logs.

| What                               | How                                                                                                                                                                                                          |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Wheel diameter, left and right     | Ruler across the tyre at its widest, through the centre. More exact: mark the tyre and the floor, push the robot straight until the wheel has made exactly one turn, measure the distance and divide by 3.1416. |
| Track width                        | Outer edge to outer edge of the two tyres, and inner edge to inner edge. The track width is the average of the two.                                                                                         |
| Wheelbase                          | From the middle of the axle to where the caster touches the floor (Kessler needs it).                                                                                                                       |
| VIN voltage while the motors run   | During `u02`, put the multimeter on VIN while the wheels spin.                                                                                                                                              |

## 5. If something goes wrong

| Problem                                       | Do this                                                                                                                                                                         |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| "running scripts is disabled"                 | Use the exact command above, with `-ExecutionPolicy Bypass`.                                                                                                                    |
| "Cannot open COMx"                            | Close the VS Code Serial Monitor or PuTTY, then run again.                                                                                                                       |
| No `RPI-RP2` drive appears                    | Unplug. Hold BOOTSEL **before** plugging in, keep holding until the drive appears, then let go.                                                                                 |
| "did not come back as a serial port"          | Unplug and plug in again (without BOOTSEL), then run the test again.                                                                                                             |
| The build fails                               | Send me the lines it printed.                                                                                                                                                   |
| The robot drives off or misbehaves            | Switch the Robo Pico off with its power switch. Send me the log (it is saved up to that moment).                                                                                 |
| A step was measured wrongly                   | Finish the run, run the test again, and tell me which log to use.                                                                                                                |
| Uncommitted-changes warning                   | Commit first (or ask me), unless it is only a practice run.                                                                                                                      |
| Pressing Button 1 does nothing                | Press firmly and hold for half a second. Button 1 is the one labelled GP20 (not GP21, not RESET). If still nothing, send me the log.                                             |
| You started a step by mistake                 | Switch the robot off, run the test again, and tell me which log to use.                                                                                                          |

## 6. Why the robot curved and what keeps it straight

The first u11 run (2026-10-23) curved right by 38-85 mm over 1 m and went about 5 % too far, although the robot's own odometry said 999 mm and a heading change under 1 degree.

- The motion controller keeps both wheels at the same **counted** distance. That only drives straight if each wheel's distance per tick is right.
- The old values came from turning the wheels by hand (S2-U01): 63 mm and 639 ticks (left), 629 ticks (right). The floor showed both wheels really go further: a rolling diameter of about 66.5 mm, with 639 ticks on both sides. The right wheel's 629 made the firmware think the right wheel went 1.4 % further than it did, so it held the right wheel back and the robot turned right.
- The S2-U09 simulation reproduces this: a 1.4 % right-wheel error gives 59 mm to the right over 1 m with a heading reading near 0 (check H19).
- PID tuning cannot fix this; only calibration can. It is now one number per wheel, `S2_WHEEL_CIRC_UM_L/R`, and every u11 run checks it and prints corrected values. Re-run u11 if the tyres, wheels or floor change, or once a week before integration tests.