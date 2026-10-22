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
6. During floor and bench tests it asks for your measurement right after each step, and saves it in the log as a `[MEASURED]` line.
7. Stops by itself at the end of the S2 tests and prints where the log is: `testing\subsystem2\logs\<date>_s2_<test>.txt`.

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
- **Takes:** about 40 s.
- **What happens:** the LEFT wheel runs forward for 3 s, backward for 3 s, then forward faster for 3 s, with a 1 s pause after each. Then the RIGHT wheel does the same.
- **You do:** watch. After each wheel's first run the script asks two questions:
  1. Did that wheel turn the way that drives the robot **forward**?
  2. Did **only** that wheel turn?
- **Good result:** both `S2-U06` lines PASS and `y` to every question. If a wheel turned backward, do not do the floor tests; tell me and I flip that wheel in software.

### Test 3: `u02` speed controller (S2-U02). Wheels lifted.

- **Why:** first real check of the speed controller (NFR4: within 10 % of the target in 0.5 s).
- **Setup:** same as `u06`.
- **Takes:** about 10 s. Both wheels spin for 1.5 s.
- **You do:** nothing.
- **Good result:** `S2-U02 PASS`. The `[S2-U02]` lines are the speed every 20 ms, which I use to tune.

### Test 4: `u11` MOVE distance and straightness (S2-U11). On the floor.

- **Why:** everyone uses the robot's distance (Sean's junction and barcode distances, Kessler's hump height, Anna's obstacle depth). This test checks it against a tape measure.
- **Setup:**
  1. Lay the tape measure flat and straight on the floor. Tape it down at both ends and in the middle. The 0 mark is the start. Leave 1.5 m clear ahead and 0.6 m behind.
  2. Stick a small piece of masking tape on the robot's front, exactly in the middle, with a pen line on it. This is the **marker**.
  3. Place the robot so its centre line is over the tape (one wheel each side) and the marker is exactly on 0.
- **Takes:** about 4 minutes (5 runs, 30 s countdown each).
- **What happens:** runs 1-3 are MOVE 1000 mm at 300 mm/s, run 4 is MOVE 1000 mm at 150 mm/s, and run 5 is MOVE -300 mm (reverse).
- **You do, after each run** (the script asks):
  1. **Tape reading** at the marker, in mm (for example `1004`).
  2. **Sideways offset:** with the ruler, how far the marker is from the tape's centre line, in mm. Use `+` if the robot ended to its own **left** of the line (as seen from behind the robot), `-` if right. Use `0` if exactly on it.
  3. Put the robot back with the marker on 0.

  **Before run 5 (reverse):** place the marker on **500** instead. The script takes 500 as the start and works out the real distance.
- **Good result:** every `S2-U11` line PASS. The real (tape) distances tell me how to correct the encoder calibration. The sideways offsets show whether the robot drives straight.

### Test 5: `u03` turn accuracy (S2-U03). On the floor.

- **Why:** NFR5 needs 90-degree turns within 5 degrees and 180-degree turns within 8 degrees. Barcode turns and Anna's bypass depend on it.
- **Setup:**
  1. Make a `+` on the floor with two 40 cm strips of masking tape, crossing at exactly 90 degrees (lay the A4 corner in the crossing). Label the four arms F (forward), L (left), R (right), B (back).
  2. Tape the 30 cm ruler on top of the robot along its centre line, sticking out past the front. This is the **pointer**.
  3. Place the robot so the middle of its axle (halfway between the two wheels) is over the crossing and the pointer lies exactly along arm F.
- **Takes:** about 7 minutes (13 turns, 30 s countdown each).
- **What happens:** 5 turns of +90 (left), 5 of -90 (right), then 3 of 180.
- **You do, after each turn:**
  1. Put the protractor's centre on the crossing with its 0 line along the target arm: L after +90, R after -90, B after 180.
  2. Read the angle between the arm and the pointer, then type the **total angle turned**. For +90 that stopped 3 degrees short, type `87`. For 2 degrees too far, type `92`. For right turns type a negative number (`-88`). For 180, type for example `176` or `183`.
  3. If the robot slid a little off the crossing, read the pointer's **direction**, not where it is.
  4. Line it up again on arm F before the countdown ends.
- **Good result:** every `S2-U03` line PASS, and every number you typed within 85-95 (or -85 to -95) for 90-degree turns and 172-188 for 180. From your numbers I tune the stopping point and the track width.

### Test 6: `u12` braking, slow speeds, stall (S2-U12). On the floor.

- **Why:** Anna's 150 mm stop distance only works if the robot brakes from 300 mm/s within 58 mm (NFR6). Her slow mode needs 150 mm/s to hold steady. A stalled wheel must stop everything (FR15).
- **Setup:** 1 m clear in front. Put the robot back at the start during each countdown. Each step drives about 0.3-0.7 m.
- **Takes:** about 5 minutes.
- **What happens and what you do:**
  1. Drive 300 mm/s, then STOP. Did the wheels skid? (y/n)
  2. Drive 300 mm/s, then emergency stop. Did the wheels skid? (y/n)
  3. Drive 150 mm/s, then STOP. Did the wheels skid? (y/n)
  4. Drive at 40, 60, 100, then 150 mm/s for 2 s each. After each, did it move smoothly, without stopping or jerking? (y/n)
  5. **Stall:** it drives at 150 mm/s and prints `HOLD the LEFT wheel now`. Pick the robot up (the wheels keep turning slowly in the air) and pinch the **left** tyre firmly between finger and thumb so it stops. Within about 1 s the motors stop and `STALL` appears. Let go and put the robot down.
- **Good result:** every `S2-U12` line PASS: braking distance from 300 mm/s of at most 58 mm, 150 mm/s held within 10 %, stall detected. Give the braking distance to Anna too.

### Optional: `u01` encoder ticks per wheel revolution (S2-U01)

The calibration from 15 October is good enough, and `u11` checks it better. Do this only if I ask. Put a tape mark on each tyre. When `GO: turn the LEFT wheel` appears, turn the left wheel slowly by hand exactly 10 times (count the mark), keeping the right wheel still. Then do the same for the right wheel. Type how many turns you really made.

### PC only: `u09` simulation (S2-U09)

Runs `motion_task.c` against a simulated robot on the PC. No Pico is needed. Run it after any change to `motion_task.c`.

## 4. Chassis measurements (once, by hand)

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
