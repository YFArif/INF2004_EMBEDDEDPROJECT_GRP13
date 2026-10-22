# Subsystem 5: how to run each test (guide for Arif)

Thank you for running these while the car is with you. One script, `s5_test.ps1`, does everything: it builds, flashes, logs, and asks you for the few measurements it needs at the right moment. It works exactly like your `s2_test.ps1`, including the Button 1 steps. **At the end of each test, send Anna the log file it names.** That file holds everything she needs.

## 0. Before the first test

**Get Anna's code (once).** In a VS Code terminal (Terminal > New Terminal, the PowerShell kind) in the project folder `INF2004_EMBEDDEDPROJECT_GRP13`:

```powershell
git fetch
git checkout buddy5-ultrasonic
git pull
```

This branch already includes your `buddy2-motion` work, merged on 2026-10-22. If `git checkout` complains about your own uncommitted changes, commit or stash them first.

**Check the S5 hardware (once):**

| Part | Where | Check |
|---|---|---|
| Ultrasonic sensor (HC-SR04+, the 3.3 V one) | Grove port 2 | White wire (GP2) to **Trig**, yellow wire (GP3) to **Echo**, red to VCC, black to GND. Sensor level, not tilted up or down. |
| Servo (SG90) | Servo port 1 (GP12) | Plugged in; the sensor turns with it. |
| Power | Robo Pico | Power bank on VIN **and** the USB cable, as for your tests. Power switch ON. |

**Tools:** the same as for your floor tests: tape measure, ruler, protractor, masking tape and pen, a box to lift the robot, your phone (to film one test), and **a box to be the obstacle** (a shoe box or a carton about 15 cm wide and 10-15 cm tall, with a flat front).

**How to start a test** (same as yours):

```powershell
powershell -ExecutionPolicy Bypass -File testing\subsystem5\s5_test.ps1 -Test auto
```

Only the word after `-Test` changes: `auto`, `bench`, `stop` or `avoid`. Answer its questions, then follow what it prints. **Close the VS Code Serial Monitor and PuTTY first.**

Each test image also runs everyone's automatic checks first (including your S2 unit checks; the motors stay still), so S5's part starts about 30 s after flashing, at the line `TEST S5 tests start`.

## 1. The tests, in this order

### Test 1: `auto`. The robot can stand anywhere; the motors stay still.

- **Why:** checks Anna's maths, scan logic, avoidance planner, and that her two tasks start and obey requests.
- **Setup:** none.
- **Takes:** about 1 minute after flashing.
- **You do:** nothing. The servo turns to straight ahead once.
- **Good result:** `Done: 7 passed, 0 failed` (S5-U06a, b, c, U09, U10, U07a, b).

### Test 2: `bench`. Robot on a box, wheels free.

- **Why:** checks the sensor's distance, the width measurement, and the servo angles.
- **Setup:**
  1. Lift the robot onto a box so the wheels hang free.
  2. Put the obstacle box **about 30 cm** straight in front of the ultrasonic sensor, with its flat face square to the sensor. Nothing else within 1 m in front.
  3. Have the ruler, the protractor and your phone ready.
- **Takes:** about 2 minutes. No button presses.
- **What happens, and what you do:**
  1. **S5-U01:** the sensor pings for 2 s. The script asks for the **ruler distance** from the front of the sensor (the two silver cans) to the box, in mm.
  2. **S5-U04:** the servo sweeps right to left, then small steps at each edge of the box. The script asks for the **real width** of the box's front face, in mm.
  3. **S5-U03:** the servo stops at 5 angles (30, 60, 90, 120, 150 degrees) for 2 s each. **Film this from above.** Afterwards the script asks:
     - did the first stop point to the robot's **right** and the last to its **left**?
     - did the middle one point straight ahead?
     - what was the **largest angle error** you saw? (From the video: compare each stop with 30/60/90/120/150; 90 = straight ahead, 0 = straight right.)
- **Good result:** every S5 line PASS. Anna uses your numbers to tune the sensor and the servo.

### Test 3: `stop`. On the floor: drive at a box, stop, look. It does **not** drive round it.

- **Why:** the first real test of Anna's obstacle stop with your `motion_task` (FR9, NFR6). The robot must slow down near the box, stop before touching it, then scan it and decide a side.
- **Setup:**
  1. A clear strip **1.5 m long and 0.6 m wide**. The obstacle box at the far end.
  2. Put a tape mark on the floor **1 m** in front of the box: every run starts with the robot's front on that mark, pointing straight at the middle of the box.
  3. Keep the USB cable slack and behind the robot.
- **Takes:** about 5 minutes (3 runs, each started by Button 1).
- **What happens:** after `GO` the robot drives straight at the box at 300 mm/s (no line following yet). It slows to 150 mm/s at about 35 cm, stops at about 15 cm, then the servo sweeps and it prints its plan.
- **You do, after each run** (the script asks):
  1. **The gap** between the robot's front and the box, in mm.
  2. Did it **slow down** before stopping? (y/n)
  3. Did it **touch** the box? (y/n)
  4. It tells you which side it chose. Is that the side with **more free space**? (y/n)
  5. Put the robot back on the mark, then press Button 1 for the next run.
- **If it does not stop:** switch the Robo Pico off at once. After 30 s the test also stops the robot by itself. Send the log.
- **Good result:** 3 x `S5-U08 PASS`, a gap of at least 50 mm every time (NFR6), no touching.

### Test 4: `avoid`. On the floor: drive round the box. Do this only after `stop` worked.

- **Why:** the full bypass (FR11): stop, scan, drive round on the side with more space, come back to the line, carry on.
- **Setup:**
  1. A clear area **2 m long and 1 m wide**. The obstacle box in the middle of the far half, with **at least 0.5 m free on both sides**.
  2. Lay a strip of masking tape on the floor from the start mark straight through the middle of the box. This is the "line" (the robot cannot see it yet; it just shows you where it should come back to).
  3. Every run: robot on the tape, its front 1 m before the box, pointing along the tape.
- **Takes:** about 8 minutes (3 runs, each started by Button 1).
- **What happens:** after `GO` it drives at the box and stops. It scans, turns 45 degrees away, drives out, turns parallel, and drives past the box while the sensor watches the box's side. Then it turns back in, returns to the tape and turns to point along it. It drives on for 1.5 s and stops.
- **You do, after each run** (the script asks):
  1. Did it **touch** the box? (y/n)
  2. **Sideways distance** from the robot's centre to the tape, in mm (`+` if the robot is left of the tape, `-` if right).
  3. Is it **pointing along the tape** again (within about 10 degrees)? (y/n)
  4. Roughly the **closest** it came to the box's side, in mm.
  5. Put it back at the start, then press Button 1 for the next run.
- **Good result:** 3 x `S5-U05 PASS`, never touching, back within about 5 cm of the tape.

## 2. If something goes wrong

| Problem | Do this |
|---|---|
| "running scripts is disabled" | Use the exact command above, with `-ExecutionPolicy Bypass`. |
| "Cannot open COMx" | Close the VS Code Serial Monitor or PuTTY, then run again. |
| No `RPI-RP2` drive appears | Unplug. Hold BOOTSEL **before** plugging in, keep holding until the drive appears. |
| The build fails | Send Anna the lines it printed. |
| `[S5] PING ... range=  -- mm` on almost every line in `bench` | The sensor hears no echo. Check the box is square to the sensor and the wires (Section 0). Send the log anyway. |
| The robot drives off or misbehaves | Switch the Robo Pico off with its power switch. The log is saved up to that moment; send it. |
| A step was measured wrongly | Finish the run, run the test again, and tell Anna which log to use. |
| Uncommitted-changes warning | Answer `y` for a practice run, and tell Anna. |

## 3. Good to know

- In the **normal** firmware (plain `make`), Anna's controller is live: **Button 1 starts a run** (the robot drives straight, because there is no line following yet) and **Button 1 again stops it**. Holding Button 1 for 2 s resets after a fault. Use the test images above instead unless Anna asks.
- Lines starting `[S5] EVT 0x..` are Anna's event log. `[S5] LINK S3 stand-in` lines are normal: they show where Sean's line following will plug in.
- Anna's measurements wanted from you (once, if you have them): the robot's outside width (widest point, including the wheels) and its length, and where the ultrasonic sensor sits (distance from the robot's front edge).
