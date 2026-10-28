# S2 motors and S3 sensor integration reference

Prepared for Arif (Buddy 2) and Sean (Buddy 3), 2026-10-28.
Code baseline: `main`, commit `a9707e4` (S3 v20, retaining S2 `4690bd9`),
plus the uncommitted v21 S3 tidy-up described in section 11.
Updated after Sean's 28 October merge. This document describes the source
at that commit; it is not a claim that all physical behaviours are verified.

## 1. Read this first

- S2 owns motor PWM, encoder counting, wheel-speed control and motion execution.
- S3 owns line/barcode sensing and calculates the line-following steering request.
- The vehicle controller decides when to follow, stop, reverse or turn.
- A finite `TURN` and steering around a curve in `FOLLOW` use different paths.
  The latest right-TURN calibration is **not a demonstrated fix for right curves**.
- The right-TURN change is built and simulation-tested, but has **not yet been
  tested on the floor**. Line following is not established as stable.

## 2. Source map

Paths are relative to this document.

| Source                                                                                                             | Responsibility                                                                         |
| ------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------- |
| [subsystem2.h](../../app_program/subsystem2/subsystem2.h)                                                          | Public commands, signs, speed limits, odometry/status types and geometry               |
| [subsystem2.c](../../app_program/subsystem2/subsystem2.c)                                                          | Copied command buffer, S3 adapter, freshness, odometry callback, shared IRQ attachment |
| [motion_task.c](../../app_program/subsystem2/motion_task.c)                                                        | 20 ms control loop, state machine, wheel PI, finite travel, stopping and faults        |
| [drv_motor.c](../../app_program/subsystem2/drv_motor.c) / [header](../../app_program/subsystem2/drv_motor.h)       | Motor direction inversion and PWM                                                      |
| [drv_encoder.c](../../app_program/subsystem2/drv_encoder.c) / [header](../../app_program/subsystem2/drv_encoder.h) | Encoder edges, timing and shared GPIO ISR                                              |
| [s3_api.h](../../app_program/subsystem3/s3_api.h)                                                                  | S3 steering, navigation, flags and odometry interfaces                                 |
| [s3_config.h](../../app_program/subsystem3/s3_config.h) | S3 pins-independent settings, priorities, edge and demo switches |
| [task_line.c](../../app_program/subsystem3/task_line.c) / [s3_api.c](../../app_program/subsystem3/s3_api.c) | Line task, steering publication, odometry and public API |
| [drv_adc.c](../../app_program/subsystem3/drv_adc.c) / [drv_barcode.c](../../app_program/subsystem3/drv_barcode.c) | ADC owner, both line IR sensors (4-sample average per 10 ms reading, v21) and shared barcode GPIO handler |
| [task_drive_test.c](../../app_program/subsystem3/task_drive_test.c) | Optional button-operated FOLLOW demo |
| [s3_line.c](../../app_program/subsystem3/s3_line.c)                                                                | Line calibration, PD steering, edge following, junction/lost/search logic              |
| [S3 integration guide](../../app_program/subsystem3/README_S3.md)                                                  | Wiring, sensor positioning and drive-test instructions                                 |

## 3. Hardware, wiring and physical reference

| Connection            | Pins        | Detail                                |
| --------------------- | ----------- | ------------------------------------- |
| Left motor M1         | GP8 / GP9   | PWM slice 4, channels A/B             |
| Right motor M2        | GP10 / GP11 | PWM slice 5, channels A/B             |
| Left encoder phase A  | GP16        | Debounced rising edges counted        |
| Right encoder phase A | GP17        | Debounced rising edges counted        |
| Barcode DOUT / AOUT | GP7 / GP28 | S3 GPIO edge input / ADC2; rewired 28 October |
| Centre line AOUT      | GP27        | ADC1                                  |
| Outer line AOUT | GP26 | ADC0; physically on the robot's RIGHT; rewired 28 October |
| Test button 1 / 2     | GP20 / GP21 | Test use must be coordinated with S5  |

The front ball is a passive caster, not the drive axle. The motion reference is
the midpoint of the imaginary line joining the two driven wheel centres.
The physical track setting is 112 mm, tyre centre to tyre centre.

The left tyre's consistent floor-contact reference is suitable for the straight
U11 measurement. During turns/curves the left wheel follows a different path
from the axle midpoint. Front/rear overhang and sensor offsets are separate
geometry: do not add them to every MOVE. Axle-to-sensor and chassis-clearance
measurements still need recording for junction placement and obstacle clearance.

Do not infer connector wire colours, motor voltage rating or encoder electrical
output type from this document. Those have not been independently established.

## 4. Direction conventions: the important distinction

LEFT/RIGHT always mean the robot's own sides. Viewed from above:

- Positive body heading = anticlockwise = nose turns LEFT.
- Negative body heading = clockwise = nose turns RIGHT.
- Positive wheel speed/duty = that wheel drives the robot forward.
- Negative wheel speed/duty = that wheel drives backward.
- Positive MOVE distance = forward; negative MOVE distance = reverse.
- Positive TURN angle = left; negative TURN angle = right.
- S2 positive steering correction = left.
- **S3 positive `dv` = right**, the opposite sign from S2.

| Motion              | Left wheel     | Right wheel    |
| ------------------- | -------------- | -------------- |
| Straight forward    | Forward        | Forward        |
| Straight reverse    | Backward       | Backward       |
| Spin LEFT in place  | Backward       | Forward        |
| Spin RIGHT in place | Forward        | Backward       |
| Forward curve LEFT  | Slower forward | Faster forward |
| Forward curve RIGHT | Faster forward | Slower forward |

S2 already compensates for mirrored motor mounting:
`DRV_MOTOR_INVERT_LEFT = 0`, `DRV_MOTOR_INVERT_RIGHT = 1`.
Callers must **not invert the right motor again**.

## 5. Public command examples: use these, not PWM, in integrated code

Include `../subsystem2/subsystem2.h` from another subsystem.
The examples below are separate alternatives, not a sequence to paste and run.
`CHECK` is the project's existing error-reporting macro.

```c
CHECK(subsystem2_move(300, 100u), "move_forward");
```

Drive straight forward 300 mm at 100 mm/s.

```c
CHECK(subsystem2_move(-300, 100u), "move_reverse");
```

Drive straight backward 300 mm at 100 mm/s. Both wheels reverse.

```c
CHECK(subsystem2_turn(90, 150u), "turn_left");
CHECK(subsystem2_turn(-90, 150u), "turn_right");
```

These are alternatives: LEFT or RIGHT 90 degrees, in place.
TURN speed is **each wheel's speed**, not the body's centre speed.
For a U-turn use `subsystem2_turn(180, 150u)`; `-180` requests a right U-turn,
but the supplied clean 180-degree measurements cover left U-turns only.

```c
CHECK(subsystem2_follow(100u), "follow_line");
CHECK(subsystem2_stop(), "stop_motion");
```

FOLLOW keeps running until replaced/stopped/faulted and uses S3 steering.
STOP is a separate command, not something to send immediately after FOLLOW
unless the intent is to cancel it. A new FOLLOW while already following changes
the base speed. Speed zero means **default speed**, not stop:
300 mm/s for MOVE/FOLLOW; 150 mm/s wheel speed for TURN.
Requests above 500 mm/s are clipped; TURN angles are clipped to +/-720 degrees.

### Ordering and completion

Commands are copied into an eight-command nonblocking message buffer.
`E_OK` means accepted into the buffer, **not completed**.
Queue saturation can return `E_TMOUT`; do not ignore command return values.

The buffer is not a trajectory executor: the motion task drains pending commands,
and new MOVE/TURN/FOLLOW commands replace the running command.
To do "reverse, then left turn", send MOVE -distance, wait for S5's
`EV_MOTION_DONE`, then send TURN +angle. Do not send both together and expect
the first to finish. A completion counter is also exposed in S2 status.
The vehicle controller owns mission sequencing/event consumption.

### What "left reverse" and "right reverse" can mean

1. **Left wheel backward:** this is one wheel's direction, not a whole-car turn.
2. **Reverse straight, then turn left/right:** use MOVE negative, completion,
   then TURN positive/negative.
3. **A curved reversing manoeuvre:** there is currently **no public S2 command**
   for independent signed wheel-speed set-points or reverse FOLLOW.
   FOLLOW speed is unsigned; passing a negative value is not a reverse command.

For interpretation only, wheel speeds `left=-100, right=-50` produce backward
travel with positive (left) body yaw; `left=-50, right=-100` produce backward
travel with negative (right) body yaw. The rear's path is not named the same way
as the nose's yaw. Define the desired body heading and path before adding such
an API; do not bypass motion_task to implement it in Sean's sensor task.

## 6. Low-level motor behaviour (bench tests only)

`drv_motor_set(DRV_MOTOR_LEFT, -300)` means left wheel backward at 30% duty;
`drv_motor_set(DRV_MOTOR_RIGHT, -300)` means right wheel backward at 30% duty.
Positive 300 means forward at 30% duty. **Duty is not speed in mm/s.**

The driver accepts -1000 to +1000 permille and clips larger magnitudes.
The closed-loop controller limits its output to +/-900 permille.
Zero brakes; `drv_motor_brake()` brakes both; `drv_motor_coast()` releases both.
The source's Robo Pico truth table is both inputs low = brake, both high = coast;
forward/reverse electrical polarity is adjusted by the inversion settings above.

PWM is 20 kHz assuming the configured 125 MHz system clock. Initialisation sets
only motor slices 4/5 and releases PWM reset if necessary; it never reasserts the
whole PWM reset, preserving other subsystems' PWM configuration.

Only motion_task owns motor output in integrated operation. Raw driver calls are
for controlled S2 bench tests while motion_task is idle, not concurrent S3 use.

## 7. How the closed loop and encoders work

Motion runs at priority 9 on core 1 every 20 ms. Each cycle measures wheel
motion, processes commands, calculates set-points, drives the motors and
publishes odometry/status. MF_ESTOP is checked before normal command execution
and can wake the task without waiting for the normal tick.

- Set-point ramp: 30 mm/s per cycle (nominal 1.5 m/s^2).
- Feed-forward: 1.25 permille per requested mm/s.
- Wheel P gain: 1; I gain: 0.2 per cycle; D gain: 0.
- Integral contribution bounded to 400 permille, with anti-windup and no
  integration while the set-point is ramping.
- Finite MOVE/TURN uses wheel-distance progress, left/right synchronisation,
  slowdown near the end and a nominal 40 ms speed-based braking lead.
- Stall criterion: duty magnitude at least 400 permille and no ticks for
  15 consecutive control cycles (300 ms). This is a specific detection
  threshold, not a guarantee of detecting every mechanical obstruction.

Encoder counters include debounced ticks, last tick time, raw rise/fall counts,
rejected glitches and minimum raw edge gap. Accepted transitions must satisfy
the 100 us edge filter; rising edges increment travel ticks.
Timing uses the 1 MHz RP2040 timer; no tick for 100 ms is treated as stopped.

Only one encoder phase per wheel is wired into this code. Actual direction is
not sensed: it is inferred, and changes with duty sign only when measured wheel
speed is zero, avoiding counting reverse braking duty as actual reverse travel.
This remains a limitation during rapid reversals or externally pushed wheels.

Current conversion constants:

| Setting                           | Value                           |
| --------------------------------- | ------------------------------- |
| Left / right ticks per revolution | 639 / 639                       |
| Left rolling circumference        | 208620 micrometres              |
| Right rolling circumference       | 209010 micrometres              |
| Physical track                    | 112 mm                          |
| Effective LEFT spin track         | 118 mm                          |
| Effective RIGHT spin track        | 117 mm, hardware retest pending |

Floor calibration establishes distance per tick; it does not independently prove
the true circumference and encoder resolution separately.
Wheel travel is signed ticks times circumference divided by ticks per revolution.
Odometry provides both wheel distances/speeds, centre distance, counts and heading.
Heading is wheel-derived, wraps to -17999..18000 centidegrees and uses directional
spin scales when both wheels move in opposite directions.

Encoder rotation does not prove body rotation: wheel slip, cable drag and caster
resistance require physical angle measurements. More encoder logs alone cannot
eliminate these errors.

## 8. Exact S3-to-S2 steering path

1. S3 reads both line sensors every 10 ms (each reading averages 4 ADC
   samples taken 2.5 ms apart, v21) and calculates `o.steer` in permille.
2. `task_line.c` uses `s3_line_dv()` to convert it to a wheel-speed difference.
   While tracking: `dv = steer * steer_max / 1000`.
   OFF LINE / LOST multiplies that by 5/4, with a final +/-511 clamp.
   The initial steer maximum is 200 mm/s; `s3_set_steer_max()` changes it.
   The v20 drive demo sets it to the base speed, 80 mm/s.
3. S3 publishes it atomically with a sequence number and validity.
4. In FOLLOW, S2 reads `s3_get_steer_sp(&dv, &seq)` every 20 ms.
5. S2 converts to its own per-wheel correction:
   `s2_correction = -(dv * 50 / 100)` using integer arithmetic.
6. S2 requests `left = base - s2_correction`,
   `right = base + s2_correction`, each clipped to +/-500 mm/s.
7. Wheel ramping and PI control then produce actual PWM.

| Base speed | S3 dv | Left target | Right target | Intended effect         |
| ---------- | ----- | ----------- | ------------ | ----------------------- |
| 100        | 0     | 100         | 100          | Straight                |
| 100        | +100  | 150         | 50           | Right curve             |
| 100        | -100  | 50          | 150          | Left curve              |
| 100        | +200  | 200         | 0            | Right pivot-like target |
| 80         | +200  | 180         | -20          | Inner wheel reverses    |

These examples show the adapter arithmetic, not v20 demo presets. They are
pre-ramp set-points, not guaranteed measured speeds. At 100 mm/s,
full steering gives a stationary inside-wheel target, not a symmetric spin.
At 150 mm/s, full steering gives 250/50 mm/s, so it cannot make the same
tight curve as at a lower base speed. With the actual v20 demo at base 80 and
steer maximum 80, full tracking requests 120/40 mm/s; OFF LINE / LOST requests
130/30 mm/s (right correction). Left corrections swap the targets. Neither
requires an inner-wheel reversal. Right-spin track calibration is used
when wheels oppose, not for ordinary forward/forward line-following curves.

### Freshness and failure behaviour

- Sequence must keep changing. An unchanged correction can be valid if its
  sequence advances. An unchanged sequence for over 100 ms is stale.
- Invalid/stale S3 steering produces **zero correction, not a motor stop**.
  FOLLOW can continue straight at its base speed. A controller must explicitly
  decide when to stop for lost/invalid sensing.
- A fresh `subsystem2_steer_set()` overrides S3 for 100 ms. That API is for S2
  tests; Sean should not use it as a second competing steering publisher.
- S3 currently publishes valid steering in FOLLOW, JUNCTION and LOST; LOST
  can therefore keep recovery steering active rather than automatically stop.
- MOVE and TURN do not consume S3 steering.

## 9. S3 integration resources and restrictions

- S2 registers `subsystem2_odom_total_mm` through `s3_set_odometry()` at startup.
  S3 line-distance logic uses forward differences; reverse/reset steps are
  ignored. Barcode navigation odometry is a separate 16-bit wrapping field.
- GPIO IRQ13 has one owner: S2. S3 already attaches its barcode handler using
  `subsystem2_gpio_isr_register()`. Do not apply the old IRQ patch again or
  register a second IRQ13 handler. The API has one hook slot, not a registry
  for arbitrarily many additional GPIO users.
- Encoders and attached barcode processing run on core 1 at interrupt level 2;
  S3's separate timer-alarm-1 sampler also uses level 2.
- S3 is the sole ADC sampler. Do not add a competing ADC reader.
- **v25 (local, 28 Oct): both line sensors sit ON the line, 12 mm apart
  centre to centre**, GP27 6 mm left and GP26 6 mm right of the car's
  centre line (`S3_IR_L_SIDE` in `s3_config.h` if mounted the other way).
  S3 steers on (right darkness - left darkness); the old straddle /
  edge-riding layout and its 20 mm spacing no longer apply. Junction
  detection is off in this layout. See `README_S3.md` v25.
- The old `S3_TEST` desk modes and `s3_bringup.c` were removed.
  `S3_DRIVE_TEST = 0` in `s3_config.h` on main means the
  button-operated drive demo is off; starting sensor tasks alone does not
  start FOLLOW.
- When enabled deliberately, Sean's v20 drive demo defaults to 80 mm/s, waits for
  calibration/line placement and GP20, then gives a three-second countdown.
  GP21 changes gain presets (default Kp 250 / Kd 50, x100). Its limits include
  3 m, 30 s, 300 mm or 2 s of persistent LOST, and 2 s without odometry change.
  It refuses to start while S2 E-stop is latched. These are demo policies,
  not universal motion-layer safety rules.

## 10. Stop, faults and restart

`subsystem2_stop()` queues a normal stop that cancels the active motion.
It can fail if the command buffer is full and can be superseded by later
commands; it is not the latched emergency-stop path.

`subsystem2_estop_set()` sets MF_ESTOP. S2 brakes and remains in ESTOP.
A stall also sets the S5 stall fault and signals EV_FAULT.
For an authorised restart, the vehicle controller first establishes it is safe,
calls `subsystem2_estop_clear()`, then checks `subsystem2_clear()` succeeds.
Clearing the flag alone does not restart motion. S3 must not clear an E-stop
just to make line-following tests run.

## 11. Branch updates and evidence

| Commit    | Relevant change                                                                               |
| --------- | --------------------------------------------------------------------------------------------- |
| `b9f5b8c` | S3 merged into main alongside S2, including shared GPIO integration                           |
| `195ddbd` | S3 difference-to-per-wheel conversion changed from 100% to 50%                                |
| `d0ecff7` | Sean's v15 guide, edge/sensor spacing and drive-test context                                  |
| `4690bd9` | Right finite-TURN scale 110 -> 117 mm; U03 prompts, physical results, 180-only rerun and logs |
| `a9707e4` | Sean v20 merge: split drivers/tasks, straddle sensing, revised pins, speed-scaled demo steering and E-stop respect; S2 source unchanged |
| v21 (uncommitted) | S3 tidy-up with Sean's consent: 11 small files merged into their only users (36 -> 23 files in the S3 folder), 4-sample line IR averaging; S2 source unchanged |

At `4690bd9`, normal firmware compiled with zero warnings/errors and all 33
motion host checks passed. The host model uses calibration constants too;
passing simulation is not an independent hardware angle measurement.

Physical evidence before the latest right adjustment:

| Test                           | Results                    | Interpretation              |
| ------------------------------ | -------------------------- | --------------------------- |
| LEFT 90                        | 90, 90, 90, 90, 88 degrees | All within +/-5 degrees     |
| RIGHT 90 with old 110 mm scale | 82, 87, 85, 84, 85 degrees | Mean 84.6; two outside +/-5 |
| LEFT 180, clean rerun          | 185, 185, 184 degrees      | All within +/-8             |

The new right scale is `110 * 90 / 84.6`, rounded to 117 mm.
Left remains 118 mm to preserve passing 90/180 results. Right -180 has not been
physically characterised by these logs. NFR5 requires ten trials each for
90/180 acceptance; the current 13-step calibration routine alone is insufficient.

See [TEST_LOG.md](TEST_LOG.md) for evidence and [TEST_GUIDE.md](TEST_GUIDE.md)
for physical procedures. Sean's v20 guide reports a v19 car run tracking 2.1 m
but shaking. The new v20 gains still need on-car validation; this is progress,
not proof of stable tracking or NFR5 compliance.

Local verification of `a9707e4`: combined normal firmware builds, all 33 S2
motion checks pass, and six S3 test programs (including track simulation) run
successfully. The simulation includes the 50% adapter and 20 ms ramp, but
not physical slip, stall or real wheel PI lag.

v21 adds optional wheel lag, 8-bit readings and oversampling to that
simulation (check S3-S91). Averaging 4 ADC samples per reading cut steering
sign flips from 26% to 17%. Letting S2 change the steering faster than its
30 mm/s per 20 ms ramp made no measurable difference (the steering seldom
changes faster than that at 80 mm/s), so the S2 ramp was **not** a cause of
the weaving and S2 was left unchanged. With v19 gains the simulation gives
Centre on edge 59% and sign flips 28%, close to the v19 car run (52% and
27%), so it is a fair guide; v20 and v21 still need car runs to confirm.
Motor behaviour at the inner wheel's low speed (S2-U12 slow-speed steps) is
not modelled and is the main untested candidate on the S2 side.

## 12. What Sean should record before changing gains

For right/left curves, collect the same timestamps for:

- Requested base speed, S3 `dv`, sequence and validity/state.
- Normalised Centre/outer readings, calibration state and edge selection.
- S2 target and measured left/right speeds, duties, heading and state.
- Surface, supply, cable slack, direction and actual path/video.

Useful distinctions:

- Wrong target sign: examine S3 edge logic and the adapter convention.
- Correct targets but measured wheels lag: examine load, ramping and motor loop.
- Steering saturated: compare base speed, runtime steer maximum and required
  curve tightness before increasing gains. The default maximum is 200 mm/s,
  but the v20 demo requests 80 and recovery uses 1.25 times that.
- Correct wheel speeds but wrong body path: inspect slip, caster, alignment,
  sensor placement and cable drag.
- Turning worsens after explicit test steering: check the 100 ms override.

Do not change the right-TURN constant to tune forward line following.
Do not flip right motor polarity to correct a sensor error.
Keep changes isolated and compare the same track at a low speed before increasing it.
