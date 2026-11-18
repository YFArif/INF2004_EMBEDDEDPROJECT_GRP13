# S3 handover for S5 (integration)

From Sean (S3, line + barcode) to Anna (S5, vehicle controller), 18 Nov 2026.
Based on `main` at `cbcc88c` plus `s3_barcode_reading()` (section 3).

**Short version:** S3's line following, junction detection and barcode
reading work in my S3 drive test (`task_drive_test.c`, car runs 5, 12 and
13 Nov). That file only runs with `S3_DRIVE_TEST 1`, so none of its
*behaviour* is in the integrated build. It drives the motors itself, which
only S5 may do in the mission. This note lists which parts S5 should copy
and which are already the same.

---

## 1. Already the same, nothing to do

| Item | Drive test | S5 now (`s5_links.c`) |
|---|---|---|
| Speed | 50 mm/s | `LINE_SPEED_MMS 50` |
| Gains | Kp 25, Kd 25, boost 250 | `LINE_KP_X100 25`, `LINE_KD_X100 25`, `LINE_BOOST_X100 250` |
| Full lock | 1 x speed | `LINE_STEER_PER_MMS 1` |
| Junction / lost / found | `flg_s3` events | same flags |
| Barcode command | `mbf_nav` | `s5_links_nav_take()` |
| Letter map | `s3_set_nav_map()` | now sent by S1's dashboard MAP message (Caleb, `telemetry_task.c`) |

Line following in the mission *is* S3's controller: S5 only enables it
(`LF_ENABLE`) and sets the values above. If the robot drives straight and
ignores the line, S3 was **not calibrated** when the run started (see 2.1).

---

## 2. What S5 should change (in `s5_mission.c`)

### 2.1 Calibration sweep (Button 2): use the heading-steered sweep

Now: `g_cal_sweep_deg = {45, -90, 45}`, fixed turns. On the car these
drifted ~1 cm and sometimes left a sensor short of black. (The comment above
it still says +-30 deg.)

Proven in the drive test (`auto_sweep()` / `sweep_turn_to()`, lines 284-349):

- Targets are **headings relative to the start**: `{+15, 0, -15, 0}` deg.
  The sensors are ~125 mm ahead of the axle, so 15 deg moves them ~33 mm:
  both cross the 18 mm line.
- After each turn: brake, let the wheels stop, read
  `subsystem2_odom_get().heading_cdeg`, and if more than **2 deg** (200 cdeg)
  off, turn by the difference. Up to **3 tries** per target. Wrap the error
  into +-180 deg.
- It ends where it started, centred on the line, ready for Button 1.

Sketch for S5's event-driven style:

```c
/* at Button 2 */  cal_start_cdeg = heading(); cal_step = 1; cal_tries = 1;
                   start_slow_turn(15, now);
/* on EV_MOTION_DONE */
s5_links_motion_settle();                       /* heading is final  */
err = cal_start_cdeg + target[cal_step-1]*100 - heading();   /* wrap +-18000 */
if ((|err| >= 200) && (cal_tries < 3))      { cal_tries++; start_slow_turn(err/100, now); }
else if (cal_step < 4) { err = ...target[cal_step]...; cal_step++; cal_tries = 1;
                         start_slow_turn(err/100, now); }
else                   { /* done: log s5_links_line_is_ready(), enter_ready() */ }
```

S5 needs one getter for the heading (like `s5_links_odom_dist_mm()`).

**Before Button 1:** `s5_links_line_is_ready()` only checks "not UNCAL". The
drive test also refuses to start until both sensors' black is at least 1800
raw (a half sweep was "valid" at 2099 and kept rising). With the 15 deg sweep
this is normally met. If runs still start blind, print the S3 status after
the sweep and tell me.

### 2.2 Junction turn: drive 125 mm forward first

Now: `execute_command()` stops and turns as soon as `EV_JUNCTION` arrives.
But the line sensors are **~125 mm ahead of the axle**, so the robot turns
before its wheels reach the crossing and misses the new line.

Proven: `DRIVE_JUNC_FWD_MM 125` (car 5 Nov: 60 mm too early, 150 mm 25 mm
past). For LEFT / RIGHT / U-TURN: **MOVE 125 mm, then TURN**, then settle,
then FOLLOW (`junction_turn()`, lines 1020-1063). STRAIGHT: just keep
following.

### 2.3 Barcode still being read at the junction: wait for it

The decoder publishes a letter only after **1.5 s of quiet** past the last
bar. If the junction comes soon after the barcode, `EV_JUNCTION` arrives
**before** the command: S5 goes straight, and the late command then acts at
the **next** junction (wrong turn).

New S3 call for this: `BOOL s3_barcode_reading(void)` in `s3_api.h` (any
task). Drive-test rule (lines 602-618): at a junction with no command
pending, if `s3_barcode_reading()` is TRUE, **stop and wait up to 2 s** for
the barcode event, then act on it at *this* junction.

### 2.4 Slow speed for recovery

`SLOW_MMS 40` is used for calibration and recovery. In the drive test,
recovery at 40 mm/s tripped S2's stall E-stop (5 Nov), so it uses
`RECOVERY_SPEED_MMS 60` (150 was too abrupt). Calibration at 40 is fine.
Suggest 60 for the recovery moves if you see stall E-stops there.

---

## 3. S3 interface (what S5 may call)

All in `app_program/subsystem3/s3_api.h`.

| Call / object | Use |
|---|---|
| `flg_line`: `LF_ENABLE`, `LF_SEARCH_L/R` | Turn steering on/off; reacquire from a side |
| `flg_s3`: `FLG_S3_JUNCTION`, `_LOST`, `_LINE_FOUND`, `_BARCODE` | Events, one per occurrence |
| `mbf_nav` (`s3_nav_msg_t`, 4 B) | Barcode command: LEFT / RIGHT / STRAIGHT / U-TURN. Letters not in the map never arrive |
| `s3_get_line_status()` | State (`S3_LINE_UNCAL` = not calibrated), counters, last letter |
| `s3_request_recalibrate()` | Forget levels before a sweep |
| `s3_set_line_gains()`, `s3_set_line_shape()`, `s3_set_steer_max()` | Tuning (S5 sets 25 / 25 / 250, 1 x speed) |
| `s3_set_nav_map("DCBA")` | Today's letters (S1 calls it from the dashboard) |
| **`s3_barcode_reading()`** (new) | A barcode is under the sensor now: wait before deciding at a junction |

**Buzzer:** GP22 belongs to S3 (`TEAM.md`): one 80 ms beep per decoded
barcode, mapped or not. For a fault beep, ask me for an `s3_beep()` call
rather than driving GP22 directly, so the two don't fight over the pin.

**Default map is `ABCD`** (A = LEFT, B = RIGHT, C = STRAIGHT, D = U-TURN). A
barcode with another letter (e.g. `F` on our practice track) is read and
beeps, but gives **no command**, so S5 does not stop at the junction.
Send the right map from the dashboard before the run.

---

## 4. How to check after the changes

1. Button 2: console shows the sweep ending within 2 deg of the start, then
   `CAL line sensors calibrated: YES`.
2. Send the map from the dashboard; console prints
   `S3 NAV MAP set: LEFT=... RIGHT=...`.
3. Run over a mapped barcode, then a junction: beep, `S3 NAV #n 'X' -> LEFT`,
   the robot drives ~125 mm past the crossing, turns 90 deg and finds the line.
4. Repeat with the junction right after the barcode (under ~1.5 s at 50
   mm/s, i.e. closer than ~75 mm): the robot stops, waits, then turns at
   **this** junction.

Please send me the serial log of any failing run (`Projects/log/COM4_*.txt`);
I can read S3's side from it.
