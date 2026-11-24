# S1 demo day: telemetry over MQTT

What Buddy 1 (S1) does on the day of the live car demo, and where the
dashboard shows each rubric item. Setup details are in
[S1_handover.md](S1_handover.md) (build, flash, session) and
[S1_dashboard.md](S1_dashboard.md) (every field on the page). The car
drives itself: the dashboard sends only START, STOP, RESET and the
barcode letter map, never steering (no remote control).

## 1. The rubric and the dashboard

S1's own point is "Send telemetry data via MQTT". The other nine points
come from S2 to S5; the dashboard is where the examiner can see each one
happen, so keep the page on screen for the whole run. The **Demo
checklist** card, below the live view, has one row per rubric point and
ticks each with its time as it happens; START clears it, so it shows the
current run only (S1_dashboard.md section 8.3). The panels below hold
the detail.

| Rubric item | Owner | Where it shows on the dashboard |
|---|---|---|
| Send telemetry data via MQTT | S1 | Robot light green, heartbeat age under 3 s, the summary cards and 3D view moving; "Telemetry / live lost" near 0 |
| Detect a barcode | S3 | S3 panel: a new barcode entry in the timeline; the Latest events line |
| Decode the barcode and obtain the code | S3 | S3 panel: the last barcode letter and its command (LEFT, RIGHT, STRAIGHT, U-TURN) |
| Measure hump height | S4 | Front page, Attitude card: Hump height in mm (S4's run peak), the hump state and Humps passed |
| Follow the line | S3, S2 | S3 line state FOLLOW; S2 motion FOLLOWING; the measured speed |
| Follow the instruction from the barcode | S3, S5, S2 | S3 junction turn; S5 state EXECUTING_COMMAND; the heading changes by the turn |
| Detect an obstacle | S5 | S5 panel: closest obstacle range; events `OBST_NEAR`, `OBST_STOP` |
| Measure obstacle width | S5 | S5 panel: the scan's gap width and clearances; event `PROFILE <width> cm` |
| Navigate around the obstacle and rejoin the line | S5, S2, S3 | S5 state OBSTACLE_AVOIDANCE, then REACQUIRE_LINE, then LINE_FOLLOWING; `AVOID_STEP` events |
| Stop motors when the car is picked up | S4, S2 | S4 panel: the red IMPACT box and motion class LIFTED; S2 motion ESTOP, speed 0 |

The events (`OBST_NEAR`, `PROFILE`, `AVOID_STEP` and the others) reach the
dashboard only on an image with S5's two-file change applied
(docs/MQTT.md section 1.2). Without it the same facts still show in the S5
panel, from the telemetry frame.

## 2. The day before

1. Build from `main` with the event change merged (or S5's patch applied),
   for the laptop that runs the broker on the demo hotspot:
   `make NET=lwip MQTT=1 -j8` (S1_handover.md section 2). Keep the UF2
   private: it holds the WiFi password and the broker address.
2. Flash it, then run one full session on the stand
   ([S1_handover.md](S1_handover.md) section 3) and check:
   - the robot light turns green within about 20 s of power-on;
   - rssi shows a dBm value, not "not measured";
   - START is acked "queued", and with S5's change the log shows
     `COMMAND START accepted by S5`;
   - STOP stops the wheels; the Escape key sends STOP too;
   - lifting the car shows IMPACT/LIFTED and the wheels stop.
3. Charge the batteries and the phone. Write down the laptop's hotspot
   address: if it changes on the day, the image must be rebuilt.

## 3. On the day

1. Phone hotspot on (2.4 GHz, Maximize Compatibility on an iPhone), only
   the laptop and the car on it. Join the laptop first.
2. Check the laptop's address has not changed:
   `python3 -I -B testing/subsystem1/s1_launch.py --show-address`.
3. Start the session: `python3 -I -B testing/subsystem1/s1_launch.py`
   (Windows: `testing\subsystem1\run_dashboard.bat`).
4. Car on, wait for the green robot light.
5. Type the day's letters in the map boxes, press **Set map**, and wait for
   "Applied on the robot: ...". Set it again after every power cycle; the
   dashboard also re-sends it on its own when the car comes back.
6. Car on the start line, press **START**, hands off.
7. During the run, point at the panels in section 1 as each item happens.
8. At the end the examiner lifts the car: IMPACT/LIFTED shows and the
   wheels stop. That ends the demo.

## 4. If something goes wrong

| What you see | What it means | What to do |
|---|---|---|
| Robot light red or grey for over 30 s | The car is not on the broker | Check the hotspot screen shows 2 devices. The car keeps driving without MQTT; S1 never blocks the mission. Show the USB console (`[S5]`, `S3` lines) as the evidence instead |
| Console loops status 1 then -3 | The hotspot refuses the car | Restart the hotspot with only the laptop and the car on it |
| START greyed out | No heartbeat yet, or the last START not acked | Wait for the green light; the reason is written under the button |
| START acked "queued" but the car does not move | S5 refused it (calibrating, or a fault) | Read the S5 panel's state and fault bits; RESET, then START |
| Map says NOT CONFIRMED | No ack after 3 tries | It is sent again when the car comes back online; press Set map again |
| Telemetry panels grey | No telemetry for 2 s | WiFi trouble; the run goes on, the panels come back with the link |

STOP is always available on the dashboard and by Escape, but the car's own
stop paths (obstacle E-stop, lift detection) do not depend on WiFi.

## 5. What is not proven on the car yet

Be ready to say so if asked:

- The STOP ack time (NFR15): S1-U06 on 10 Nov 2026 measured 62 % of STOPs
  acked under 300 ms (FAIL). The STOP still reaches S5; only the ack can be
  late.
- The event channel and RSSI (S1-U30): built and host-tested, not run on
  the car yet.
- The telemetry source mapping (S1-U17): not run yet.
