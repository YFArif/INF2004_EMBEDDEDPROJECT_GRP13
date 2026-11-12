# S1 laptop dashboard

`testing/subsystem1/s1_dashboard.py` is a one-file dashboard for the
robot: START, STOP and RESET, the ack for each command, an online light,
the 1 Hz heartbeat and four telemetry panels (S2 motion, S3 line and
barcodes, S4 motion sensor, S5 controller). It runs on the laptop that runs the broker,
serves one page on `http://127.0.0.1:8765/` (that laptop only), and talks
to the broker through `mosquitto_sub` and `mosquitto_pub`. Python
standard library only. Test: S1-U16 in `testing/subsystem1/TEST_LOG.md`.

> **First test with the wheels off the ground.** Put the robot on a stand
> so the wheels turn freely, and keep a hand near the GP20 button.

## 1. What it does, and what it does not

- Mission commands only: START, STOP and RESET. No steering, no speed:
  the robot drives itself.
- Commands are never published retained (section 11).
- An ack with result 0 is labelled **queued**: S1 handed the command to
  S5, which decides on its own. S5's verdict is on the robot's USB
  console only (`[S5] EVT 0x65 a=<opcode> b=<1 or 0>`) until the event
  channel is built. The S5 panel does show the state change.
- Telemetry (section 8.1) is shown, read-only. Events (EVT codes) are
  not sent yet, so the event log shows none. The heartbeat's rssi and
  evt_drop are always 0 for now; rssi 0 reads "not measured".
- The page, the log and the console never show the broker address, the
  WiFi details or a login.
- The page loads nothing from anywhere else. Its Content-Security-Policy
  header makes the browser block any such load, and stops other sites
  from showing the dashboard inside a frame.

## 2. What you need

| Item | Version | Notes |
|---|---|---|
| Python | 3.9 or later | Tested with 3.9 and 3.13. No packages to install. Windows: use `py -3` (section 15) |
| mosquitto (broker and clients) | 2.0 or later | Tested with 2.1.2. macOS: `brew install mosquitto`. Windows: section 15. Check with `mosquitto_sub --help \| head -2`. Not needed for `--simulate` (section 12) |
| This repository | branch `main` | It must contain `app_program/subsystem1/telemetry_task.c` (Stage 4a). Pull `main` if your clone does not have it |
| Pico SDK and ARM compiler | as in `README.md` section 1 | Only to build the robot image (section 5) |
| A phone hotspot | 2.4 GHz, WPA2 | Section 3 |

The retained-command banner needs `mosquitto_sub` with MQTT v5
(`-V mqttv5 --retain-as-published`). With an older client the dashboard
still runs, and the page says "banner unavailable".

## 3. The hotspot: 2.4 GHz, WPA2, private

The Pico W radio works on 2.4 GHz only, and the firmware joins with WPA2
(a WPA3-only network is refused: "authentication rejected").

- **iPhone:** Settings > Personal Hotspot: turn on "Allow Others to
  Join" and "Maximize Compatibility" (this gives 2.4 GHz).
- **Android:** Hotspot settings: band 2.4 GHz, security WPA2-Personal.

Use a password only you know: the broker accepts any client that can
reach it (section 6), so anyone on the hotspot could send commands.
Never run the broker on a shared or public network.

## 4. Your laptop's address on the hotspot

The robot connects to the broker on your laptop, so the image needs the
laptop's address on the hotspot. Join the hotspot with the laptop, then:

- **macOS:** `ipconfig getifaddr en0` (en0 is Wi-Fi on most Macs; if it
  prints nothing, `networksetup -listallhardwareports` names the Wi-Fi
  device), or System Settings > Wi-Fi > Details > TCP/IP.
- **Windows:** `ipconfig`, the "IPv4 Address" under "Wireless LAN
  adapter Wi-Fi".
- **Linux:** `ip -4 addr show`.

**The address can change** when the hotspot or the laptop reconnects:
the phone hands addresses out again. Check it before every session. If
it changed, put the new one in `mqtt_config.h`, rebuild and reflash
(section 5). The usual sign of a stale address: the robot light never
turns green. Do not paste the address into chats, issues or logs.

## 5. Build the robot image with your own settings

Both files below are git-ignored. Never commit them, and never share the
`.uf2` you build: it holds the WiFi password and the broker address.

### 5.1 `config/wifi_secrets.h`

```bash
cp config/wifi_secrets.example.h config/wifi_secrets.h
```

Change only the first two values:

```c
#define WIFI_SSID     "your-hotspot-name"
#define WIFI_PASSWORD "your-hotspot-password"
```

The `MQTT_*` lines further down are no longer read (the MQTT driver
takes its settings from `mqtt_config.h` since Stage 3); leave them as
they are.

### 5.2 `app_program/subsystem1/mqtt_config.h`

```bash
cp app_program/subsystem1/mqtt_config.example.h app_program/subsystem1/mqtt_config.h
```

Set these values (placeholders shown; `192.0.2.10` is a documentation
address, use your laptop's from section 4):

```c
#define MQTT_LINK_CFG_SET               1
#define MQTT_LINK_CFG_BROKER_HOST       "192.0.2.10"
#define MQTT_LINK_CFG_BROKER_PORT       1883U
#define MQTT_LINK_CFG_CLIENT_ID         "robocar-buddy1"
#define MQTT_LINK_CFG_TOPIC_PREFIX      "robot"
#define MQTT_LINK_CFG_USERNAME          ""
#define MQTT_LINK_CFG_PASSWORD          ""
```

- The build stops while `MQTT_LINK_CFG_SET` is 0.
- The client ID must be unique on the broker.
- The prefix is at most 32 characters, with no `#`, `+` or slash at
  either end. The dashboard reads the same file, so both use the same
  prefix.
- Leave the login empty: the broker in section 6 allows anonymous
  clients.

### 5.3 Build

From the repository root, with `PICO_SDK_PATH` set as in `README.md`
section 1:

```bash
cd build_make
```

```bash
make NET=lwip MQTT=1 -j8
```

The image is `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`. Do
not add `TEST=1`: test images ignore START, STOP and RESET.

### 5.4 Flash

1. On the car, switch the battery off first. Hold the **BOOTSEL** button
   and plug the Pico W into USB, then release. A drive called
   **RPI-RP2** appears.
2. Copy the image onto it. On macOS, from the repository root:

   ```bash
   cp build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2 /Volumes/RPI-RP2/
   ```

   On Windows or Linux, drag the file onto the RPI-RP2 drive.
3. The Pico restarts by itself. A "disk not ejected properly" notice is
   harmless.

To see S5's verdicts, read the robot's USB serial console. On macOS,
with the robot plugged in (Ctrl-C to stop):

```bash
cat /dev/cu.usbmodem*
```

`README.md` section 2.4 has `screen` and the Linux and Windows ports.
The console drops output nobody reads (everything printed before you
open it) and can drop bytes under load, so open it before powering the
robot, and take the robot's state from the S5 panel (telemetry
`ctrl_state`), not from the console.

## 6. Start the broker

Create `mosquitto.conf` anywhere outside the repository, for example
`~/s1-broker/mosquitto.conf`:

```
listener 1883
allow_anonymous true
persistence false
sys_interval 5
```

| Line | Why |
|---|---|
| `listener 1883` | Listen on every interface, so the robot can reach it over the hotspot |
| `allow_anonymous true` | No login, which is why the hotspot must be private |
| `persistence false` | Nothing survives a broker restart, a stray retained command included |
| `sys_interval 5` | `$SYS/broker/uptime` every 5 s: it keeps the dashboard's broker light green |

Start it, and leave the window open:

```bash
mosquitto -c ~/s1-broker/mosquitto.conf -v
```

- If `mosquitto` is not found, Homebrew put it in `/opt/homebrew/sbin`
  (Apple silicon) or `/usr/local/sbin`: use the full path.
- If macOS asks whether mosquitto may accept incoming connections, allow
  it: the robot connects in.
- Stop it with Ctrl-C when the session ends.

## 7. Start the dashboard

**One command** (`testing/subsystem1/s1_launch.py`, test S1-U23) starts
the broker of section 6 and the dashboard together, from the
repository root:

```bash
python3 -I -B testing/subsystem1/s1_launch.py
```

It finds mosquitto (the PATH, then the Homebrew folders or
`C:\Program Files\mosquitto`), creates `~/s1-broker/mosquitto.conf`
with the four lines of section 6 if it is missing (it never changes an
existing one), and refuses to start while port 1883 or 8765 already
answers. It then runs the broker and the dashboard with `--host
127.0.0.1 --prefix robot` and a new `--log` file in `~/s1-broker` for
every session, and opens the page. It warns if that log is still empty
15 s after the start and at exit. Ctrl-C stops the dashboard and the
broker. Options: `--prefix`, `--workdir`, `--http-port`,
`--no-browser`, `--simulate` (no broker), `--show-address` (prints the
laptop's address for section 4); anything after `--` goes to the
dashboard. Exit status: the dashboard's, or 2 (mosquitto is missing),
3 (the broker did not start), 4 (a port is in use). On Windows, see
section 15.

The manual way below is the fallback. From the repository root:

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py
```

It reads the broker host, port and prefix from
`app_program/subsystem1/mqtt_config.h`. On the broker laptop itself this
also works, keeps working when the hotspot address changes (the robot
still needs the current address in its image), and needs no
`mqtt_config.h` at all, so it also suits a laptop that has not built an
image:

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py --host 127.0.0.1 --prefix robot
```

Without `mqtt_config.h`, `--host` alone stops with "no topic prefix":
give `--prefix` too.

Then open http://127.0.0.1:8765/ in a browser on the same laptop. Ctrl-C
in the terminal quits.

Order for a session: hotspot on, laptop joined, broker started,
dashboard started, robot powered (wheels off the ground), wait for the
green light.

| Option | Default | What it does |
|---|---|---|
| `--config PATH` | `app_program/subsystem1/mqtt_config.h` | Where to read the broker settings |
| `--host`, `--port`, `--prefix` | from the file | Override one setting |
| `--http-port N` | 8765 | The page's port (still 127.0.0.1 only) |
| `--log FILE` | none | Append every event to FILE as JSON lines |
| `--ack-timeout S` | 2.0 | Seconds to wait for an ack |
| `--simulate` | off | No broker, no robot: a fake car answers (section 12) |
| `--banner-off` | off | Run without the retained-command banner |
| `--check` | off | Run test S1-U16 part A (simulate mode) |

## 8. What the page shows

| Item | Meaning |
|---|---|
| Broker light | Green: a message from the broker in the last 12 s. Red: none, or `mosquitto_sub` stopped |
| Robot light | Green: status `online` and a heartbeat within 3 s. Grey: no status yet, or online and waiting for the first heartbeat. Red: status `offline` (the broker's last will), or online but no heartbeat for more than 3 s |
| Heartbeat age | Seconds since the last heartbeat; red above 3 s |
| Uptime | The robot's time since power-up, h:mm:ss (16 bits: wraps after 18.2 h) |
| connect_count | Broker sessions since power-up; it rises by 1 at each reconnect |
| cmd_reject | Commands S1 refused: codec rejections, post failures (S5's queue full) and commands the driver dropped |
| rssi | "not measured": always 0 for now, because the robot cannot read the signal strength yet |
| tel_drop / evt_drop | tel_drop: telemetry, live and heartbeat messages the robot did not send since power-up, because its publish queue was too full (it keeps room for acks) or the driver refused them. It should stay 0; if it climbs, the link is too slow for 8 messages a second. It stops at 255, shown as "255 or more": from there the page cannot show new drops until the robot restarts. evt_drop: always 0 for now, no events yet |
| Heartbeats lost | Gaps in the heartbeat sequence since the dashboard started (heartbeats use QoS 0, so a few can go missing). A robot restart does not count |
| Telemetry / live lost | Gaps in each message's sequence since the dashboard started: messages the robot handed to the driver that never arrived. A gap of more than 25 is read as a restart and not counted |
| Red banner | A retained command is stored on `<prefix>/cmd/...` (section 11) |
| Amber banner | That retained command was cleared |
| "banner unavailable" (top bar) | The banner is off: `--banner-off`, an old `mosquitto_sub`, or a broker without MQTT v5 |

**Buttons.** STOP is always enabled; the Escape key also sends STOP.
START and RESET are enabled only while the robot light is green, and
START (or RESET) stays disabled until the ack of the previous one
arrives, so a double click cannot run the robot from FINISHED. An
enabled START or RESET is lit (white text, visible border); a disabled
one is dimmed and shows the reason underneath, for example "online,
waiting for a heartbeat". The buttons are decided by the robot light only, not by the S5
panel: a START sent while the robot calibrates is acked "queued" and
refused by S5 (section 9).

**Command results** (the "Last commands" table and the event log).
"queued" is shown in blue, not green: it means the command reached the
robot's controller, not that the controller acted on it.

| Result | Meaning |
|---|---|
| pending | Sent; waiting up to 2 s for the ack |
| queued | Ack result 0: S1 handed the command to S5. Not "done": S5 may still refuse it (section 9) |
| rejected | Ack result 1: S1's codec refused the command, or S5's queue was full |
| no ack | No ack within 2 s: the robot was offline, or the command or its ack was lost |
| queued (late), rejected (late) | The ack came after the 2 s limit |
| duplicate (log only) | A second copy of an ack; QoS 1 can repeat a message |
| publish failed | `mosquitto_pub` could not reach the broker |

Each ack is matched to its command by cmd_id, which counts 0 to 63 and
wraps. The round trip runs from just before `mosquitto_pub` starts to
the ack's arrival, both on the laptop's clock, so it includes
`mosquitto_pub`'s own start-up.

### 8.1 The telemetry panels

Two messages on `<prefix>/telemetry` feed them (layouts in
`docs/S1_binary_frame.md`): the 2 Hz telemetry frame (type 1, 25 bytes)
and the 5 Hz live message (type 6, 14 bytes). Each panel says how long
ago its data arrived and greys out after 2 s without any (robot off,
WiFi gone, or a panel's owner not sending); a page opened late shows
what the dashboard last saw. The dashboard only reads: nothing here
sends a command.

| Panel | Shows |
|---|---|
| S2 motion | Motion state (IDLE, FOLLOWING, MOVING, TURNING, ESTOP), red E-STOP and STALL tags, commanded speed (mean of the two wheel targets), measured speed (mean of the two wheels), left and right wheel speed, distance, heading from wheel odometry, fault bits |
| S3 line and barcodes | Line state (FOLLOW, JUNCTION, LOST, DISABLED, SEARCH, UNCAL) and the last barcode letter, outer and centre sensor bars (0 white to 255 black), steering, OUTER rule, the last barcode with its command and direction, the last junction turn, S3's own counters, a tally of accepted barcodes per letter, the rejected total, and a timeline with a time on each entry |
| S4 motion sensor | A large red IMPACT box (held for 3 s after the last flagged frame, so a short impact is not missed) and a count of impacts seen, pitch, roll, turn rate, motion class, hump state, hump count and peak height, IMU ok, and a pitch and roll graph of the last 60 s (scale +/-32 degrees) |
| S5 controller | Controller state (INIT to FINISHED), calibration, fault bits, closest obstacle, and the scan's gap width and clearances when a scan is valid |

How to read the S3 timeline and tally. S3 reports counters, not events,
so the dashboard turns a counter that moved into a timeline entry
(barcode, rejected, junction, line lost). Each entry is dated by when its
message arrived on the laptop, so the times are good to about 0.2 s. The
letter comes from S3's last-barcode field: if two barcodes land inside one
0.2 s message gap (or a message is lost), the extra ones are counted under
`?`. S3 sends only a count of rejected barcodes, not which letter was
rejected, so the rejected figure is one total, not per letter. If a counter
jumps by more than a believable amount for 0.2 s (rejected 3, the others 4),
or the live sequence jumps by more than 25, or the heartbeat's uptime goes
back, the dashboard assumes a restart or lost messages, writes a note in
the timeline and counts again from the next message. A burst of more than
that many real events inside one gap is therefore noted, not listed.

One case can still fool it: a restart when the rejected count was 13 to 15
(or another counter 252 to 255) and the live sequence was within 25 of its
own wrap. The counter going back to 0 then looks like a small rise, so one
wrong timeline entry can appear until the next heartbeat shows the restart.
A reboot during an outage longer than the previous run also shows no uptime
drop; the counters and the live sequence are then the only signs.

Fault bits are copied as S5 sets them (stall, encoder, IMU, no echo, WiFi);
the S2 and S5 panels show the same field. Heading is in degrees from the
wheel encoders, so it drifts and starts from 0 at power-up.

The `--log` file records every telemetry and live message as it arrives,
so it grows by roughly 20 MB an hour while the robot runs.

## 9. Operator rules

These are S5's rules (`s5_mission_command()` in
`app_program/subsystem5/s5_mission.c`):

- START runs the robot only from READY. In FINISHED, START only returns
  to READY; the next START runs.
- In READY, START is refused until S3's line sensors are calibrated: the
  console prints `RUN refused: line sensors not calibrated, press Button
  2 on the line`. Put the robot on the line and press Button 2 (GP21),
  releasing it within 2 s: it turns left, right and back on the spot and
  prints `CAL line sensors calibrated: YES` (or `NO`: press again). The
  calibration is lost at every power-up. On a stand the sweep returns NO,
  because the sensors see no floor (readings of 3700 or more are
  ignored).
- START is refused while running, while calibrating (up to 3 s after
  power-up, and during the Button 2 sweep) and in FAULT (send RESET
  first).
- STOP is always accepted: it ends a run in FINISHED, and does nothing
  when the robot is not running. The STOP button, the Escape key and the
  GP20 button all stop the robot.
- RESET works only from FAULT or FINISHED, back to READY with the motors
  braked.
- After a GP20 start the robot is running, so a dashboard START is
  refused; STOP still works. GP20 held for 2 s is RESET.
- `TEST=1` images ignore START, STOP and RESET until an S5 test step arms
  the controller.
- A command sent while the robot is offline is lost: it gets no ack and
  never runs. That includes a command sent during a reconnect. Resend
  it if there is no ack.

## 10. A session, step by step

1. Robot on a stand, wheels off the ground.
2. Hotspot, broker and dashboard as in sections 3, 6 and 7.
3. Power the robot. Wait for the green light and a heartbeat age under
   1.5 s.
4. Calibrate the line sensors (section 9). This needs the robot on the
   line: on the stand every START is refused (`b=0`).
5. START: "queued", and the wheels turn (the console shows `b=1`).
6. STOP: "queued", and the wheels stop (FINISHED).
7. START once more: "queued", but the wheels stay still (back to READY).
   START again to run.

## 11. Clearing a retained command

A retained message stays on the broker and goes to every client that
subscribes, again and again. The robot subscribes after every reconnect,
and lwIP does not tell it that a message was retained, so it would run a
retained command each time it reconnects. The dashboard never publishes
with retain; the red banner means another client did.

To clear it, switch the robot off first, then publish an empty retained
message on the topic the banner names, on the broker laptop:

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t 'robot/cmd/<suffix>' -r -n
```

Replace `robot` with your prefix and `<suffix>` with the end of the
topic. The banner turns amber ("cleared"). If the robot is online, it
receives the empty message as a command, refuses it (ack "rejected",
cmd_reject +1) and does nothing else. With `persistence false`, a broker
restart also clears it.

## 12. Practice with `--simulate`

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py --simulate
```

No broker and no robot: a fake car answers with S5's rules and
telemetry_task's acks, heartbeat, telemetry and live message. Its state
(CALIBRATING, READY, LINE_FOLLOWING, FINISHED, FAULT) shows in the
Simulator panel and, as for a real robot, in the S5 panel. It calibrates
for 3 s first. It does not model the line-sensor check of section 9: it
accepts START without Button 2. Press START: while it runs it follows a line, reads a
barcode every 7 s (letters A to D), takes a junction every 11 s, rejects
a barcode every 19 s, crosses a hump every 17 s and loses the line every
29 s, and tel_drop rises by 3 every 5 s, to show that field working. The
values are made up; only the formats and rates are real.

| Simulator button | What it does |
|---|---|
| Pause heartbeat, Resume heartbeat | Heartbeats stop (red after 3 s) and come back |
| Go offline, Come back online | The session drops (status offline) and returns (connect_count +1) |
| Stall fault | FAULT, if the fake car is running |
| GP20 press | Button 1: START, or STOP while running |
| Reject next | The next command gets result 1 ("rejected") |
| Drop next ack | The next command runs, but its ack never comes ("no ack") |
| Delay next ack 3 s | "no ack", then a late ack |
| Send next ack twice | The second copy is logged as a duplicate |
| Retained command, Clear retained | Another client stores a retained RESET on `<prefix>/cmd/test`, then clears it |
| S4 impact | A 1 s IMPACT flag (motion class 6) in the telemetry |
| S4 hump | A 3 s hump: climbing, then descending |
| S3 barcode, S3 barcode rejected, S3 junction, S3 line lost | One S3 event now, even when the car is not running |

## 13. Troubleshooting

| What you see | Check |
|---|---|
| "no broker host" or "no topic prefix" when starting | Create `mqtt_config.h` (section 5.2), or give `--host` and `--prefix` |
| Broker light red | Is mosquitto running? Same port? |
| Robot light grey | The robot has not connected since the broker started: hotspot, the address in the image (section 4), the robot's console |
| Robot light red, "no heartbeat" | WiFi dropped, or an image without `telemetry_task` (before Stage 4a) |
| No acks | Robot offline; or the dashboard's prefix differs from the image's |
| Acks "queued" but nothing moves | S5 refused it (section 9; read the console): most often the line sensors are not calibrated. Or a `TEST=1` image |
| Console loops `waiting for IP (1)` then `FAILED: authentication rejected` | Restart the hotspot with only the broker laptop and the robot on it (section 3) |
| START greyed out | The robot light is not green, or START is waiting for its ack |
| "Address already in use" | Another dashboard is running; quit it or use `--http-port` |

## 14. Limits

- S5's verdict and events: not sent yet (the event channel is the next
  step); the S5 panel shows the state only.
- RSSI: always 0 for now, shown as "not measured".
- A frame built just after the radio drops can carry 0x10 and arrive after the reconnect.
- S3 rejected barcodes are one total, not per letter (section 8.1).
- A restart can leave one wrong S3 timeline entry in a rare case (section
  8.1).
- Barcode command names follow S3's current command numbering
  (`s3_cmd_t`): if S3 renumbers it, `BC_CMD_NAMES` in
  `app_program/subsystem1/dashboard/s1_decode.py` must follow.
- A repeated command with the same cmd_id is not filtered on the robot
  yet (Stage 4b).
- One dashboard per robot. A second client's commands are logged as
  "command from another client", but its acks can confuse the matching.
- Tested on macOS with mosquitto 2.1.2 and Python 3.13.
- Tested on one Windows laptop (Python 3, mosquitto 2.1.2) with no robot:
  section 15. Not tested on Windows with a robot.

## 15. Windows

This section was tested on one Windows laptop (Python 3, mosquitto 2.1.2)
with no robot: every check in 15.5 passed. Not tested on Windows: a robot
connecting through Windows Firewall, building the firmware (`README.md`
section 1 says to use WSL), and the items in 15.6.

**One command.** Double-click `testing\subsystem1\run_dashboard.bat`,
or run `py -3 -I -B testing/subsystem1/s1_launch.py` in PowerShell from
the repository root (section 7). It finds mosquitto in
`C:\Program Files\mosquitto` without the PATH line of 15.1, refuses to
start while the mosquitto service holds port 1883 (stop it as in 15.2),
starts the broker and the dashboard, and opens the page. The window
stays open only if the launcher stops on an error. Ctrl-C stops both;
Windows may then ask "Terminate batch job (Y/N)?", and either answer
is fine. If Windows asks about mosquitto, allow Private networks only
(15.4). Not yet run on Windows (S1-U23 part C). The manual steps below
are the fallback.

Run the broker and the dashboard in a normal PowerShell window on
Windows itself. Every command below is for PowerShell, from the repository
root unless it says otherwise.

### 15.1 Python and mosquitto

- **Python:** install Python 3 from python.org and check it with
  `py -3 --version`. Wherever this guide says `python3`, use `py -3`:
  Windows usually has no `python3`.
- **mosquitto:** run the Windows installer from mosquitto.org/download
  (it needs administrator rights). It installs to
  `C:\Program Files\mosquitto`, which is not on the PATH.

In every new PowerShell window, before any `mosquitto_*` command and
before starting the dashboard (it starts `mosquitto_sub` and
`mosquitto_pub` itself), run:

```powershell
$env:Path += ";C:\Program Files\mosquitto"
```

Check that it worked:

```powershell
mosquitto_sub --help
```

The first lines say `mosquitto_sub version 2.x.x`. Without the PATH line
the dashboard stops with "mosquitto_sub is not on the PATH" (ignore the
"brew install" hint in that message: it is for macOS).

### 15.2 Stop the mosquitto Windows service first

The installer also sets up a background service named `mosquitto`. With
its default settings it listens on 127.0.0.1 only. If it is running when
you start your own broker (15.3), two brokers share port 1883. The
dashboard connects to 127.0.0.1, so it reaches the service's broker, while
a robot connecting to the laptop's hotspot address would reach yours. They
never see each other: the robot light would stay grey and nothing reports
an error. This happened on the test laptop.

In an **administrator** PowerShell, check for the service:

```powershell
Get-Service mosquitto
```

If it says `Running`, stop it:

```powershell
Stop-Service mosquitto
```

Stop it coming back after a restart:

```powershell
Set-Service mosquitto -StartupType Manual
```

After you start your broker (15.3), check that only one process listens:

```powershell
netstat -ano | findstr :1883
```

You should see `LISTENING` on `0.0.0.0:1883` and `[::]:1883`, all with the
same process id. A `LISTENING` line on `127.0.0.1:1883` with a different
process id means the service is still running.

### 15.3 Start the broker

Create the four-line `mosquitto.conf` of section 6, outside the
repository:

```powershell
New-Item -ItemType Directory -Force "$HOME\s1-broker" | Out-Null
```

```powershell
Set-Content -Path "$HOME\s1-broker\mosquitto.conf" -Value "listener 1883","allow_anonymous true","persistence false","sys_interval 5" -Encoding ascii
```

Start the broker and leave the window open:

```powershell
mosquitto -c "$HOME\s1-broker\mosquitto.conf" -v
```

It prints "Opening ipv4 listen socket on port 1883" and "mosquitto version
2.x.x running". The log's times are seconds since 1970. When the dashboard
connects, the log shows `New client connected from 127.0.0.1:... as
s1dash-<number>`; the number is the process id of the dashboard's
`mosquitto_sub`, which changes every time you restart the dashboard.

### 15.4 Windows Firewall and the network type

The robot connects in from the hotspot, so Windows Firewall must allow
mosquitto in. The first time a broker listens, Windows asks. The checks in
15.5 use 127.0.0.1 only and do not need it.

Windows may treat a phone hotspot as a **Public** network. The broker
accepts any client without a login (section 6), so do not allow mosquitto
on Public networks. Allow it on Private networks only, and set your own
hotspot's network to Private once you have joined it. Look at
`NetworkCategory` in the output (it also prints the hotspot's name:
do not paste it into chats):

```powershell
Get-NetConnectionProfile
```

If it says `Public`, run this in an administrator PowerShell, with the
`InterfaceAlias` from that output (usually `Wi-Fi`):

```powershell
Set-NetConnectionProfile -InterfaceAlias "Wi-Fi" -NetworkCategory Private
```

This step has not been tested with a robot yet.

### 15.5 Check your setup without a robot

Each step builds on the one before. Use one PowerShell window per command
that stays running, each with the PATH line of 15.1.

1. **The dashboard alone.** No broker needed. Open
   `http://127.0.0.1:8765/` and use section 12:

   ```powershell
   py -3 -I -B testing/subsystem1/s1_dashboard.py --simulate
   ```

2. **The live path.** Start the broker (15.3) in one window and the
   dashboard in another:

   ```powershell
   py -3 -I -B testing/subsystem1/s1_dashboard.py --host 127.0.0.1 --prefix robot
   ```

   Within about 6 s the page says "broker connected". The robot light is
   grey and START and RESET are dimmed.

3. **The command bytes.** In a third window, watch the command topic:

   ```powershell
   mosquitto_sub -h 127.0.0.1 -t robot/cmd/dash -F "%t %x"
   ```

   Click STOP. A fresh dashboard's first command has id 0, so the window
   prints `robot/cmd/dash 1480`. The page shows "pending", then "no ack"
   after 2 s, because no robot answers.

4. **A fake robot.** In a fourth window, publish the robot's `online`
   status:

   ```powershell
   mosquitto_pub -h 127.0.0.1 -t robot/status -r -m online
   ```

   The light turns red, "online, but no heartbeat for more than 3 s".
   Now send 20 heartbeats, one a second (each is a valid 9-byte
   HEARTBEAT frame, `docs/S1_binary_frame.md` section 4):

   ```powershell
   1..20 | ForEach-Object { [IO.File]::WriteAllBytes("$HOME\s1-broker\hb.bin",[byte[]](0x12,$_,0x00,$_,0x00,0x02,0x00,0x00,0x00)); mosquitto_pub -h 127.0.0.1 -t robot/telemetry -f "$HOME\s1-broker\hb.bin"; Start-Sleep -Seconds 1 }
   ```

   While it runs the light is green, the heartbeat age stays under about
   1.5 s, the uptime counts up and START and RESET light up. A START
   click prints `robot/cmd/dash 1441` in the third window and ends in "no
   ack". About 3 s after the last heartbeat the light turns red again.
   Clear the fake status when you finish:

   ```powershell
   mosquitto_pub -h 127.0.0.1 -t robot/status -r -n
   ```

### 15.6 Not available on Windows

`s1_dashboard.py --check` and `testing/subsystem1/cmd_rtt.py` stop their
`mosquitto` helpers with SIGINT, which Windows does not support, and
`--check` tests file modes. Read from the code, not run on Windows: run
them on macOS or Linux.

### 15.7 Windows troubleshooting

| What you see | Check |
|---|---|
| `py` is not recognised | Install Python 3 (15.1), or try `python` |
| "mosquitto_sub is not on the PATH" | The PATH line of 15.1, in this window |
| Broker light red | Is the broker window running? `netstat -ano \| findstr :1883` |
| Robot on, light grey, and the broker log never shows the robot's client id | A second broker (15.2), the address in the image (section 4), or the firewall (15.4) |
| `netstat` shows two different process ids listening on port 1883 | The mosquitto service is still running (15.2) |
