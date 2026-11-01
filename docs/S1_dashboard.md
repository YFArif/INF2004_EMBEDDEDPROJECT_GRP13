# S1 laptop dashboard

`testing/subsystem1/s1_dashboard.py` is a one-file dashboard for the
robot: START, STOP and RESET, the ack for each command, an online light
and the 1 Hz heartbeat. It runs on the laptop that runs the broker,
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
  console only (`[S5] EVT 0x65 a=<opcode> b=<1 or 0>`) until Stage 4b.
- No telemetry frames, events or RSSI yet (Stage 4b). The telemetry
  panel stays empty.
- The page, the log and the console never show the broker address, the
  WiFi details or a login.
- The page loads nothing from anywhere else. Its Content-Security-Policy
  header makes the browser block any such load, and stops other sites
  from showing the dashboard inside a frame.

## 2. What you need

| Item | Version | Notes |
|---|---|---|
| Python | 3.8 or later | Tested with 3.13. No packages to install |
| mosquitto (broker and clients) | 2.0 or later | Tested with 2.1.2. macOS: `brew install mosquitto`. Check with `mosquitto_sub --help \| head -2` |
| This repository | branch `buddy1-integration` | It must contain `app_program/subsystem1/telemetry_task.c` (Stage 4a). Ask Caleb if your clone does not have the branch |
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

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then
   release. A drive called **RPI-RP2** appears.
2. Copy the image onto it. On macOS, from the repository root:

   ```bash
   cp build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2 /Volumes/RPI-RP2/
   ```

   On Windows or Linux, drag the file onto the RPI-RP2 drive.
3. The Pico restarts by itself. A "disk not ejected properly" notice is
   harmless.

To see S5's verdicts, open the robot's USB serial console
(`/dev/cu.usbmodem*` on macOS) in any serial terminal.

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

From the repository root:

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py
```

It reads the broker host, port and prefix from
`app_program/subsystem1/mqtt_config.h`. On the broker laptop itself this
also works, and keeps working when the hotspot address changes (the
robot still needs the current address in its image):

```bash
python3 -I -B testing/subsystem1/s1_dashboard.py --host 127.0.0.1
```

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
| rssi | "not measured": Stage 4a cannot read the signal strength yet |
| tel_drop / evt_drop | 0 in Stage 4a |
| Heartbeats lost | Gaps in the heartbeat sequence since the dashboard started (heartbeats use QoS 0, so a few can go missing) |
| Red banner | A retained command is stored on `<prefix>/cmd/...` (section 11) |
| Amber banner | That retained command was cleared |
| "banner unavailable" (top bar) | The banner is off: `--banner-off`, an old `mosquitto_sub`, or a broker without MQTT v5 |

**Buttons.** STOP is always enabled; the Escape key also sends STOP.
START and RESET are enabled only while the robot light is green, and
START (or RESET) stays disabled until the ack of the previous one
arrives, so a double click cannot run the robot from FINISHED. An
enabled START or RESET is lit (white text, visible border); a disabled
one is dimmed and shows the reason underneath, for example "online,
waiting for a heartbeat". The dashboard cannot see S5's state, so it
never shows "calibrating": a START sent while the robot calibrates is
acked "queued" and refused by S5 (section 9).

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

## 9. Operator rules

These are S5's rules (`s5_mission_command()` in
`app_program/subsystem5/s5_mission.c`):

- START runs the robot only from READY. In FINISHED, START only returns
  to READY; the next START runs.
- START is refused while running, while calibrating (up to 3 s after
  power-up, or after Button 2) and in FAULT (send RESET first).
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
  never runs.

## 10. A session, step by step

1. Robot on a stand, wheels off the ground.
2. Hotspot, broker and dashboard as in sections 3, 6 and 7.
3. Power the robot. Wait for the green light and a heartbeat age under
   1.5 s.
4. START: "queued", and the wheels turn (the console shows `b=1`).
5. STOP: "queued", and the wheels stop (FINISHED).
6. START once more: "queued", but the wheels stay still (back to READY).
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
telemetry_task's acks and heartbeat. Its state (CALIBRATING, READY,
LINE_FOLLOWING, FINISHED, FAULT) shows in the Simulator panel only; a
real robot's state is not visible until Stage 4b. It calibrates for 3 s
first.

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

## 13. Troubleshooting

| What you see | Check |
|---|---|
| "no broker host" when starting | Create `mqtt_config.h` (section 5.2), or give `--host` |
| Broker light red | Is mosquitto running? Same port? |
| Robot light grey | The robot has not connected since the broker started: hotspot, the address in the image (section 4), the robot's console |
| Robot light red, "no heartbeat" | WiFi dropped, or an image without `telemetry_task` (before Stage 4a) |
| No acks | Robot offline; or the dashboard's prefix differs from the image's |
| Acks "queued" but nothing moves | S5 refused it (section 9; read the console), or a `TEST=1` image |
| START greyed out | The robot light is not green, or START is waiting for its ack |
| "Address already in use" | Another dashboard is running; quit it or use `--http-port` |

## 14. Limits

- S5's verdict, telemetry frames and events: Stage 4b.
- RSSI: not measured yet.
- A repeated command with the same cmd_id is not filtered on the robot
  yet (Stage 4b).
- One dashboard per robot. A second client's commands are logged as
  "command from another client", but its acks can confuse the matching.
- Tested on macOS with mosquitto 2.1.2 and Python 3.13 only.
