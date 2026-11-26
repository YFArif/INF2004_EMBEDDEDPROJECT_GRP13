# S1 MQTT link

Buddy 1 (Caleb), subsystem 1: how the robot and the dashboard talk over
MQTT. It covers the settings, the topics, what happens when the network
or a queue fails, and how to read the messages. The bit layout of every
message is in [S1_binary_frame.md](S1_binary_frame.md); it is not
repeated here.

Every number below comes from the code or a test log, named in the
Source column by symbol (so the reference survives line changes) or by
TEST_LOG row. A rule that comes from the MQTT 3.1.1 standard or from the
mosquitto tools is marked *standard* or *tool*. Anything not built yet
is marked **planned**. Checked on 27 Oct 2026 against commit `5d23dbf`,
and the Stage 3 statements on 30 Oct 2026 against the adapter commit
`4cddb49`; both commits are on `main`. lwIP is the Pico SDK's copy,
`STABLE-2_2_1_RELEASE`.

## 1. Status

| Part | State | Source |
|---|---|---|
| MQTT driver (`mqtt_link`, `mqtt_queue`) | Built and host-tested. In `MQTT=1` builds it is polled through the Stage 3 adapter; run on the robot in S1-U13: PASS at `9a01717`, 31 Oct 2026 | `app_program/subsystem1/`; TEST_LOG S1-U09, S1-U13 |
| Message codec (`msg_codec`) | Built, host-tested and timed on the Pico | `app_program/subsystem1/`; TEST_LOG S1-U05, S1-U08 |
| Dashboard decoder (`s1_decode.py`) | Built and host-tested | `app_program/subsystem1/dashboard/`; TEST_LOG S1-U11, S1-U12 |
| Stage 3: the radio service task calls `mqtt_link_poll()` | Adapter written: `mqtt_utk_start()` calls `mqtt_link_poll(netif_default)`. Instructor approval 27 Oct 2026; team agreement in person 29 Oct 2026. On the robot: S1-U13 PASS at `9a01717`, 31 Oct 2026 | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`; TEAM.md section 3 ("WiFi and MQTT are special"); TEST_LOG S1-U13 |
| `telemetry_task`, Stage 4a: START, STOP and RESET forwarded to S5 with an ack, and the 1 Hz heartbeat | On `main` and host-tested (S1-U14). Not run on the robot yet (S1-U06). The 2 Hz telemetry frame and the 5 Hz live message are written in the same task (7 Nov 2026) and host-tested (S1-U05, S1-U11, S1-U14, S1-U16 part A), not run on the robot yet; each goes only under the queue rule of section 1.1. Fault bit 4 is implemented (8 Nov 2026, PR #5, section 1.1) and host-tested (S1-U14), not run on the robot yet. Events (Stage 4b, section 1.2) and the heartbeat RSSI are written (21 Nov 2026, branch `s1-demo-ready`) and host-tested (S1-U14, S1-U09, S1-U11), not run on the robot yet (S1-U30) | `app_program/subsystem1/telemetry_task.c`, `s1_telem.c`; section 1.1; TEST_LOG S1-U14, S1-U06 |
| Laptop dashboard (`s1_dashboard.py`) | On `main`: START, STOP and RESET with each ack matched by cmd_id, the online light and heartbeat, the retained-command banner, and `--simulate` (a fake car). Test S1-U16: part A automated in simulate mode, part B on the robot after S1-U06 | `testing/subsystem1/s1_dashboard.py`; [S1_dashboard.md](S1_dashboard.md); TEST_LOG S1-U16 |

Stage 3 replaced the template's demo client in `mqtt_utk.c` (a text
heartbeat on `MQTT_TOPIC`, 60 s keep-alive) with the adapter, and the
`MQTT_*` settings in `config/wifi_secrets.h` are no longer read. In
`MQTT=1` builds `subsystem1_init()` calls `mqtt_link_init()`, which
creates the driver's two message buffers, and `mqtt_utk_start()` calls
`mqtt_link_poll(netif_default)` on every pass of `cyw43_utk_task` while
the link is up (the "Start-up" note in `mqtt_link.h`). S1-U13 ran it on
the robot on 31 Oct 2026 and passed: the status topic, a broker restart,
hotspot drops and a power cut (section 9). It also logged five
unexpected disconnects, not explained yet: four after the second boot,
each followed by `online` with no reboot, and one in the first boot,
followed by a power cycle (TEST_LOG S1-U13).

### 1.2 Stage 4b: events and RSSI

- **Events (FR16).** Any task reports an event with
  `subsystem1_event_post(code, arg_a, arg_b)` (`subsystem1.h`, defined in
  `s1_event.c`). It never waits: it puts one 3-byte record into
  `mbf_event` (30 records, 240 B) or, if the buffer is full, drops it and
  counts the drop atomically (either core may post). Without MQTT the call
  does nothing and returns false. On each pass `telemetry_task` takes up
  to 2 records, but only while the publish queue holds at most 2 messages
  (`S1_TELEM_EVT_QUEUED_MAX`), so events wait in `mbf_event`, not in front
  of an ack. Each is stamped with the uptime in ms and published as a 7 B
  EVENT on `<prefix>/event` at QoS 1, whether or not the session is up (as
  the acks are). evt_drop in the heartbeat is the `mbf_event` drops plus
  the events the driver refused.
- **Codes.** S5's table (`s5_event.h`, 0x50 to 0x65) is the only one so
  far; 0x10 to 0x4F are kept for S1 to S4. The dashboard names each code
  (`s1_decode.event_text()`), for example `OBST_STOP 12 cm at 90 deg` or
  `CTRL_STATE READY -> LINE_FOLLOWING`.
- **S5's verdict.** S5 reports what it did with each command as event
  `COMMAND` (0x65, `{opcode, 1 accepted / 0 refused}`), so the ack keeps
  its meaning, "queued for S5", and no second ack is needed. The event
  carries no cmd_id; the dashboard shows it next to the opcode.
- **S5's side** is a two-file change in S5's folder (`s5_event.c` calls
  `subsystem1_event_post()`; `s5_links.c` drops its ack TODO). It is for
  S5's owner to apply and review; until it is merged S5's events go to the
  console only and evt_drop stays 0.
- **RSSI (assumption A15).** `mqtt_link_poll()` reads the signal strength
  while a session is up, at most every 2 s (`RSSI_PERIOD_MSEC`), through
  `s1_radio_rssi_read()` (`cyw43_wifi_get_rssi()`, one request to the
  radio chip on the radio service task, the only task that may make one).
  It is clamped to -127 to 0 dBm and kept in `mqtt_link_status_t`
  `rssi_dbm`; a failed read keeps the last value. The heartbeat sends it;
  0 means not read yet.
- **Cost** (`NET=lwip MQTT=1` against `main` at `27ad563`, measured with
  `arm-none-eabi-size`): text +928 B, data +0, bss +264 B (240 B of it
  `mbf_event`), for both `SMP=1` and `SMP=0`. The default image's `.bin`
  is identical to `main`'s. `mbf_event` makes 7 of the kernel's 12
  message buffers.

### 1.1 Stage 4a: commands, ack and heartbeat

`telemetry_task` runs at priority 14 on core 0, with its own 2048-byte
stack (Week 6 report section 3; NFR9 keeps it off core 1). In `MQTT=1`
builds `subsystem1_start()` starts it once `mqtt_link_init()` has
succeeded. Each pass it waits up to 50 ms in
`mqtt_link_receive_command()`; a command wakes it at once, so the wait
adds nothing to a STOP's round trip (NFR15).

- **Command to S5.** The type field picks the path: type 7 is a MAP
  (next point); any other type goes to the codec, which checks the 2-byte
  command. A valid START, STOP or RESET goes to
  `subsystem5_command_post(opcode, cmd_id)`, which queues it for
  `vehicle_ctrl_task` without blocking and raises `EV_COMMAND`.
- **MAP to S3.** A 5-byte MAP (S1_binary_frame.md section 5) carries the
  barcode letters for LEFT, RIGHT, STRAIGHT and U-TURN. `telemetry_task`
  turns each 6-bit index into its character with S3's `s3_code39_char()`
  and calls `s3_set_nav_map()` (IF-S3f). S3 judges the map: a repeated
  letter, `*`, or an index above 43 (`?`) is refused and S3 keeps its old
  map. `s3_set_nav_map()` takes `gh_mtx_s3` with `TMO_FEVR`, so the task can
  wait while S3's line or barcode task holds it, and it prints
  `S3 NAV MAP set: LEFT=D RIGHT=C STRAIGHT=B U-TURN=A` (or `REJECTED
  (kept)`) with `tm_printf` on `telemetry_task`'s own stack (S1-U25
  measures it). A MAP is taken in any state; S5 is not asked. S3 does
  not save the map and goes back to ABCD at every reset, so the dashboard
  sends it again after a restart (S1_dashboard.md).
- **Ack.** One 3-byte ack per command the driver delivers, on
  `<prefix>/ack` at QoS 1, not retained. Result 0 means **queued for S5**:
  the post returned `E_OK`. `vehicle_ctrl_task` decides later, on
  core 1, and S1 cannot see that verdict, so the dashboard labels a
  result 0 "queued", not "started". S5 refuses, for example, START in
  FAULT or while running, and RESET outside FAULT or FINISHED
  (`s5_mission_command()`); its verdict shows only on the console
  (`[S5] EVT 0x65 a=<opcode> b=<1 or 0>`). Result 1 means the codec
  refused the command (echo rules in S1_binary_frame.md section 5.1) or
  the post failed (S5's queue full or missing): a **post failure**.
  A MAP's ack has opcode 0 and the MAP's cmd_id; result 0 means
  **applied** (`s3_set_nav_map()` returned `E_OK`), not queued, and
  result 1 that S3 refused it or that it was malformed (opcode 0 and
  cmd_id 0 then). Ack opcode 0 also answers a refused command; section
  5.1 of S1_binary_frame.md says how the two are told apart.
- **Heartbeat.** 9 bytes on the telemetry topic at QoS 0, once a second
  on absolute deadlines, and only while the session is up and the driver
  snapshot is fresh, so an outage does not fill the publish queue that
  the acks need. rssi is the driver's last reading (section 1.2; 0 means
  "not measured" yet), evt_drop the events lost (section 1.2). tel_drop is
  the periodic
  messages skipped or refused (next point), and cmd_reject is the codec
  rejections plus the post failures plus the refused or malformed MAPs
  (`map_reject`) plus the driver's `command_dropped`.
- **Keeping room for the acks.** The publish queue is first in, first out
  and the driver sends one message at a time, so a periodic message delays
  any ack queued behind it. Before each periodic message `telemetry_task`
  reads `mqtt_link_publish_free()`: telemetry and live each go only with at
  most 1 record queued (`S1_TELEM_TEL_QUEUED_MAX`,
  `S1_TELEM_LIVE_QUEUED_MAX`), the heartbeat only with at most 3
  (`S1_TELEM_HB_QUEUED_MAX`), all in `s1_telem.h`. A message held back is
  counted in
  `periodic_skipped`, sent as tel_drop, and is not retried. Acks are never
  held back. A QoS 0 publish still counts as finished only when its TCP ACK
  arrives; whether to change that waits for the measurements on the car.
- **Offline.** The session is clean: a command sent while the robot is
  offline is lost and never acked. A command already received when the
  session drops is still posted; its ack waits in the publish queue.
  Never publish a command with retain: lwIP does not report the retain
  flag, so the robot would run a retained command again after every
  reconnect.
- **Fault bit 4 (dashboard link lost).** Each pass reads the radio link
  (`s1_radio_link_read()`, through `cyw43_utk_get_status()`) and the driver
  status. `s1_telem_wifi_update()` sets `S5_FAULT_WIFI` with
  `subsystem5_fault_set()` when the radio is not `CYW43_LINK_UP` or the
  session is down (the heartbeat's test, so a stale status counts as down),
  and clears it with `subsystem5_fault_clear()` when both are good. It is
  armed only after the first successful connection and acts only on a
  change. There is no `EV_FAULT` and no wire change, and S5 does not act on
  the bit (S1_binary_frame.md section 3.1). Each change prints one line:
  `S1: wifi fault set link=<L> mqtt=<M> stale=<0|1> mask=0x<hh>`, or the same
  with `clear`. `link` is the radio link status, `mqtt` the driver state (0
  IDLE, 1 RESOLVING, 2 CONNECTING, 3 CONNECTED, 4 BACKOFF) and `mask` the flag
  read back, cut to 5 bits. `tm_printf` masks core 0's interrupts for the
  whole line, and a core-1 task that prints at that moment waits for it; not
  measured.

## 2. Configuration

The broker settings live in `app_program/subsystem1/mqtt_config.h`,
copied from `mqtt_config.example.h` and never committed (the folder's
`.gitignore` excludes it).

| Setting | Value or limit | Source |
|---|---|---|
| Broker host | A dotted IPv4 address is used as it is; a name is looked up by DNS | `resolve_broker()` in `mqtt_link.c` |
| Port | 1883 in the example. Plain TCP: there is no TLS, so the login travels in clear | `MQTT_LINK_CFG_BROKER_PORT` in `mqtt_config.example.h`; `start_connect()` passes no TLS settings |
| Client ID | At most 32 characters, unique on the broker | `CLIENT_ID_MAX_LEN`, `mqtt_link_check_client_t` |
| Topic prefix | At most 32 characters; no leading or trailing slash; no `#` or `+` | `PREFIX_MAX_LEN`, `mqtt_link_check_prefix_t`, `check_config()` |
| User name, password | At most 32 characters each; `""` means none | `LOGIN_MAX_LEN`, `login_or_null()` |
| Enable switch | `MQTT_LINK_CFG_SET` must be 1, or the build stops | `mqtt_config.example.h` |

## 3. Connection behaviour

| Item | Value | Source |
|---|---|---|
| Keep-alive | 5 s | `KEEPALIVE_SECONDS`; S1-U09 (`test_connect_info`) |
| Keep-alive ping | A PINGREQ on every 5 s timer tick, whatever else was sent: with the keep-alive equal to the timer period, one tick always reaches it | lwIP `mqtt_cyclic_timer()`, `MQTT_CYCLIC_TIMER_INTERVAL` 5 s |
| Robot gives up on a silent broker | 5 to 10 s after the last byte from the broker (TCP acknowledgements count): lwIP closes on the second tick with nothing received, because its test is ticks x 5 > 5 + 5 / 2 | lwIP `mqtt_cyclic_timer()`, `mqtt_tcp_sent_cb()` |
| Connect time limit | 5 s; lwIP's own limit is 100 s | `CONNECT_TIMEOUT_MSEC`, `step_connecting()`; lwIP `MQTT_CONNECT_TIMOUT`; pinned at compile time in S1-U09 |
| Retry delay | 1, 2, 4, 8, 8, ... s | `BACKOFF_MIN_MSEC`, `BACKOFF_MAX_MSEC`, `next_backoff()`; S1-U09 (`test_backoff`) |
| Retry delay reset | Back to 1 s after a session that lasted at least 10 s | `STABLE_SESSION_MSEC`, `enter_backoff()`. No test covers this yet. |
| While the link is down | The driver is not polled (the adapter runs only while `CYW43_LINK_UP`), so it makes no tries and does not build up a long retry delay during an outage; if a poll finds the netif without link or address, it waits for the link without further tries | `cyw43_utk.c`, `mqtt_utk_start()`; `step_idle()`, `link_ready()` |
| Session | Clean: the broker keeps nothing for the robot while it is away | lwIP `mqtt_client_connect()` always sets `MQTT_CONNECT_FLAG_CLEAN_SESSION` |
| Order after connecting | Subscribe to the commands, publish "online", then send queued messages one at a time | `step_connected()`, `step_publish()` |
| Publish retries | 3 tries in all, then the message is dropped and counted; a lost connection does not count as a try | `PUBLISH_MAX_ATTEMPTS`, `finish_message()` |
| When a try fails | No acknowledgement within 30 s | lwIP `MQTT_REQ_TIMEOUT` |

## 4. Topics

`<prefix>` is `MQTT_LINK_CFG_TOPIC_PREFIX`. FR13 (Week 6 report,
section 1.1) names the last-will topic `robot/status`: with the prefix
set to `robot`, the driver publishes exactly that, because it builds the
will topic as `<prefix>/status` (`build_topics()`,
`MQTT_LINK_TOPIC_STATUS`). `mqtt_config.example.h` uses the prefix
`robocar/car1`, which gives `robocar/car1/status`.
[S1_binary_frame.md](S1_binary_frame.md) writes every topic as
`<prefix>/...`; its one wire example (section 9.1) uses the prefix
`robot`, marked there as a guess.

| Topic | ID | Direction | Carries | Source |
|---|---|---|---|---|
| `<prefix>/telemetry` | 0 | robot to dashboard | TELEMETRY, LIVE and HEARTBEAT frames; the type nibble of the header byte tells them apart | `MQTT_LINK_TOPIC_TELEMETRY`, `TOPIC_TELEMETRY`; S1_binary_frame.md section 11 |
| `<prefix>/event` | 1 | robot to dashboard | EVENT frames | `MQTT_LINK_TOPIC_EVENT`, `TOPIC_EVENT` |
| `<prefix>/ack` | 2 | robot to dashboard | ACK frames | `MQTT_LINK_TOPIC_ACK`, `TOPIC_ACK` |
| `<prefix>/cmd/#` | - | dashboard to robot | COMMAND and MAP frames; the type nibble tells them apart. Only the payload reaches the robot's code, not the topic | `build_topics()`, `incoming_publish_cb()` |
| `<prefix>/status` | - | robot to everyone | The text `online` or `offline` (section 6) | `MQTT_LINK_TOPIC_STATUS`, `send_birth()` |

- The IDs are the values of `mqtt_link_topic_t`; tasks publish by ID and
  `mqtt_link.c` adds the prefix (S1-U09 checks all three).
- `cmd/` and `status` are reserved: no ID maps to them, so the robot can
  never publish there (the note above `TOPIC_TELEMETRY`).
- `#` also matches `<prefix>/cmd` itself (*standard*, MQTT 3.1.1 section
  4.7.1.2).
- The part of a command topic after the prefix, `cmd/` included, may be
  at most 48 characters, or the command is dropped
  (`MQTT_LINK_TOPIC_MAX`, `incoming_publish_cb()`).

## 5. QoS and retain

| Topic | QoS | Retained | Source |
|---|---|---|---|
| `status` | 1 | yes | `start_connect()` (last will), `send_birth()` |
| `cmd/#` (subscription) | 1 | - | `COMMAND_QOS` |
| `telemetry` (TELEMETRY, LIVE, HEARTBEAT) | 0 | no | `telemetry_task` |
| `ack` | 1 | no | `telemetry_task` (`s1_telem_on_command()`) |
| `event` | 1 | no | `telemetry_task` (`s1_telem_event()`), section 1.2 |

The driver takes QoS and retain from each `mqtt_link_publish()` call. A
command can arrive twice under QoS 1, so the command handler must accept a
repeat
(the "Command delivery" note in `mqtt_link.h`).

## 6. Online and offline (NFR14)

- On every connect the robot registers a last will: `offline` on
  `<prefix>/status`, QoS 1, retained (`start_connect()`).
- Straight after connecting, before any queued message, it publishes a
  retained `online` on the same topic (`step_publish()`, `send_birth()`).
- The broker publishes the `offline` will:
  - at once when the robot closes the connection itself, because lwIP's
    `mqtt_disconnect()` closes TCP without sending an MQTT DISCONNECT,
    and a broker publishes the will whenever a connection ends without
    one (*standard*, MQTT 3.1.1 section 3.1.2.5);
  - 7.5 s (1.5 x the 5 s keep-alive) after the last packet when the robot
    simply goes silent (*standard*, MQTT 3.1.1 section 3.1.2.10).
- NFR14, from the Week 6 design report, section 1.2 (Non-Functional
  Requirements): "After the access point or broker comes back, telemetry
  shall resume within 15 s. The broker shall show the robot as offline
  within 10 s of link loss (keep-alive 5 s plus last-will)." The budget
  for the first sentence is in section 9. S1-U13 measured the second on
  the robot: `offline` 5.3 to 7.8 s after the hotspot-off or power-cut
  marker (section 9). The first needs `telemetry_task`.

## 7. Message formats

All payloads are binary, packed MSB first, as defined in
[S1_binary_frame.md](S1_binary_frame.md) sections 2 to 6. The layout
version is 1 (`MSG_LAYOUT_VERSION`); a receiver rejects any other.

| Message | Topic | Payload size | Source |
|---|---|---|---|
| TELEMETRY | `telemetry` | 25 B | `MSG_TEL_BYTES` |
| LIVE | `telemetry` | 14 B | `MSG_LIV_BYTES` |
| HEARTBEAT | `telemetry` | 9 B | `MSG_HB_BYTES` |
| EVENT | `event` | 7 B | `MSG_EVT_BYTES` |
| COMMAND | `cmd/...` | 2 B | `MSG_CMD_BYTES` |
| MAP | `cmd/...` | 5 B | `MSG_MAP_BYTES` |
| ACK | `ack` | 3 B | `MSG_ACK_BYTES` |

The driver drops an incoming payload longer than 5 bytes, the MAP
(`MQTT_LINK_COMMAND_MAX` = `MSG_IN_MAX_BYTES`, counted in
`command_dropped`; 2 bytes until the MAP existed); a shorter one reaches
the command handler, whose decoder rejects anything that is not exactly
a COMMAND or a MAP. The driver also refuses to publish a payload over 25
bytes (`MQTT_LINK_PAYLOAD_MAX`).

## 8. Message rates

| Message | Rate | Source |
|---|---|---|
| TELEMETRY | 2 Hz | `telemetry_task.c`, `S1_TELEM_TEL_PERIOD_MSEC` (written, not run on the robot): S1_binary_frame.md section 3 |
| LIVE | 5 Hz | `telemetry_task.c`, `S1_TELEM_LIVE_PERIOD_MSEC` (written, not run on the robot): S1_binary_frame.md section 3A |
| HEARTBEAT | 1 Hz while the session is up | Stage 4a: `telemetry_task.c`, `S1_TELEM_HB_PERIOD_MSEC` |
| EVENT | When a task posts one; at most 2 per `telemetry_task` pass (40 a second) | Stage 4b: `telemetry_task.c`, `S1_TELEM_EVT_PER_PASS` (written, not run on the robot), section 1.2 |
| ACK | One per command the driver delivers | Stage 4a: `telemetry_task.c` |

EVENT exists since 21 Nov 2026 (Stage 4b). The heartbeat and the ack
exist since Stage 4a, and TELEMETRY and LIVE since 7 Nov 2026; none is
measured on the robot yet (S1-U06). The telemetry topic now carries 8
publishes a second (2 Hz telemetry, 5 Hz live, 1 Hz heartbeat), so the
NFR15 trials must be run with all three on. Those are the rates on a link
that keeps up; on a slower one the queue rule of section 1.1 skips messages
and tel_drop counts them.

The three schedules start apart so that they never share a deadline. After
the first heartbeat, the first live message is due at 100 ms and the first
telemetry frame at 250 ms (`S1_TELEM_LIVE_OFFSET_MSEC`,
`S1_TELEM_TEL_OFFSET_MSEC`). With no offsets, all three fall due in the same
pass every 1000 ms. S1-U14 checks the deadlines at 1 ms steps.

A deadline is not the same as the pass that notices it. `telemetry_task`
looks at its schedules once a pass, and a pass lasts the 50 ms command wait
(`TELEMETRY_TASK_RX_WAIT_MSEC`) plus its own work, or less when a command
arrives. The closest deadlines are a frame and a live message 50 ms apart,
the largest gap these periods allow with any offsets, so a pass longer than
50 ms can still find those two due together. A model with a fixed pass length
gives none at 50 ms, one about every 25 s at 51 ms and one about every 3 s at
60 ms; the real pass length is not measured. `send_periodic()` sends both,
each under the queue rule of section 1.1.

## 9. Recovery

**WiFi.** Measured in S1-U10 (TEST_LOG row of 26 Oct 2026; logs
`2026-10-26_s1u10_*`), on the template's WiFi code with a phone hotspot
switched off three times:

| Drop | Hotspot off | Detected | Link up | How | Source |
|---|---|---|---|---|---|
| 1 | about 60 s (phone clock, minute precision) | 20:02:04 (status 1) | 20:03:12 | status -3 at 20:03:03 (printed as "FAILED: authentication rejected"), then the template's retry (joining at 20:03:08) | S1-U10 row; `2026-10-26_s1u10_wifi_rejoin.txt` |
| 2 | about 10 s | 20:04:11 | 20:04:19 | the chip itself, no joining line | S1-U10 row; same log |
| 3 | 20:17:16 to 20:22:28 (5 min 12 s, Mac-clock markers) | 20:17:26 | 20:22:36, 8 s after the hotspot came back | the chip itself; status stayed 1, never negative | S1-U10 row; `2026-10-26_s1u10_wifi_rejoin_long.txt`, `2026-10-26_s1u10_markers.txt` |

The link came back every time, with no power cycle and no code change.
Only drop 3 has the time the hotspot came back, so the time from the
hotspot's return to link up is known for drop 3 alone. While the hotspot
was off the link status read 1 (JOIN), never 0; it went negative once,
-3 in drop 1.

**WiFi rejoin in MQTT=1 builds (FR17).** On branch `s1-fr17`, code commit
`244bef8`: host-tested (S1-U28, PASS), not yet run on hardware (S1-U29).
Two of the three S1-U10 recoveries above were the chip's own (no
`joining` line), so S1-U10 never showed the host retrying a lost link.
The template's retry in `cyw43_utk_task()` is armed only by a negative
status and has no time limit on any other. After an access point loss the
status reads 1 (JOIN): the link-down event clears only the netif's link
flag, and the driver's join state stays ACTIVE (cyw43-driver v1.1.1,
`cyw43_ctrl.c` lines 403 to 414 and 613 to 626); nothing in the driver
has a timer. So when the chip did not recover, nothing did: on 8 Nov 2026
the status read 1 from 17:53:26 and still read 1 at the power cut at
18:01:28.3, 7 min 25 s after the hotspot came back on at 17:54:03.6, with no
`joining` line (`2026-11-08_s1u15_console.txt`,
`2026-11-08_s1u15_markers.txt`), and on the cold boot of 10 Nov 2026 each
join held status 1 for 14 to
201 s before a -3, eight times, about 624 s in all
(`2026-11-10_s1u21_console.txt.gz`).

The link status values, from the same driver:

| Status | Name | Meaning |
|---|---|---|
| 3 | UP | the netif has its link and an address |
| 2 | NOIP | associated, no address yet |
| 1 | JOIN | a join is active: just requested, or the link was lost and the driver still waits |
| 0 | DOWN | after a disassociation event (a leave, or a de-authentication with reason 2) |
| -1 | FAIL | setting the SSID failed |
| -2 | NONET | no network with that SSID was found |
| -3 | BADAUTH | authentication or the key exchange failed |

A -3 is not always a wrong password. When the key exchange times out
(`CYW43_EV_PSK_SUP` with status 4, 8 or 10 and reason 15, `cyw43_ctrl.c`
line 419), the driver joins again by itself and the status stays 1; only
a failure outside that list gives -3. The driver also sets the join
ACTIVE again when a good authentication follows a bad one (lines 385 to
388), so a -3 can be followed by a 1 within one attempt, and the
template's pending retry was cancelled by that 1.

In MQTT=1 builds the template's retry is compiled out, and
`cyw43_utk_task()` passes every status, on every pass, to S1's policy
(`app_program/subsystem1/s1_wifi_rejoin.c`, a pure state machine) and
carries out the join or leave it returns. The timings are starting
values; S1-U29 tunes them.

| State | Entered | Left |
|---|---|---|
| JOINING | at the boot join, or when a wait ends | status 3: UP. A negative status, a join request the driver refuses, or 10 s at status 0, 1 or 2: BACKOFF, attempt + 1 |
| UP | status 3 | any other status: LOST. After 10 s of continuous UP the attempt counter goes back to 0 (status 3 after a rejoin means "associated, with the old address", not "lease confirmed") |
| LOST | a status other than 3 after UP | status 3: UP. A negative status, or 3 s (the chip's grace): BACKOFF |
| BACKOFF | a lost link or a failed attempt | status 3: UP. The wait over (1 s, 2 s, then 4 s for every later attempt): JOINING, or LEAVING |
| LEAVING | only with `S1_WIFI_LEAVE_BEFORE_JOIN` 1, when a wait ends | status 0, or 1 s: JOINING. A status of 3 here is not UP: a leave is in flight |

- Console: one line per state change, `S1: wifi rejoin <from> -> <to>
  attempt=<n> status=<s> wait=<ms> ms (<reason>)`, and once at start,
  `S1: wifi rejoin start JOINING (boot join) leave_first=<0 or 1>`. These
  lines hold no SSID or address, but the template's `wifi: joining` and
  `wifi: link up` lines still print them, so console logs still need
  masking.
- The leave switch, `S1_WIFI_LEAVE_BEFORE_JOIN`, is 0 by default: a plain
  join, which `cyw43_wifi_join()` accepts while a join is active. A leave
  only sends a request; the driver clears its join state later, when the
  disassociation event arrives, and an event that arrives after the new
  join would clear its ACTIVE bit, so the driver could never report
  link-up (the 10 s limit then recovers). S1-U29 decides the default.
- MQTT=0 builds are unchanged: with the default, NET=lwip and TEST=1
  profiles, the image of `244bef8` is byte-identical to main's
  (`ec4839a`); `cyw43_utk.o` differs from main's only in debug
  information (identical with it stripped, identical disassembly).
- Limits: a boot join request the driver refuses (`cyw43_wifi_join()`
  returns an error) still stops the radio task, as in the template
  (`init_result`); a link that reads 3 while the Mac or the broker is
  away starts no WiFi action, by design; no retry can join a hotspot that
  refuses the client.

**Recovery time, end to end, by layer.** Reasoned from the code, not
measured; S1-U29 measures it. The radio task asks for a 2 ms delay per
pass (`CYW43_POLL_MS`), but the kernel tick is 10 ms (`CNF_TIMER_PERIOD`
in `config/config.h`) and a delay ends on the first tick at least the
delay plus one tick after the last tick (`knl_timer_insert()` in
`kernel/tkernel/timer.c`), so a pass starts about every 20 ms, and
`sys_now()` moves in 10 ms steps. A join request blocks its pass for
about a quarter of a second (240 to 260 ms from `joining` to the next
status line in the logs).

| Layer | From, to | Time |
|---|---|---|
| 1. Detection | access point gone, to status 1 | the chip's (10 s in S1-U10 drop 3) |
| 2. Grace | status 1, to BACKOFF | 3 s |
| 3. Wait | BACKOFF, to the join request | 1 s, 2 s, then 4 s |
| 4. Join | request, to status 3 | 3.2 to 3.9 s for a good join; at most 10 s, then the attempt is abandoned and the cycle returns to layer 3 |
| 5. Address | status 3, to an address | none after a drop: the old address is kept while the link is down, and lwIP restarts DHCP on link-up in the background (`dhcp_network_changed_link_up()`). At a cold boot DHCP is part of layer 4 |
| 6. MQTT | status 3, to `online` | measured 2.6 to 4.6 s (S1-U13, with the Mac's own rejoin). From the code: any retry delay still running (at most 8 s), a connect of at most 5 s, then CONNACK, SUBSCRIBE and `online` |
| 7. Telemetry | `online`, to the first frame | at most 0.5 s (the 2 Hz schedule) |

From the moment the access point is back:

- Best case: the chip recovers by itself (8 s after the hotspot came back
  in S1-U10 drop 3), or a join is requested just as it returns: about
  3.2 + 2.6 + 0.5 = 6.3 s.
- The policy is in BACKOFF when it returns: at most 4 + 3.9 + 4.6 + 0.5,
  about 13 s.
- Worst case: it returns just after a join that cannot see it was
  requested, and that join runs to its limit: 10 + 4 + 3.9 + 4.6 + 0.5,
  about 23 s, more if MQTT still has a retry delay to wait out. NFR14's
  15 s is therefore not guaranteed in the worst case.

While the access point stays away, one cycle is a join (at most 10 s,
less if the chip reports a negative status first) and a 4 s wait: at
most one join request every 14 s, each blocking the radio task for about
a quarter of a second.

**Does `mqtt_utk.c` need a change for this? No.** It calls
`mqtt_link_poll()` only while the status is 3, but lwIP's timers keep
running during the outage, because `cyw43_utk_task()` calls
`sys_check_timeouts()` on every pass whatever the status. lwIP's
keep-alive ends a session the broker no longer answers 5 to 10 s after
its last byte, and `connection_cb()` then starts the driver's retry delay
(1 s after a session of 10 s or more, else doubled, at most 8 s). After
an outage longer than about 18 s (10 s plus the longest delay) that delay
is over when the link returns, so the first poll goes straight to a
connect; after a shorter one the session may still be open, or up to 8 s
of delay may be left. One cost stays: during the outage the driver's
state shown to other tasks is frozen, which the WiFi fault rule covers
with the stale flag (section 1.1). Polling the driver during the outage
would let it close the session at once, but that changes the gate in
`cyw43_utk.c` and the behaviour of `mqtt_link.c`, which this change
leaves alone.

**MQTT.** From the code:

- `cyw43_utk.c` calls the adapter only while the link is
  `CYW43_LINK_UP`, so during a WiFi outage the driver is not polled at
  all. lwIP's own timers still run and end a session the broker no longer
  answers, 5 to 10 s after its last byte (section 3); the driver's
  `connection_cb()` then starts a retry delay (`enter_backoff()`).
- If a poll finds the netif without link or address, the driver closes
  the session, waits out one retry delay and then waits for the link
  without further tries (`link_ready()`, `abort_session()`,
  `step_idle()`).
- When the link is back, the driver first waits out any retry delay still
  running (`step_backoff()`), then looks the broker up, connects (5 s
  limit), subscribes and publishes `online` (section 3).
- Queued publishes and commands survive a reconnect, and a lost
  connection does not use up a publish try (the `PUBLISH_MAX_ATTEMPTS`
  and `submit_pending()` notes in `mqtt_link.c`; not covered by a test).

**MQTT, measured on the robot.** S1-U13 (TEST_LOG row of 31 Oct 2026, at
`9a01717`; logs `2026-10-31_s1u13_*`). The robot and the Mac share the
hotspot, so every drop also drops the Mac. Drop 1 is left out: the Mac
did not rejoin within 60 s, so it is invalid.

| Drop | Hotspot off | `offline` after the off marker | Robot link up after hotspot-on | `online` after hotspot-on | Link up to `online` |
|---|---|---|---|---|---|
| 2 | 12.6 s | 5.4 s | 5.4 s | 10.0 s | 4.6 s |
| 3 | 63.0 s | 6.3 s | 15.7 s (status -3 on the way) | 18.6 s | 2.9 s |
| 4 | 62.1 s | 5.3 s | 6.8 s | 9.4 s | 2.6 s |

The `online` times come from the broker log (1 s resolution) and include
the Mac's own rejoin: in drop 2 the Mac's address came back in the same
second as `online`, 4.6 s after the robot's link. Broker restart, after
38.7 s down: `online` 2.3 s after the broker started. Power cut:
`offline` 7.8 s after the marker. After the second boot, four unexpected
disconnects between the planned actions (two closed by the robot, two
keep-alive timeouts at the broker) were each followed by `online` within
0.9 to 8.2 s, with no reboot; their cause is not known yet.

**NFR14 budget.** After the access point or broker comes back,
telemetry shall resume within 15 s (section 6). S1-U13 measured
`online`, standing in for telemetry, with Mac-clock markers for every
drop: 10.0, 18.6 and 9.4 s after the hotspot came back, so one of the
three valid drops is over 15 s. In that drop the robot's own link took
15.7 s (status -3, then link up with no joining line) and the MQTT part,
link up to `online`, took 2.9 s. Over the three drops the MQTT part took
2.6 to 4.6 s, including the wait for the Mac. The first telemetry
frame can wait up to one more 500 ms period (**planned**,
`telemetry_task`), so telemetry itself is still unmeasured end to end.

If only the broker was down, the WiFi needs no rejoin. The driver waits
up to 8 s between tries (`BACKOFF_MAX_MSEC`), and that wait starts only
when a try ends: a try in flight to a silent host when the broker comes
back can fail on its 5 s timeout first (`CONNECT_TIMEOUT_MSEC`;
`step_connecting()`, `enter_backoff()`). So about 7 s left of the 15 s
is the best case, not the budget. S1-U13 measured one case: `online`
2.3 s after the broker started, after 38.7 s down.

## 10. Full queues and drop counters

Both queues hold 8 messages (`MQTT_LINK_QUEUE_DEPTH`). When a queue is
full a new message is refused and the queued ones are kept, so after an
outage the robot sends the oldest first.

| Counter in `mqtt_link_status_t` | Counts | Source |
|---|---|---|
| `publish_queue_full` | Publishes refused because the publish queue was full | `mqtt_queue_put_publish()`, `count_full()` |
| `publish_failed` | Publishes given up after 3 tries, refused by lwIP for good, or with a topic ID out of range | `finish_message()`, `submit_pending()`, `take_pending()` |
| `command_dropped` | Commands over 5 bytes (the MAP; 2 bytes before it), command topics over 48 characters, or a full command queue | `incoming_publish_cb()`, `finish_command()` |
| `command_received` | Commands handed to the robot's code | `finish_command()` |
| `connect_count`, `disconnect_count`, `backoff_msec` | Sessions made and lost, and the current retry delay | `connection_cb()`, `enter_backoff()` |
| `b_stale` | The driver has not run for 1000 ms or more | `MQTT_LINK_STALE_MSEC` |

Read them with `mqtt_link_get_status()`. `mqtt_link_publish_free()` says how
many more records the publish queue can take, from the kernel's free buffer
size (`tk_ref_mbf`); it counts the queue only, not the message being sent.

The heartbeat carries the robot-side drops to the dashboard. `tel_drop` is
`telemetry_task`'s `periodic_skipped` (the queue rule, section 1.1) plus
`periodic_failed` and `hb_publish_failed` (refused by the driver).
`cmd_reject` is section 1.1's sum. `evt_drop` is the posts `mbf_event`
refused (`s1_event_drop_count()`) plus the events the driver refused
(`evt_publish_failed`), section 1.2.
Messages not sent because the session is down are not drops: nothing is
built for them. A message the driver refuses also shows as a seq gap on the
dashboard; one the queue rule skips does not.

A task that calls `mqtt_link_publish()` needs 72 B of its own stack for
it, before the kernel's `tk_snd_mbf()`: 64 B for `mqtt_link_publish()`
and 8 B for `mqtt_queue_put_publish()`. Source: `-fstack-usage` and the
`mqtt_link_publish` prologue in the disassembly, with the build's own
flags, at `5d23dbf`, for both `SMP=1` and `SMP=0`.

The driver itself runs on the radio service task. The deepest static
chain from `cyw43_utk_task` through `mqtt_utk_start()` and
`mqtt_link_poll()` is 1768 B of that task's 8 KB stack. It is an
estimate: `-fstack-usage` and `-fcallgraph-info=su` on the Stage 3
`NET=lwip MQTT=1` build (29 Oct 2026, `SMP=1`), with eight indirect
calls resolved by hand; it leaves out the other indirect calls, interrupt
frames and library leaves such as `memcpy`.

## 11. Reading messages: `s1_decode.py`

`app_program/subsystem1/dashboard/s1_decode.py` decodes one payload into
the same field names and values as the robot's own decoder, and rejects
what it rejects (S1-U11). Python 3 standard library only.

From the command line, one payload per line as hex on stdin:

```
python3 app_program/subsystem1/dashboard/s1_decode.py [--describe] [--session]
```

- Each line prints one JSON object. A rejected line prints
  `{"line": n, "error": reason, "detail": text}`; the reasons are
  `empty`, `version`, `type`, `length` and `hex` (`decode()`, `main()`).
- Exit status: 0 if every line decoded, 1 if any line was rejected, 2 for
  a bad option (`main()`; S1-U11 `CommandLine` tests).
- `--describe` adds display fields (pitch and roll in degrees, mean speed,
  fault names, opcode and result names) (`describe()`). A MAP shows its
  letters (`?` above index 43, as S3 reads it); an ack with opcode 0 and
  an id above 0 is named MAP, with result 0 "applied" and 1 "refused",
  and one with id 0 INVALID (S1_binary_frame.md section 5.1).
- `--session` keeps state across lines: lost frames from `seq`, and full
  encoder counts rebuilt from the 16 bits sent (`Session`, `unwrap16()`).

Offline check with the 25 golden vectors:

```
python3 testing/subsystem1/golden.py docs/S1_binary_frame.md \
    | cut -d'|' -f3 \
    | python3 app_program/subsystem1/dashboard/s1_decode.py --describe
```

From Python: `decode()`, `describe()`, `unwrap16()` and `Session` (see the
module's docstring).

## 12. End to end

**Run on this Mac on 27 Oct 2026, at `5d23dbf`, each with exit status 0:**

```
sh testing/subsystem1/run_codec_test.sh      # S1-U05: PASS
sh testing/subsystem1/run_link_test.sh       # S1-U09: PASS (needs PICO_SDK_PATH)
sh testing/subsystem1/run_decoder_test.sh    # S1-U11: PASS
```

and the offline check of section 11: 18 JSON lines, no errors. That was
the vector count on that date; there are 21 since `75f20da` (LIV-1 to
LIV-3), and the same check now prints 21 lines.

**Local broker, dashboard side only** (the robot is not in it):

```
python3 -I -B testing/subsystem1/broker_test.py
```

It starts mosquitto on 127.0.0.1 only, runs these, and stops it:

```
mosquitto_sub -h 127.0.0.1 -q 1 -t 's1test/stream' -F '%x' \
    | python3 app_program/subsystem1/dashboard/s1_decode.py --describe --session
mosquitto_pub -h 127.0.0.1 -q 1 -t 's1test/stream' -s     # each golden vector
mosquitto_sub -h 127.0.0.1 -q 1 -t 'robocar/car1/cmd/#' -C 2 -F '%x' \
    | python3 app_program/subsystem1/dashboard/s1_decode.py --describe
printf '\024\205' | mosquitto_pub -h 127.0.0.1 -q 1 -t 'robocar/car1/cmd' -s
printf '\024\205' | mosquitto_pub -h 127.0.0.1 -q 1 -t 'robocar/car1/cmd/ui' -s
```

Result: PASS on 27 Oct 2026 at `f2e2f54`: the 18 golden vectors decoded
through the broker as directly, 0 error lines; STOP id 5 from both
topics received on `robocar/car1/cmd/#` at QoS 1. With the 21 vectors:
PASS on 8 Nov 2026 at `ba81b3d`, 21 lines, 0 error lines. TEST_LOG S1-U12.

**On the robot: S1-U13, PASS on 31 Oct 2026 at `9a01717`** (TEST_LOG
S1-U13; section 9 for the times). Steps 1 to 3 and 6 below ran, with
mosquitto on the Mac as the broker: `online` appeared on `robot/status`,
a subscriber started later received it with the retain flag set, and
switching the hotspot off or cutting the robot's power brought `offline`
5.3 to 7.8 s later. Steps 4 and 5 need `telemetry_task` and have not
run.

**On the robot, step by step.** Steps 1 to 3 and 6 need only Stage 3;
steps 4 and 5 also need `telemetry_task`:

1. Run a broker the robot can reach, for example mosquitto on port 1883.
2. Fill in `app_program/subsystem1/mqtt_config.h`; build with
   `make NET=lwip MQTT=1 -j8` in `build_make/` and flash the image.
3. Check the status topic:
   `mosquitto_sub -h <broker> -t '<prefix>/status' -v` shows `online`.
4. Decode the robot's messages (the status topic is text, so it is left
   out):
   `mosquitto_sub -h <broker> -t '<prefix>/telemetry' -t '<prefix>/event' -t '<prefix>/ack' -F '%x' | python3 app_program/subsystem1/dashboard/s1_decode.py --describe --session`
   (`-F '%x'` prints each payload as hex, one per line: *tool*,
   mosquitto_sub; S1-U12).
5. Send STOP with command ID 5 (golden vector CMD-1, bytes 14 85):
   `printf '\024\205' | mosquitto_pub -h <broker> -q 1 -t '<prefix>/cmd' -s`
   (*tool*: mosquitto_pub reads the payload from stdin; *standard*:
   `<prefix>/cmd` matches the robot's `<prefix>/cmd/#`, MQTT 3.1.1
   section 4.7.1.2). The ACK arrives on `<prefix>/ack`.
6. Switch the network off: `offline` appears on `<prefix>/status`.

Steps 5 and 6 are the basis of S1-U06 (START, STOP and RESET, with the
STOP round trip under 300 ms) and of the NFR14 test.

## 13. Known limits

| Limit | Source |
|---|---|
| No TLS: the login and all messages travel in clear | `start_connect()` passes no TLS settings |
| Clean session: a command sent while the robot is offline is lost | lwIP `mqtt_client_connect()`; "Command delivery" note in `mqtt_link.h` |
| QoS 1 commands can arrive twice | `COMMAND_QOS`; "Command delivery" note |
| A retained command whose topic is longer than lwIP's 128-byte header buffer makes the robot reconnect over and over until the retained message is cleared; only trusted clients should publish under `<prefix>/cmd/` | lwIP `MQTT_VAR_HEADER_BUFFER_LEN`; "Command topic length" note in `mqtt_link.h` |
| Prefix 32 characters, command topic suffix 48, incoming payload 5 bytes (the MAP), payload 25 bytes | `PREFIX_MAX_LEN`, `MQTT_LINK_TOPIC_MAX`, `MQTT_LINK_COMMAND_MAX`, `MQTT_LINK_PAYLOAD_MAX` |
| 8 messages per queue; when full, the new message is refused | `MQTT_LINK_QUEUE_DEPTH` |
| With the queue rule, an ack can still wait behind the message being sent plus 2 queued ones, or more if heartbeats were queued while the link was slow. The driver waits for each message's TCP ACK before the next, so this delay is not known until measured on the car | section 1.1; `step_publish()`; S1-U06 |
| `telemetry_task` preempted by the WiFi task while it holds an owner's mutex delays a core-1 task; rare, known, not worked around, not measured | S1_binary_frame.md section 3.4 |
| Only one task may read commands | "Commands" note in `mqtt_link.h` |
| The robot notices a silent broker 5 to 10 s late | section 3 |
| The retry-delay reset after a 10 s session has no test yet | `STABLE_SESSION_MSEC` |
| Heartbeat RSSI is read at most every 2 s, and reads 0 ("not measured") until the first read of a session | `read_rssi()` in `mqtt_link.c`; section 1.2 |
| An ack's result 0 means queued for S5, not carried out; S5's verdict arrives as event `COMMAND` (0x65), without the cmd_id, once S5's side of section 1.2 is merged | sections 1.1 and 1.2 |
| Events from S5 reach the dashboard only once S5 applies its two-file change (section 1.2) | section 1.2 |
| A retained command runs again after every reconnect: lwIP does not report the retain flag | section 1.1 |
| Every layout field is still marked "guess" or "from Version B" until its owner confirms it | S1_binary_frame.md section 1 |

## 14. What changes next

- S1-U13 done (31 Oct 2026): sections 1, 9 and 12 carry its results.
- Stage 4a written (1 Nov 2026): sections 1, 1.1, 8 and 13. S1-U06 adds
  the measured heartbeat rate and STOP round trip.
- Fault bit 4 implemented (8 Nov 2026, PR #5): sections 1 and 1.1. S1-U06
  checks it on the robot.
- Stage 4b written (21 Nov 2026, section 1.2): events and RSSI. S5's
  verdict travels as event `COMMAND`, not in the ack. S1-U30 measures it
  on the robot.
- After the car measurements: decide whether a QoS 0 publish should count
  as finished before its TCP ACK (section 1.1).
- Sections 2, 3, 4, 7, 11 and 13 stay as they are, unless the layout
  version changes.
