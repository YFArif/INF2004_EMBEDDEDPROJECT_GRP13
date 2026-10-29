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
is marked **planned**. Checked on 27 Oct 2026 against commit `5d23dbf`
on branch `buddy1-mqtt-port`, and the Stage 3 statements on 30 Oct 2026
against the adapter commit on `buddy1-mqtt-stage3`; lwIP is the Pico
SDK's copy, `STABLE-2_2_1_RELEASE`.

## 1. Status

| Part | State | Source |
|---|---|---|
| MQTT driver (`mqtt_link`, `mqtt_queue`) | Built and host-tested. In `MQTT=1` builds it is polled through the Stage 3 adapter; not run on the robot yet (S1-U13) | `app_program/subsystem1/`; TEST_LOG S1-U09 |
| Message codec (`msg_codec`) | Built, host-tested and timed on the Pico | `app_program/subsystem1/`; TEST_LOG S1-U05, S1-U08 |
| Dashboard decoder (`s1_decode.py`) | Built and host-tested | `app_program/subsystem1/dashboard/`; TEST_LOG S1-U11, S1-U12 |
| Stage 3: the radio service task calls `mqtt_link_poll()` | Adapter written: `mqtt_utk_start()` calls `mqtt_link_poll(netif_default)`. Instructor approval 27 Oct 2026; team agreement in person 29 Oct 2026. On-robot test S1-U13 not run | `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`; TEAM.md section 3 ("WiFi and MQTT are special") |
| `telemetry_task`: builds the frames, handles commands | **Planned** | - |

Stage 3 replaced the template's demo client in `mqtt_utk.c` (a text
heartbeat on `MQTT_TOPIC`, 60 s keep-alive) with the adapter, and the
`MQTT_*` settings in `config/wifi_secrets.h` are no longer read. In
`MQTT=1` builds `subsystem1_init()` calls `mqtt_link_init()`, which
creates the driver's two message buffers, and `mqtt_utk_start()` calls
`mqtt_link_poll(netif_default)` on every pass of `cyw43_utk_task` while
the link is up (the "Start-up" note in `mqtt_link.h`). It has not been
run on the robot; S1-U13 decides whether it works there.

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
| `<prefix>/telemetry` | 0 | robot to dashboard | TELEMETRY and HEARTBEAT frames; the type nibble of the header byte tells them apart | `MQTT_LINK_TOPIC_TELEMETRY`, `TOPIC_TELEMETRY`; S1_binary_frame.md section 11 |
| `<prefix>/event` | 1 | robot to dashboard | EVENT frames | `MQTT_LINK_TOPIC_EVENT`, `TOPIC_EVENT` |
| `<prefix>/ack` | 2 | robot to dashboard | ACK frames | `MQTT_LINK_TOPIC_ACK`, `TOPIC_ACK` |
| `<prefix>/cmd/#` | - | dashboard to robot | COMMAND frames. Only the payload reaches the robot's code, not the topic | `build_topics()`, `incoming_publish_cb()` |
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
| `telemetry` (TELEMETRY, HEARTBEAT) | 0 | no | **planned** (`telemetry_task`) |
| `event`, `ack` | 1 | no | **planned** (`telemetry_task`) |

The driver takes QoS and retain from each `mqtt_link_publish()` call, so
the planned values become real only with `telemetry_task`. A command can
arrive twice under QoS 1, so the command handler must accept a repeat
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
  for the first sentence is in section 9. Neither sentence is measured
  end to end yet: the second needs the on-robot test S1-U13 (not run),
  and the first also needs `telemetry_task`.

## 7. Message formats

All payloads are binary, packed MSB first, as defined in
[S1_binary_frame.md](S1_binary_frame.md) sections 2 to 6. The layout
version is 1 (`MSG_LAYOUT_VERSION`); a receiver rejects any other.

| Message | Topic | Payload size | Source |
|---|---|---|---|
| TELEMETRY | `telemetry` | 25 B | `MSG_TEL_BYTES` |
| HEARTBEAT | `telemetry` | 9 B | `MSG_HB_BYTES` |
| EVENT | `event` | 7 B | `MSG_EVT_BYTES` |
| COMMAND | `cmd/...` | 2 B | `MSG_CMD_BYTES` |
| ACK | `ack` | 3 B | `MSG_ACK_BYTES` |

The driver drops a command longer than 2 bytes
(`MQTT_LINK_COMMAND_MAX`, counted in `command_dropped`); a shorter one
reaches the command handler, whose decoder rejects it. The driver also
refuses to publish a payload over 25 bytes (`MQTT_LINK_PAYLOAD_MAX`).

## 8. Message rates

| Message | Rate | Source |
|---|---|---|
| TELEMETRY | 2 Hz | **planned**: S1_binary_frame.md section 3 |
| HEARTBEAT | 1 Hz | **planned**: S1_binary_frame.md section 4 |
| EVENT | When something happens | **planned** |
| ACK | One per command | **planned** |

None of these rates exists in the code yet (`telemetry_task` is not
built), and none is measured.

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

**NFR14 budget.** After the access point or broker comes back,
telemetry shall resume within 15 s (section 6). The WiFi rejoin time
from the hotspot's return is measured for drop 3 of S1-U10 only: 8 s, at
most 9 s given the 1 s timestamps. Drop 1 came back 9 s after its -3
(20:03:03 to 20:03:12, through the template's retry), but its hotspot-on
time was recorded only to the minute, so its rejoin time from the
hotspot's return is unknown; drop 2 has no hotspot-on time at all. The
about 6 s left for the MQTT reconnect (broker lookup, connect, subscribe,
`online`) and the first telemetry frame, which can wait up to one 500 ms
period (**planned**, `telemetry_task`), is therefore established only for
the 8 s case; a re-measurement with Mac-clock markers for every drop is
needed (**planned**). The reconnect itself is not measured, and the whole
chain is unmeasured end to end. S1-U13 (not run) will measure the time to
`online` after each drop; telemetry still needs `telemetry_task`.

If only the broker was down, the WiFi needs no rejoin. The driver waits
up to 8 s between tries (`BACKOFF_MAX_MSEC`), and that wait starts only
when a try ends: a try in flight to a silent host when the broker comes
back can fail on its 5 s timeout first (`CONNECT_TIMEOUT_MSEC`;
`step_connecting()`, `enter_backoff()`). So about 7 s left of the 15 s
is the best case, not the budget. Not measured.

## 10. Full queues and drop counters

Both queues hold 8 messages (`MQTT_LINK_QUEUE_DEPTH`). When a queue is
full a new message is refused and the queued ones are kept, so after an
outage the robot sends the oldest first.

| Counter in `mqtt_link_status_t` | Counts | Source |
|---|---|---|
| `publish_queue_full` | Publishes refused because the publish queue was full | `mqtt_queue_put_publish()`, `count_full()` |
| `publish_failed` | Publishes given up after 3 tries, refused by lwIP for good, or with a topic ID out of range | `finish_message()`, `submit_pending()`, `take_pending()` |
| `command_dropped` | Commands over 2 bytes, command topics over 48 characters, or a full command queue | `incoming_publish_cb()`, `finish_command()` |
| `command_received` | Commands handed to the robot's code | `finish_command()` |
| `connect_count`, `disconnect_count`, `backoff_msec` | Sessions made and lost, and the current retry delay | `connection_cb()`, `enter_backoff()` |
| `b_stale` | The driver has not run for 1000 ms or more | `MQTT_LINK_STALE_MSEC` |

Read them with `mqtt_link_get_status()`. The heartbeat's `tel_drop`,
`evt_drop` and `cmd_reject` fields will carry the robot-side drops to
the dashboard: **planned** (`telemetry_task`).

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
  fault names, opcode and result names) (`describe()`).
- `--session` keeps state across lines: lost frames from `seq`, and full
  encoder counts rebuilt from the 16 bits sent (`Session`, `unwrap16()`).

Offline check with the 18 golden vectors:

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

and the offline check of section 11: 18 JSON lines, no errors.

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
topics received on `robocar/car1/cmd/#` at QoS 1. TEST_LOG S1-U12.

**Not run:**

- Anything on the robot. S1-U13 (Stage 3 on the robot: status topic,
  broker restart, hotspot drops, power cut) is defined but not run, and
  steps 4 and 5 below also need `telemetry_task`.

**On the robot (planned).** Steps 1 to 3 and 6 need only Stage 3, which
is written but not run on the robot (S1-U13); steps 4 and 5 also need
`telemetry_task`:

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
| Prefix 32 characters, command topic suffix 48, command 2 bytes, payload 25 bytes | `PREFIX_MAX_LEN`, `MQTT_LINK_TOPIC_MAX`, `MQTT_LINK_COMMAND_MAX`, `MQTT_LINK_PAYLOAD_MAX` |
| 8 messages per queue; when full, the new message is refused | `MQTT_LINK_QUEUE_DEPTH` |
| Only one task may read commands | "Commands" note in `mqtt_link.h` |
| The robot notices a silent broker 5 to 10 s late | section 3 |
| The retry-delay reset after a 10 s session has no test yet | `STABLE_SESSION_MSEC` |
| Heartbeat RSSI is not available yet (it must be read inside the radio service task), and the S4 terrain fields will read 0 until S4's data exists | **planned** (`telemetry_task`) |
| Every layout field is still marked "guess" or "from Version B" until its owner confirms it | S1_binary_frame.md section 1 |

## 14. What changes next

- After S1-U13 (Stage 3 on the robot): section 1 says whether the driver
  works there, section 9 gets the measured reconnect and NFR14 times, and
  section 12 gets the results of steps 1 to 3 and 6.
- With `telemetry_task`: the planned values in sections 5, 8 and 10 become
  code, and the measured rates are added.
- Sections 2, 3, 4, 7, 11 and 13 stay as they are, unless the layout
  version changes.
