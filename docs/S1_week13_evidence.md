# S1 Week 13 evidence: minimum-bit data representation

Buddy 1 (S1: WiFi, MQTT, telemetry and commands). The four items Change
Requirement 1 asks for, for S1's data. Every number names its source;
the full layout, with every field's offset and status, is
[S1_binary_frame.md](S1_binary_frame.md), and every test is in
`testing/subsystem1/TEST_LOG.md`.

Before = the Week 6 design with JSON and text messages, frozen at tag
`pre-minbit` (`a25b9f4`, 19 Oct 2026). After = `main`.

## 1. Data representation

All S1 messages are packed with masks and shifts into a byte array, most
significant bit first, never by sending a C struct or bit-field
(Barr-C 5.5). Signed values are clamped, then offset to unsigned before
masking (Barr-C 5.3b). One shared header, `msg_layout.h`, holds every
position and width; compile-time checks stop the build if two fields
overlap or a message outgrows its size. Byte 0 of every message is the
layout version (4 bits) and the message type (4 bits).

### 1.1 Telemetry frame (2 Hz, 25 B; S1_binary_frame.md section 3)

| Field | Valid range | Bits | Representation |
|---|---|---|---|
| seq | 0 to 255 | 8 | modulo counter |
| ctrl_state (S5) | 11 states | 4 | enum value |
| motion_state (S2) | 5 states | 3 | enum value |
| fault_mask | 5 faults | 5 | one bit each: stall, encoder, IMU, no echo, WiFi |
| line_state (S3) | 6 states | 3 | enum value |
| bc_letter (S3) | 44 Code 39 characters + "none" | 6 | character index, 63 = none |
| bc_cmd, bc_dir (S3) | 4 commands; forward or reversed | 2 + 1 | enum value; flag |
| speed_l, speed_r (S2) | -512 to 511 mm/s (design max 400) | 10 each | clamped, offset by 512 |
| count_l, count_r (S2) | 32-bit counters | 16 each | low 16 bits; the decoder takes differences modulo 65536 |
| dist_mm (S2) | -32768 to 32767 mm | 16 | clamped, offset |
| pitch, roll (S4) | -32.0 to +31.5 deg in 0.5 deg steps | 7 each | rounded, clamped, offset |
| hump_state (S4) | 4 states | 2 | enum value |
| peak_mm (S4) | 0 to 255 mm | 8 | clamped |
| hump_count (S4) | 0 to 15 | 4 | saturates |
| motion_class (S4) | 8 classes | 3 | enum value |
| turn_rate (S4) | -512 to 511 deg/s | 10 | clamped, offset |
| imu_ok, cal_done (S4) | yes or no | 1 each | flag |
| closest_mm (S5) | 0 to 1023 mm | 10 | clamped (the ±1 cm target rules out whole cm) |
| angle_idx (S5) | 25 servo positions | 5 | step index |
| width, clear_l, clear_r (S5) | 0 to 1023 mm | 10 each | clamped |
| padding | | 4 | 0 |

196 data bits + 4 padding = 200 bits = 25 B.

### 1.2 The other messages (sections 3A to 6)

| Message | Size | Fields (bits) |
|---|---|---|
| Live, 5 Hz | 14 B | motion state 3, E-stop 1, commanded speed 10, heading 10 (half degrees), line state 3, steering 10, two IR readings 8 + 8, overshoot 1, barcode letter 6, command 3, direction 1, junction turn 2, four S3 counters |
| Heartbeat, 1 Hz | 9 B | seq 8, uptime 16 (s), -RSSI 7, connect count 8, tel_drop 8, evt_drop 8, cmd_reject 8; counts saturate |
| Command (dashboard to robot) | 2 B | opcode 2 (1 START, 2 STOP, 3 RESET; 0 invalid), cmd_id 6 |
| Map (dashboard to robot) | 5 B | cmd_id 6, four letters 6 each (Code 39 index) |
| Ack | 3 B | opcode 2, cmd_id 6, result 2 |
| Event (wire) | 7 B | code 8, two argument bytes, time 24 (ms, modulo) |
| Event record in `mbf_event` | 3 B | code 1 B + 2 argument bytes |

### 1.3 Data shared between tasks

| Shared data | Writers | How access is made safe |
|---|---|---|
| Fault bitmask `flg_fault` | each owner, S1 for bit 4 (WiFi loss) | kernel event flag: `tk_set_flg` / `tk_clr_flg` are single kernel calls, so no read-modify-write in task code; S1 sets and clears only its own level bit (Version B) |
| MQTT driver status | the radio task only | seqlock: readers retry until they get a copy no write overlapped (`mqtt_link_get_status()`) |
| Drop counters (publish queue full, `mbf_event` full) | any task, either core | `atomic_add()` on dual-core builds; dispatch disabled around the increment on single-core |
| Messages between tasks | | kernel message buffers (`mbf_telemetry`, `mbf_cmd`, `mbf_event`): each record is copied in and out whole |
| Messages being packed | `telemetry_task` only | task-local buffers; never shared while being built |

## 2. RAM before and after

Measured with `arm-none-eabi-size` and `arm-none-eabi-nm -S` on the
`NET=lwip MQTT=1` image unless marked computed (S1_binary_frame.md
sections 8 and 9.2).

| Item | Before | After | Source of "after" |
|---|---|---|---|
| Publish record | 214 B | 32 B | `sizeof`, host and target |
| `mbf_telemetry` (8 records) | 1760 B | 288 B | linked image |
| Command record | 116 B | 8 B | `sizeof` (2 B length + 5 B payload + 1 pad, since the MAP) |
| `mbf_cmd` (8 records) | 960 B | 96 B | `g_command_buf` 0x60 in the linked image |
| `mbf_event` (30 records) | 2048 B (computed, Week 6) | 240 B | `g_event_buf` 0xF0 in the linked image (21 Nov 2026) |
| **The three message buffers** | **4768 B** | **624 B** | **-4144 B (-87 %)** |

The driver objects and their two buffers alone, measured in the linked
image before and after: 3392 B -> 728 B (-2664 B), at `d604644` and
`df58a14` (section 8.3).

The trade-off is flash: the codec adds 4124 B of code at -O2 (section
9.2), out of the Pico W's 2 MB.

## 3. Message size before and after

| Message | Before | After | Change |
|---|---|---|---|
| Telemetry payload | 138 B (JSON worst case) | 25 B | -113 B (-82 %) |
| Command payload | up to 16 B (text) | 2 B | -14 B |
| Event on the wire | up to 64 B (text) | 7 B | -57 B |
| Event record between tasks | up to 64 B | 3 B | -61 B |
| One telemetry PUBLISH on the wire, topic `robot/telemetry`, QoS 0 | 158 B | 44 B | -114 B |

Heartbeat (9 B), live (14 B), ack (3 B) and map (5 B) did not exist as
sized messages before (section 9.1).

## 4. Correctness and timing

### 4.1 No loss of function (host tests, latest runs)

| Test | What it proves | Result |
|---|---|---|
| S1-U05 | the codec matches the layout bit for bit: 25 golden vectors, min/max, clamp, wrap and rounding cases, 18 deliberate mutations caught | PASS (506 checks) |
| S1-U09 | the driver enforces the size caps and topic IDs; over-long messages refused and counted | PASS |
| S1-U11 | the dashboard decoder reads every vector exactly as the C codec writes it; event names match S5's table | PASS (40 tests) |
| S1-U12 | the vectors pass through a real broker and decode unchanged | PASS (25/25) |
| S1-U14 | `telemetry_task`'s logic: commands, acks, heartbeat (with RSSI and evt_drop), events, schedules, queue rules, fault bit | PASS (367 checks, 32/32 mutations) |
| S1-U16 part A | the dashboard end to end in simulate mode | PASS |

Dates and commits are in the Results table of `TEST_LOG.md`.

### 4.2 On the robot

| Test | Result |
|---|---|
| S1-U13 (31 Oct 2026): link, status topic, broker restart, hotspot drops, power cut | PASS |
| S1-U06 (10 Nov 2026): commands and heartbeat on the car | heartbeat rate (step 3), bad command rejected (6) and reconnect after a hotspot drop (9) PASS; NFR15 (4) FAIL, below; steps 5, 7, 8 and 10 not run |

### 4.3 Timing

Packing costs microseconds against budgets of hundreds of milliseconds
(S1-U08, on the Pico W, core 1, 20 Oct 2026):

| Operation | Warm mean | Slowest seen | Budget |
|---|---|---|---|
| Pack a telemetry frame | 27 µs | 257 µs | 500 ms period |
| Unpack a telemetry frame | 21 µs | 231 µs | - |
| Pack a command | 1 µs | 3 µs | 300 ms (NFR15) |
| Unpack a command | 1.4 µs | 32 µs | 300 ms (NFR15) |

The worst case, 257 µs, is 0.05 % of the 500 ms telemetry period.

**NFR15 (STOP acked within 300 ms) fails on the car:** 62 % of 50 STOPs
in S1-U06. The packing is not the cause. Unpacking a command takes at
most 32 µs, and a bare Pico W running only the radio over the same phone
hotspot also failed (S1-U18 definition: 80, 58 and 78 % under 300 ms on
2 Nov 2026). The delay is in the WiFi link. The candidate fix, the
radio's power save off, is on branch `s1-pm-off` and waits for S1-U22.
The STOP itself still reaches the controller; only its ack can be late.

## 5. Not covered here

- Other subsystems' fields and RAM: each owner records their own.
- On-car runs of the event channel and RSSI (S1-U30) and of the
  telemetry source mapping (S1-U17): defined, not run yet.
