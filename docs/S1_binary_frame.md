# S1 binary message layout (draft 3)

Buddy 1 (Caleb), subsystem 1: WiFi, MQTT, telemetry and commands.
Status: draft 3, approved by Caleb on 19 Oct 2026 as the basis for the
codec; fields marked guess still need owner confirmation.

Version history:

- Draft 3, 19 Oct 2026: first agreed layout. It is the basis for
  `msg_layout.h`, `msg_codec.h` and `msg_codec.c` in
  `app_program/subsystem1/`.

This document answers the Tribunal 1 change requirement "Minimum-Bit
Data Representation" for the MQTT messages S1 sends and receives. It is
the source for the shared layout header (C), the encoder and decoder, the
host round-trip test and the dashboard decoder. If the code and this
document ever disagree, fix whichever is wrong and bump the layout
version.

## 1. Status of each number

Every field carries one of three statuses:

| Status | Meaning |
|---|---|
| from code | Type or range read from code that is on `main` today. |
| from Version B | Width agreed at Tribunal 1 (Version B tables), or a later team ruling. Not yet in any buddy's code. |
| guess | S1's proposal or an estimate. Needs the owning buddy to confirm. |

Rulings applied (19 Oct 2026, after Tribunal 1):

- Buddy 5 distances are 10 bits in mm, clamped at 1023.
- Buddy 2 speed is 10 bits signed. **Unconfirmed**: `MOTION_CMD.speed`
  says "never negative", but `ODOM.speed_mms` is signed.
- An event is a 1-byte code plus a fixed number of argument bytes. The
  number is a constant in the shared layout header.
- The no-echo and encoder-fault bits of `flg_fault` are reserved (TBD).
- Barr-C deviations are approved by the project manager (not yet named).
  No deviation is expected for this layout code.

## 2. Wire rules (all message types)

1. **Bit stream, MSB first.** Fields are written one after another with
   no gaps. Bit offset 0 is bit 7 (the MSB) of byte 0, bit offset 7 is
   bit 0 of byte 0, bit offset 8 is bit 7 of byte 1, and so on. A field
   that crosses a byte boundary puts its most significant bits in the
   earlier byte. This fixes the byte order, so the RP2040 (little-endian)
   and the dashboard (any language) decode the same way.
2. **No C structs or bit-fields on the wire.** The encoder writes each
   field with masks and shifts on unsigned values (Barr-C 5.3, 5.5). A
   compile-time check pins every message length.
3. **Signed fields use offset binary.** A signed field of width w stores
   `value + 2^(w-1)` as an unsigned number, so the encoder never applies a
   bitwise operator to signed data (Barr-C 5.3b). The decoder subtracts
   `2^(w-1)`. Example: a 10-bit speed of -400 is stored as 112.
4. **Clamp before packing.** A value outside its range is clamped to the
   nearest end, never wrapped, except where a field says "modulo". For a
   signed field that needs a unit change, the order is: convert with
   rounding (3.3), then clamp, then add the offset (rule 3).
5. **Padding bits are 0** and fill the last byte. The decoder ignores them.
6. **Exact lengths.** A received message whose length differs from the
   length for its type, or whose version is not the current one, is
   rejected.

### 2.1 Common header byte (every message)

| Field | Range | Bits | Bit offset | Status | Notes |
|---|---|---|---|---|---|
| version | 0 to 15 | 4 | 0 | from Version B | Layout version. The first agreed layout is 1. 0 is never valid. |
| msg_type | 0 to 15 | 4 | 4 | guess | 1 TELEMETRY, 2 HEARTBEAT, 3 EVENT, 4 COMMAND, 5 ACK. 0 and 6 to 15 reserved. |

Version B asked for "a version byte in each message". Sharing the byte
with the type keeps the 2-byte command at 2 bytes.

## 3. TELEMETRY (robot to dashboard, 2 Hz)

Published by the MQTT code inside `cyw43_utk_task`, from a frame that
`telemetry_task` builds every 500 ms. The field list follows FR12 (robot
state, wheel speeds, encoder counts, line sensor status, last barcode,
hump peak, obstacle profile, distance travelled), plus terrain (IF-08)
and the fault mask. The Week 6 JSON key list (Appendix B) was not
available when this layout was written, so the field list is not a
one-to-one copy of it.

| # | Field | Source | Range (physical -> stored) | Bits | Bit offset | Status | Notes |
|---|---|---|---|---|---|---|---|
| 0 | version, msg_type | S1 | see 2.1 | 8 | 0 | see 2.1 | msg_type = 1 |
| 1 | seq | S1 | 0 to 255, modulo | 8 | 8 | guess | +1 per frame. The dashboard counts gaps as lost frames. |
| 2 | ctrl_state | S5 | 0 to 10 used, 0 to 15 | 4 | 16 | from Version B | 11 flat states in the Week 6 statechart: INIT, CALIBRATING, READY, LINE_FOLLOWING, APPROACH_JUNCTION, EXECUTING_COMMAND, REACQUIRE_LINE, LINE_RECOVERY, OBSTACLE_AVOIDANCE, FAULT, FINISHED. Numbering from B5. |
| 3 | motion_state | S2 | 0 to 5 used, 0 to 7 | 3 | 20 | from Version B | "3 bits, 6 states". The 6 states are not in code yet (`MC_TYPE` has 5 command types, not states). |
| 4 | fault_mask | all | 5 flag bits | 5 | 23 | from Version B | `flg_fault & 0x1F` (`tk_ref_flg`), copied with no remap: integer bit n is fault n, at offset 27 - n (3.1). |
| 5 | line_state | S3 | 0 to 7 | 3 | 28 | from Version B | Values from B3 (TRACKING, JUNCTION, LOST, SEARCHING, SENSOR_FAULT ... in Week 6). |
| 6 | bc_letter | S3 | 0 = none, 1 to 43 = Code 39 symbol | 6 | 31 | from Version B | Last decoded barcode. Symbol order from B3. |
| 7 | bc_cmd | S3 | 0 to 3 | 2 | 37 | from Version B | Barcode command. Meaning of each value from B3. |
| 8 | bc_dir | S3 | 0 or 1 | 1 | 39 | from Version B | `dir_reversed` of NAV_MSG. |
| 9 | speed_l_mms | S2 | -512 to 511 mm/s -> 0 to 1023 | 10 | 40 | from Version B | `ODOM.speed_l_mms` is `W` (from code). Design max 400 mm/s. Signedness unconfirmed. |
| 10 | speed_r_mms | S2 | -512 to 511 mm/s -> 0 to 1023 | 10 | 50 | from Version B | `ODOM.speed_r_mms`, as above. Robot speed = mean of the two, done by the decoder. |
| 11 | count_l | S2 | low 16 bits of `UW`, modulo | 16 | 60 | guess | `ODOM.count_l` is `UW` (from code). See 3.2. |
| 12 | count_r | S2 | low 16 bits of `UW`, modulo | 16 | 76 | guess | As above. |
| 13 | dist_mm | S2 | -32768 to 32767 mm -> 0 to 65535 | 16 | 92 | from Version B | `ODOM.dist_mm` is `W` (from code). "16+ bits". Clamped, not wrapped. Course length unknown. |
| 14 | pitch | S4 | -32.0 to +31.5 deg, 0.5 deg steps -> 0 to 127 | 7 | 108 | from Version B | From IF-08 `pitch_cdeg`: rounded half away from zero (3.3), clamped, then offset by 64. |
| 15 | roll | S4 | -32.0 to +31.5 deg, 0.5 deg steps -> 0 to 127 | 7 | 115 | from Version B | From IF-08 `roll_cdeg`, exactly as pitch (3.3). |
| 16 | hump_state | S4 | 0 to 3 | 2 | 122 | guess | Week 6 hump states: FLAT, CLIMBING, DESCENDING, REPORT. |
| 17 | peak_mm | S4 | 0 to 255 mm | 8 | 124 | from Version B | Run peak hump height. Clamped. |
| 18 | hump_count | S4 | 0 to 15, saturates | 4 | 132 | guess | Number of humps on the course not given in the brief. |
| 19 | motion_class | S4 | 0 to 6 used, 0 to 7 | 3 | 136 | guess | 7 classes in FR19: STATIONARY, MOVING, ACCELERATING, TURNING, CLIMBING_HUMP, DESCENDING_HUMP, IMPACT. |
| 20 | turn_rate | S4 | -512 to 511 deg/s -> 0 to 1023 | 10 | 139 | from Version B | Width from Version B. Unit 1 deg/s, as FR20 states (IF-08's name `turn_rate_cdps` disagrees; 10 bits in cdeg/s would only reach 5 deg/s, and an in-place turn is about 200 deg/s). Derived from the wheel difference, which this frame already carries; it may be dropped (section 7). |
| 21 | imu_ok | S4 | 0 or 1 | 1 | 149 | guess | IF-08 flag. |
| 22 | cal_done | S4 | 0 or 1 | 1 | 150 | guess | IF-08 flag. |
| 23 | closest_mm | S5 | 0 to 1023 mm, clamped | 10 | 151 | from Version B | Ruling: 10 bits mm, clamp 1023. HC-SR04 reads further (about 4 m). |
| 24 | angle_idx | S5 | 0 to 24 used, 0 to 31 | 5 | 161 | from Version B | "5-bit step, 25 positions". **Step size is a guess** (7.5 deg over 0 to 180 deg). See 7. |
| 25 | width_mm | S5 | 0 to 1023 mm, clamped | 10 | 166 | from Version B | |
| 26 | clear_l_mm | S5 | 0 to 1023 mm, clamped | 10 | 176 | from Version B | |
| 27 | clear_r_mm | S5 | 0 to 1023 mm, clamped | 10 | 186 | from Version B | |
| - | padding | - | 0 | 4 | 196 | - | |

**Total: 196 data bits + 4 padding bits = 200 bits = 25 bytes.**

### 3.1 fault_mask bit numbering (field 4)

**Integer bit n of `flg_fault` is fault n.** The field is
`flg_fault & 0x1F` (read with `tk_ref_flg`, masked as an unsigned
value), copied with no remapping, and written MSB first like every other
field (rule 1 of section 2). So integer bit 4 is the first bit on the
wire (offset 23) and integer bit 0 is the last (offset 27).

| `flg_fault` bit n | Value | Fault | Wire offset | Owner sets | Who clears | Kind | Status |
|---|---|---|---|---|---|---|---|
| 0 | 0x01 | motor stall | 27 | S2 | vehicle_ctrl_task only | latched | from Version B |
| 1 | 0x02 | encoder fault | 26 | TBD | TBD | TBD | reserved (TBD) |
| 2 | 0x04 | IMU fault | 25 | S4 | vehicle_ctrl_task only | latched | from Version B |
| 3 | 0x08 | no echo | 24 | TBD | TBD | TBD | reserved (TBD). Week 6 says "not a fault"; the requirement lists it. |
| 4 | 0x10 | WiFi loss | 23 | S1 | S1 | level | from Version B |

The numbering is the 19 Oct ruling. Version B gives Buddy 5 the
`flg_fault` bit positions, so B5 needs to define `flg_fault` with this
same numbering; otherwise `telemetry_task` would have to remap before
copying.

A decoder reads fault n as `(fault_mask >> n) & 1`. The one-hot vectors
TEL-F0 to TEL-F4 in 10.2 check this: an encoder or decoder that reverses
the order fails four of the five (only the middle bit, IMU, is the same
either way round).

Telemetry is only published while connected, so the WiFi-loss bit is
almost always 0 on the wire. It is there for `vehicle_ctrl_task`, and is
sent so the dashboard sees the same mask the robot uses.

### 3.2 Why 16-bit modulo encoder counts

From code: `S2_WHEEL_DIA_MM` 63 and `S2_TICKS_PER_REV_L/R` 639/629
(provisional) give about 0.31 mm per count, so about 3200 counts per
metre. At the 400 mm/s design maximum and 2 Hz telemetry, a wheel moves at
most about 650 counts between frames. A 16-bit count wraps after about
20 m of travel, so the decoder can rebuild the full 32-bit count as long
as fewer than about 20 m pass between two frames it receives (the
difference of two readings, modulo 65536). The robot keeps the full
32-bit counters (the agreed ISR exception). Only the wire copy is cut.

### 3.3 Pitch and roll rounding (fields 14 and 15)

Input: IF-08 `pitch_cdeg` / `roll_cdeg`, signed centidegrees. Output:
signed half-degrees (`hdeg`), then stored as `hdeg + 64` in 7 bits.
Three steps, in this order, all on signed integers (no floating point):

1. **Limit, then round half away from zero.** The input is an
   `int32_t`. It is first limited to -4000 ... +4000 cdeg while still
   signed, so taking the magnitude can never overflow (the absolute value
   of `INT32_MIN` does not fit an `int32_t`). Anything beyond 3225 cdeg
   saturates in step 2 anyway, so the limit changes no result. Then work
   on the magnitude so the rule is the same on both sides of zero (C
   integer division truncates toward zero, so dividing a negative number
   directly would round differently):

   ```
   lim  = cdeg limited to -4000 ... +4000 (in int32_t)
   mag  = |lim|                           (0 ... 4000, cannot overflow)
   hdeg = (mag + 25) / 50                 (integer division)
   if lim < 0: hdeg = -hdeg
   ```
2. **Clamp** `hdeg` to -64 ... +63 (-32.0 deg to +31.5 deg). The range is
   not symmetric: +31.75 deg rounds to +63.5 hdeg -> 64 -> clamped to 63,
   while -31.75 deg rounds to -64 and is in range.
3. **Offset**: store `hdeg + 64` (0 to 127). Only this unsigned value is
   masked and shifted (Barr-C 5.3b).

This conversion is part of the encoder module (so S1-U05 tests it), and
`telemetry_task` calls it when it copies `terrain` into the frame.

Test cases (S1-U05). "stored" is the 7-bit field value.

| cdeg in | Step 1 hdeg | After clamp | Stored | Decoded deg | Why |
|---|---|---|---|---|---|
| 0 | 0 | 0 | 64 | 0.0 | zero |
| +24 | 0 | 0 | 64 | 0.0 | below half, rounds down |
| -24 | 0 | 0 | 64 | 0.0 | below half, rounds toward zero |
| +25 | +1 | +1 | 65 | +0.5 | exact half, away from zero |
| -25 | -1 | -1 | 63 | -0.5 | exact half, away from zero |
| +26 | +1 | +1 | 65 | +0.5 | above half |
| -26 | -1 | -1 | 63 | -0.5 | above half |
| +75 | +2 | +2 | 66 | +1.0 | exact half, away from zero |
| -75 | -2 | -2 | 62 | -1.0 | exact half, away from zero |
| +3150 | +63 | +63 | 127 | +31.5 | largest exact value |
| +3175 | +64 | +63 | 127 | +31.5 | rounds past the top, clamped |
| -3175 | -64 | -64 | 0 | -32.0 | rounds to the bottom, in range |
| -3225 | -65 | -64 | 0 | -32.0 | rounds past the bottom, clamped |
| +9000 | +80 (input limited to 4000) | +63 | 127 | +31.5 | far out of range |
| -9000 | -80 (input limited to -4000) | -64 | 0 | -32.0 | far out of range |
| `INT32_MAX` (2147483647) | +80 (input limited to 4000) | +63 | 127 | +31.5 | largest input, no overflow |
| `INT32_MIN` (-2147483648) | -80 (input limited to -4000) | -64 | 0 | -32.0 | abs() would overflow without the limit |

## 4. HEARTBEAT (robot to dashboard, 1 Hz)

| Field | Range | Bits | Bit offset | Status | Notes |
|---|---|---|---|---|---|
| version, msg_type | see 2.1 | 8 | 0 | see 2.1 | msg_type = 2 |
| seq | 0 to 255, modulo | 8 | 8 | guess | Own counter, separate from telemetry. |
| uptime_s | 0 to 65535 s, saturates | 16 | 16 | guess | About 18 h. |
| rssi_neg | 0 to 127 = -RSSI in dBm, clamped | 7 | 32 | guess | A15: must be read inside the service task. |
| connect_count | 0 to 255, saturates | 8 | 39 | guess | `mqtt_link` status `connect_count`. |
| tel_drop | 0 to 255, saturates | 8 | 47 | guess | Telemetry frames dropped on a full buffer. |
| evt_drop | 0 to 255, saturates | 8 | 55 | guess | Events dropped on a full `mbf_event`. |
| cmd_reject | 0 to 255, saturates | 8 | 63 | guess | Codec rejections plus the driver's `command_dropped` (section 5.1). |
| padding | 0 | 1 | 71 | - | |

**Total: 71 data bits + 1 padding bit = 72 bits = 9 bytes.**

## 5. COMMAND (dashboard to robot) and ACK (robot to dashboard)

Commands are only START, STOP and RESET, so remote input can never steer
the robot. They arrive on `<prefix>/cmd/...` (the driver subscribes to
`<prefix>/cmd/#`).

COMMAND:

| Field | Range | Bits | Bit offset | Status | Notes |
|---|---|---|---|---|---|
| version, msg_type | see 2.1 | 8 | 0 | see 2.1 | msg_type = 4 |
| opcode | 1 START, 2 STOP, 3 RESET; 0 rejected | 2 | 8 | from Version B | "2-bit opcode". 0 is reserved so an all-zero payload is rejected. |
| cmd_id | 0 to 63, modulo | 6 | 10 | guess | Set by the dashboard, echoed in the ack, so it can time each STOP round trip (NFR15). |

**Total: 16 bits = 2 bytes.**

ACK, published on `<prefix>/ack` (not under `cmd/`, which the driver
reserves for incoming commands):

| Field | Range | Bits | Bit offset | Status | Notes |
|---|---|---|---|---|---|
| version, msg_type | see 2.1 | 8 | 0 | see 2.1 | msg_type = 5 |
| opcode | 0 to 3, echoed | 2 | 8 | guess | 0 when the command could not be parsed. |
| cmd_id | 0 to 63, echoed | 6 | 10 | guess | 0 when the command could not be parsed. |
| result | 0 accepted, 1 rejected (bad length, version or opcode), 2 and 3 reserved | 2 | 16 | guess | |
| padding | 0 | 6 | 18 | - | |

**Total: 18 data bits + 6 padding bits = 24 bits = 3 bytes.**

### 5.1 What happens to a bad command

There are two places a command can be refused, and only the second one
can be acked:

| Case | Where it stops | Acked? | Counted in |
|---|---|---|---|
| Payload longer than `MQTT_LINK_COMMAND_MAX` (2 B once the cap comes from the layout header, 9.2) | The driver, before the codec: `incoming_publish_cb` accepts a message only if `total_len <= MQTT_LINK_COMMAND_MAX` (`mqtt_link.c:1344`); otherwise the payload is never copied | **No.** The cmd_id is never read, so there is nothing to echo. The dashboard sees a timeout. | `command_dropped` in the driver status (`mqtt_link.c:1201`) |
| Topic suffix longer than `MQTT_LINK_TOPIC_MAX`, or the command queue full | The driver, same path | No | `command_dropped` (`mqtt_link.c:1201` and `1209`) |
| Shorter than 2 B, wrong version or type, or opcode 0 | The codec, in `telemetry_task` | Yes, result 1. opcode and cmd_id are echoed when the length, version and type are right; otherwise both are 0. | `cmd_reject` (kept by `telemetry_task`) |

The driver does count the oversize case, but in its own
`command_dropped`, which also counts over-long topics and a full queue.
It does not reach the heartbeat by itself. **Proposal:** the heartbeat
`cmd_reject` field sends the sum of the codec rejections and the driver's
`command_dropped` (read with `mqtt_link_get_status()`, saturating at 255).
That needs no driver change. Keeping them apart would need a second
8-bit heartbeat field, which this layout does not include.

## 6. EVENT

Two forms. Inside the robot, every task sends a fixed-size record to
`mbf_event`. On the wire, S1 adds the header and a time stamp.

`mbf_event` record (inside the robot, set in the shared layout header):

| Field | Range | Bytes | Status | Notes |
|---|---|---|---|---|
| code | 0 to 255 | 1 | from Version B (ruling) | Event code table owned by B5 (not written yet). |
| args | fixed | `EVENT_ARG_BYTES` = 2 | guess | Each code defines how it uses the bits. Example: HUMP = peak_mm (8) + hump_count (4) + 4 spare. |

Record = 3 bytes.

EVENT message on the wire:

| Field | Range | Bits | Bit offset | Status | Notes |
|---|---|---|---|---|---|
| version, msg_type | see 2.1 | 8 | 0 | see 2.1 | msg_type = 3 |
| code | 0 to 255 | 8 | 8 | from Version B (ruling) | |
| args | per code | 16 | 16 | guess | `EVENT_ARG_BYTES` x 8. |
| t_ms | 0 to 16 777 215 ms, modulo | 24 | 32 | guess | Robot uptime when `telemetry_task` drains the event (up to about 50 ms after it was sent). About 4.6 h before wrapping. FR19 asks for the time of each motion-class change. |

**Total: 56 bits = 7 bytes.**

## 7. Questions for the owners

| Buddy | Question | Fields |
|---|---|---|
| B2 Arif | Is speed ever negative (reverse, in-place turns)? If not, 9 bits unsigned is enough. Longest run distance? Names of the 6 motion states? | 3, 9, 10, 13 |
| B3 Sean | Line state values, barcode command values, Code 39 symbol order. | 5 to 8 |
| B4 Kessler | Hump states, how many humps, does the getter return natural units? **turn_rate:** Week 6 P5 (and FR20, ω = (v_R − v_L) / b) derives it from the wheel-count difference, which the frame already carries (counts 11 and 12, wheel speeds 9 and 10). The dashboard can compute it from those with b = `S2_TRACK_MM` (112 mm). Does the dashboard need it on the wire at all? If not, field 20 is dropped: 186 data bits + 6 padding = 24 B instead of 25 B, and fields 21 to 27 move 10 bits earlier (imu_ok at 139 ... clear_r_mm at 176). If it stays, confirm the unit (FR20 says deg/s, IF-08 says cdeg/s). | 14 to 22 |
| B5 Anna | Define `flg_fault` with the 3.1 numbering (bit 0 stall, 1 encoder, 2 IMU, 3 no echo, 4 WiFi loss); controller state numbers; event code table; angle step size (25 positions of 7.5 deg, or 5 deg fine steps over a smaller arc?); is 1023 mm clamping fine for the profile? | 2, 4, 23 to 27, events |
| Team | Encoder-fault and no-echo bits: who sets them, latched or level? | 4 |

## 8. Before baseline (19 Oct 2026, commit a25b9f4)

### 8.1 Message sizes before

| Message | Before | How obtained |
|---|---|---|
| Telemetry payload | JSON, worst case 138 B, cap 160 B | Week 6 report (computed there, not measured). |
| Command payload | text, up to 16 B in the design; driver accepts up to 64 B (`MQTT_LINK_COMMAND_MAX`) | Week 6 report; code. |
| Event | text, up to 64 B | Week 6 report (IF-10). |
| Heartbeat, ack | not defined as a size in Week 6 | - |

Size of an MQTT PUBLISH on the wire, from the driver's own formula in
`mqtt_link.c`: 1 type byte + 1 length byte (2 when the rest is 128 B or
more) + 2 + topic length + 2 more for a QoS 1 packet ID + payload. The
topic is `<prefix>/<suffix>`; the driver's worst case
(`PUBLISH_WIRE_MAX`, prefix 32 + suffix 48, QoS 1, payload 160) is 248 B.

### 8.2 RAM and flash before, object level (measured; primary before)

Profile `NET=lwip MQTT=1` (`mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt`),
objects built 16 Oct 2026 04:27 from the sources at 429c7e9 (a25b9f4
changed only the test log). `arm-none-eabi-size`:

| Object | text (flash) | data | bss | Main bss items |
|---|---|---|---|---|
| `mqtt_link.o` | 2454 B | 1 B | 648 B | `g_pending` 214 (one publish record), `g_command` 116, `g_cmd_filter` 82, `g_status_topic` 82, `g_published` 60, `g_status` 60 |
| `mqtt_queue.o` | 560 B | 0 B | 2733 B | `g_publish_buf` 1760 (`mbf_telemetry` storage), `g_command_buf` 960 (`mbf_cmd` storage) |
| **Total** | **3014 B** | **1 B** | **3381 B** | |

These match the computed figures in `mqtt_queue.c`: publish record
214 B, cost 220 B, 8 x 220 = 1760 B; command record 116 B, cost 120 B,
8 x 120 = 960 B; 2720 B for the two buffers.

**In the linked image these objects take 0 B today.** Nothing calls the
driver until Stage 3, so `--gc-sections` drops every section of both
objects (they are listed under "Discarded input sections" in the map).
Image totals for reference: `..._lwip_mqtt.elf` text 339368, data 96,
bss 82356; `..._lwip.elf` text 329880, data 96, bss 82144. The object
sizes above are therefore the primary "before". Section 8.3 gives the
linked-image figure, measured separately.

Not in the build yet (computed only, Week 6 design):

| Item | Before |
|---|---|
| `mbf_event` storage | 2048 B (about 30 text events of 64 B + 4 B header) |
| `telemetry_task` stack | 2048 B (not built; measure the high-water mark) |

S1 static RAM before, **object level plus computed `mbf_event`** (stack
not counted): 3382 B + 2048 B = **5430 B**. This is not a linked-image
figure; the linked-image figure is in 8.3.

### 8.3 Linked-image before (measured 19 Oct 2026)

Goal: an image-level number that includes the driver, without changing
driver code, the makefiles or any tracked file. Unlike 8.2, these are
linked-image figures, not object sizes.

**Result.** Measured with the method below (option A): the `NET=lwip
MQTT=1` profile, built from the exported `pre-minbit` tree with the
placeholder `.example` configs. "Normal" is that profile's own
`_lwip_mqtt.elf`; "rooted" is the same objects linked again with the 13
roots listed in step 4.

| Section | Normal `_lwip_mqtt.elf` | Rooted (13 roots) | Difference |
|---|---|---|---|
| text | 339368 B | 344692 B | +5324 B |
| data | 96 B | 100 B | +4 B |
| bss | 82356 B | 85744 B | +3388 B |
| **Static RAM (data + bss)** | | | **+3392 B** |

The normal ELF from the exported tree is the same size as the
repository's own build at `a25b9f4` (339368 / 96 / 82356). Both driver
buffers are linked: the rooted map places `.bss.g_publish_buf` (1760 B)
and `.bss.g_command_buf` (960 B) in RAM, and neither is under
"Discarded input sections".

- **bss is 7 B more than the object-level 3381 B (8.2).** The new bss
  symbols total 3383 B: the driver's 3381 B plus 2 B of lwIP DNS state
  (`dns_seqno` and `dns_last_pcb_idx`, 1 B each), which the driver pulls
  in through `dns_gethostbyname`. The other 5 B is alignment padding.
- **data is +4 B:** `gb_command_drop` (1 B) plus 3 B of alignment
  padding.
- **text is +5324 B, not the object-level 3014 B:** the driver adds
  2930 B (`mqtt_link.o` 2370 B, slightly under its object size because
  some strings merge with identical ones already in the image;
  `mqtt_queue.o` 560 B); the rest is code the normal image did not have:
  the lwIP DNS client 1492 B, the kernel's `tk_del_mbf` 264 B, C library
  functions 324 B (`strcmp`, `memchr`, `strpbrk`, `strcpy`), other lwIP
  and libtm pieces 312 B, and 2 B of alignment.

**The "after"** will be measured the same way, with the same 13 roots,
once the driver change (caps from the layout header, 9.2) is in. The
codec is not a root, so the difference isolates the change to the
driver; the codec's own size is taken from its object file
(`msg_codec.o`: 4124 B text, 0 data, 0 bss at the profile's `-O2`).

**Tag.** The local annotated tag `pre-minbit` on `a25b9f4` (the last
commit before any minimum-bit change) lets the "before" be rebuilt at
any time after the branch moves on. It was created with:

```
git tag -a pre-minbit a25b9f4 -m "Last commit before the minimum-bit change"
```

It is local only. Pushing it to the team repo is a separate decision.

**Method (option A): a throwaway build linked with extra GC roots.** No
credentials: the build uses the `.example` config headers only, so
nothing real ends up in the image (the objects already in `build_make/`
were compiled with the local `config/wifi_secrets.h`, so they are not
re-used).

1. Export the tagged tree outside the repo folder, without touching the
   repo's `.git`: `git archive pre-minbit | tar -x -C <temporary directory>/pre-minbit`.
2. In that copy only: copy `config/wifi_secrets.example.h` to
   `config/wifi_secrets.h` (placeholder SSID and password), and copy
   `app_program/subsystem1/mqtt_config.example.h` to `mqtt_config.h`,
   changing only `MQTT_LINK_CFG_SET` from 0 to 1 (placeholder broker
   `mqtt.example.com`). The image is never flashed.
3. Build the profile there: `make NET=lwip MQTT=1 -j8`. This gives the
   normal `_lwip_mqtt.elf`.
4. Print that profile's link command (`make -n -B NET=lwip MQTT=1`; the
   line comes from `makefile:83`) and run it again by hand with a second
   output name and map, plus one `-Wl,--undefined=<symbol>` per public
   entry point of both modules (13 roots):
   - `mqtt_link_init`, `mqtt_link_poll`, `mqtt_link_get_status`,
     `mqtt_link_publish`, `mqtt_link_receive_command`;
   - `mqtt_queue_init`, `mqtt_queue_is_ready`, `mqtt_queue_put_publish`,
     `mqtt_queue_get_publish`, `mqtt_queue_put_command`,
     `mqtt_queue_get_command`, `mqtt_queue_full_count`,
     `mqtt_queue_now_msec`.

   A symbol named with `--undefined` is a root for `--gc-sections`. The
   two buffers are `static`, so they cannot be named as roots themselves;
   they are kept because `mqtt_queue_init` refers to them.
5. Check the rooted map: `.bss.g_publish_buf` and `.bss.g_command_buf`
   must appear in the `.bss` output section, not under "Discarded input
   sections". If either is missing, the figure is invalid.
6. `arm-none-eabi-size` both ELFs from step 3 and 4. Linked figure =
   rooted minus normal. Both come from the same objects, so the
   placeholder strings cancel out.

**Checking a result.** The bss difference must be close to the
object-level bss of the two driver objects. Small differences come from
alignment padding and from lwIP or C-library state that only the driver
uses (as above). A gap of more than about 64 B that the two maps do not
explain means the figure must not be used.

Cost of option A: no tracked file changes and no change to the repo's
`.git`; a copy of the tree (a few MB) and one build in a temporary
directory outside the repository, deleted afterwards; the repo's own
`build_make` outputs untouched. Limits: the roots are not real calls, so
the number leaves out `telemetry_task`, its stack, `mbf_event` and the
Stage 3 adapter, and it misses any small saving a real call site would
give through constant propagation.

**What Week 13 will report, labelled "static":**

1. **Static, rooted link (driver only).** Before: the table above, at
   `pre-minbit`. After: the same 13 roots on the commit with the driver
   change. Same recipe, so the difference isolates the change to the
   driver buffers.
2. **Static, Stage 3/4 build.** `pre-minbit` cannot give this "before",
   because no Stage 3/4 code exists at that commit. Instead, once Stage 3
   and 4 exist, the same Stage 3/4 code is built twice: once with the old
   constants (payload 160 B, command 64 B, 64-byte text event records)
   and once with the new ones from the layout header. How the old
   constants are selected for that one build (for example a throwaway
   local edit, never committed) is decided with Stage 4.

Both are static RAM and flash figures from the linker. Stack use is not
in them; `telemetry_task`'s high-water mark is measured separately on
hardware.

**Option B (fallback): a throwaway harness file.** About 30 lines of C
outside the repo that call the public functions of both modules once,
compiled with the profile's `CFLAGS` and added to the same link line
with one `--undefined` root for the harness itself. Same static sizes as
option A, slightly more realistic code generation, but it is more code
to write and keep in step with the compiler flags. Not needed, since
option A worked.

## 9. After, computed from this draft (not measured)

### 9.1 Message sizes

| Message | Before | After | Change |
|---|---|---|---|
| Telemetry payload | 138 B (JSON worst case) | 25 B (24 B without turn_rate) | -113 B (-82 %) |
| Command payload | up to 16 B (design) | 2 B | -14 B |
| Event, on the wire | up to 64 B | 7 B | -57 B |
| Event, `mbf_event` record | up to 64 B | 3 B | -61 B |
| Heartbeat payload | not defined | 9 B | - |
| Ack payload | not defined | 3 B | - |

Example on the wire, topic `robot/telemetry` (15 characters, a guess),
QoS 0: before 1 + 2 + 2 + 15 + 138 = 158 B, after 1 + 1 + 2 + 15 + 25 =
44 B.

### 9.2 RAM, if the driver caps are cut to the largest message

**The caps come from the layout header, not typed numbers.** When the
driver is changed (step 4 of the S1 plan, after this layout is agreed),
`mqtt_link.h` includes the layout header and defines:

```
#define MQTT_LINK_PAYLOAD_MAX   MSG_MAX_BYTES   /* largest message */
#define MQTT_LINK_COMMAND_MAX   MSG_CMD_BYTES
```

and a compile-time check (a negative-size array typedef, the pattern
`mqtt_link.c` and `mqtt_queue.c` already use) stops the build unless:

- every message length (`MSG_TEL_BYTES`, `MSG_HB_BYTES`, `MSG_EVT_BYTES`,
  `MSG_ACK_BYTES`) is at most `MQTT_LINK_PAYLOAD_MAX`;
- `MQTT_LINK_COMMAND_MAX` equals `MSG_CMD_BYTES`;
- `PUBLISH_WIRE_MAX` is still below lwIP's output ring (the existing
  check, unchanged).

So dropping or widening a field changes the buffers automatically, and a
cap that no longer fits a message cannot build. With today's layout the
header gives 25 and 2; the numbers below are those values put through
the record and cost formulas in `mqtt_queue.c` (depth 8, and
`MQTT_LINK_TOPIC_MAX` 48 unchanged). If turn_rate is dropped (section 7),
`MSG_MAX_BYTES` becomes 24: the publish record shrinks to 78 B but still
costs 84 B in the buffer (rounded up to 4), so the totals below do not
change.

| Item | Before | After | How |
|---|---|---|---|
| Publish record | 214 B | 80 B | 49 topic + 1 pad + 2 len + 1 qos + 1 retain + 25 payload = 79, struct aligned to 80 |
| `mbf_telemetry` (8 records) | 1760 B | 672 B | 8 x (4 + 80) |
| Command record | 116 B | 54 B | 49 topic + 1 pad + 2 len + 2 payload |
| `mbf_cmd` (8 records) | 960 B | 480 B | 8 x (4 + 56) |
| `mqtt_link.o` bss | 648 B | 452 B | `g_pending` 214 -> 80, `g_command` 116 -> 54 |
| `mqtt_queue.o` bss | 2733 B | 1165 B | the two buffers + 13 B of IDs and counters |
| `mbf_event` (30 events) | 2048 B | 240 B | 30 x (4 + 4), record 3 B rounded to 4 |
| **S1 static RAM, object level plus computed `mbf_event`** | **5430 B** | **about 1858 B** | Not linked-image figures; see 8.3 for those |

The 49-byte topic now dominates the publish record (49 of 80 bytes).
Cutting `MQTT_LINK_TOPIC_MAX` to fit the real suffixes would save more.
That is a separate decision and is not counted above.

## 10. Tests this layout needs

From both Version B (draft tables) and the submitted Tribunal 1 document.
Test IDs are proposals; they go into `testing/subsystem1/TEST_LOG.md`
before they are run.

| Proposed ID | Test | Success criterion | Source |
|---|---|---|---|
| S1-U05 | Host round trip of every message type; cases listed in 10.1 | Every case in 10.1 gives its expected value; lengths match sections 3 to 6; bad version, type or length rejected | Version B (R3, "encode-decode test") |
| INT (B1 + all) | Encode on the robot, decode on the dashboard, every packet type | Every decoded value matches what was sent; message sizes as in 9.1 | Version B tables |
| S1-U03 (redefined) | STOP over the binary command: 50 trials | Robot stops and the ack arrives in under 300 ms in at least 95 % of trials (NFR15) | Version B tables; Week 6 NFR15 |
| INT (S4 + S1) | `imu_task` run time and lateness before and after packing, with WiFi telemetry running | Well inside 20 ms; never more than 60 ms late (no FIFO overflow) | Submitted document |
| Week 13 | RAM and message size, before and after | Section 8 vs the same measurement after | Change requirement |

### 10.1 S1-U05 cases

Every field is also tested at its exact minimum and maximum. The rows
below are the extra wrap, clamp and reject cases.

Wrap (modulo) fields:

| Field | Input | Expected on decode |
|---|---|---|
| seq (telemetry, heartbeat) | 255, then the next frame | 255, then 0 |
| count_l, count_r | 65535; 65536; 0x12345678 | 65535; 0; 0x5678 |
| count_l, count_r reconstruction | previous full count 65535, received low bits 0 | full count 65536 (difference taken modulo 65536) |
| count_l, count_r reconstruction | previous 0x0001FFF0, received 0x0010 | 0x00020010 |
| t_ms (event) | 16 777 215; 16 777 216; 0x01ABCDEF | 16 777 215; 0; 0xABCDEF |
| cmd_id (command, ack) | 63; 64 | 63; 0 |
| fault_mask | 0xE5 (bits above the 5 defined) | 0x05 (stall + IMU): unknown bits dropped, not clamped to all-faults |
| uptime_s (heartbeat) | 65535; 65536; 70000 | 65535; 65535; 65535 (saturates, as section 4 defines it) |

The reconstruction rows test a reference function in the test file;
the dashboard decoder must use the same rule.

Clamp fields (one value beyond each end that can be exceeded):

| Field | Input | Expected |
|---|---|---|
| ctrl_state | 16; 255 | 15 |
| motion_state, line_state, motion_class | 8 | 7 |
| bc_letter | 64 | 63 |
| bc_cmd, hump_state | 4 | 3 |
| hump_count | 16; 99 | 15 |
| peak_mm | 256; 999 | 255 |
| angle_idx | 32 | 31 |
| closest_mm, width_mm, clear_l_mm, clear_r_mm | 1024; 4000 | 1023 |
| speed_l_mms, speed_r_mms | 512; -513; +/-100000 | 511; -512 |
| dist_mm | 32768; -32769 | 32767; -32768 |
| turn_rate (if kept) | 512; -513 | 511; -512 |
| pitch, roll | the section 3.3 table (rounding, then clamp) | as in 3.3 |
| rssi (heartbeat) | +5 dBm; -128 dBm | 0; -127 |
| connect_count, tel_drop, evt_drop, cmd_reject | 256; 0xFFFFFFFF | 255 |
| opcode (command put), result (ack) | 4 | 3 |

Reject cases: each message one byte short and one byte long; version 0
and version 2; a wrong type in the low nibble; a command with opcode 0
(rejected, but its cmd_id is still returned for the ack); a NULL buffer.

Build checks (mutation, like S1-U04): change one layout constant at a
time (a field position, `MSG_TEL_BYTES`, `MQTT_LINK_COMMAND_MAX`) and
confirm the build stops at the compile-time check.

### 10.2 Golden vectors

One or more fixed byte strings per message type. They were produced by a
Python script (`golden.py`, not yet in the repository) that reads
the field widths and offsets from this document's own tables and packs
each value as a string of 0/1 characters, MSB first. It shares no code
with the C codec, so it is an independent check of both the codec and
the document.

How they are used:

- **C test (S1-U05):** each vector's values are packed with the codec
  and must give exactly these bytes; each byte string is unpacked and
  must give exactly these values.
- **Dashboard decoder:** decodes each byte string and must show these
  values.
- If a field moves or changes width, the vectors change with it, and the
  layout version goes up (rule in 2.1).

Values in wire units: signed values before the offset is added, counts
before the modulo, pitch and roll as both cdeg (input) and hdeg (sent).

| Vector | Field values (wire units) | Bytes (hex, in order) |
|---|---|---|
| TEL-1 typical | seq 7, ctrl 3, motion 1, fault 0x04 (IMU), line 0, letter 11, bc_cmd 2, bc_dir 1, speed_l +300, speed_r -120, count_l 70000 (sent 4464), count_r 0x0001FFFF (sent 65535), dist -1234, pitch -75 cdeg (-2 hdeg), roll +25 cdeg (+1 hdeg), hump_state 2, peak 37, hump_count 3, class 4, turn -45, imu_ok 1, cal_done 1, closest 350, angle 12, width 210, clear_l 400, clear_r 1023 | `11 07 32 40 5D CB 18 81 17 0F FF F7 B2 E7 D0 62 53 8E 9E AF 30 D2 64 3F F0` |
| TEL-2 all minimum | every unsigned field 0, flags 0, every signed field at its minimum (speeds -512, dist -32768, pitch/roll -64 hdeg, turn -512) | `11 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00` |
| TEL-3 all maximum | every field at its largest stored value (speeds 511, dist 32767, pitch/roll +63 hdeg, turn 511, counts 65535) | `11 FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF F0` |
| TEL-F0 | TEL-2 with only fault_mask 0x01 set (flg_fault bit 0, motor stall; wire offset 27) | `11 00 00 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00` |
| TEL-F1 | TEL-2 with only fault_mask 0x02 set (flg_fault bit 1, encoder fault; wire offset 26) | `11 00 00 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00` |
| TEL-F2 | TEL-2 with only fault_mask 0x04 set (flg_fault bit 2, IMU fault; wire offset 25) | `11 00 00 40 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00` |
| TEL-F3 | TEL-2 with only fault_mask 0x08 set (flg_fault bit 3, no echo; wire offset 24) | `11 00 00 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00` |
| TEL-F4 | TEL-2 with only fault_mask 0x10 set (flg_fault bit 4, WiFi loss; wire offset 23) | `11 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00` |
| TEL-INDEP | a second independent calculation from the section 3 table: seq 165, ctrl 9, motion 5, fault 0x01 (stall), line 6, letter 37, bc_cmd 2, bc_dir 1, speed_l -400, speed_r +400, count_l 0x00071234 (sent 0x1234), count_r 0x0009ABCD (sent 0xABCD), dist -1500, pitch +75 cdeg (+2 hdeg), roll -25 cdeg (-1 hdeg), hump_state 2, peak 60, hump_count 3, class 4, turn -90, imu_ok 1, cal_done 0, closest 1500 (clamped to 1023), angle 12, width 250, clear_l 0, clear_r 1023 | `11 A5 9A 1D 2D 1C 39 01 23 4A BC D7 A2 48 4F E3 C3 8D 35 FF B0 FA 00 3F F0` |
| HB-1 | seq 42, uptime 1234 s, rssi -67 dBm (sent 67), connect 3, tel_drop 1, evt_drop 0, cmd_reject 2 | `12 2A 04 D2 86 06 02 00 04` |
| EVT-1 | code 0x21, args 0x25 0x30, t_ms 123456 | `13 21 25 30 01 E2 40` |
| CMD-1 | STOP, id 5 | `14 85` |
| CMD-2 | START, id 0 | `14 40` |
| CMD-3 | RESET, id 63 | `14 FF` |
| CMD-4 | opcode 0 (rejected), id 5 | `14 05` |
| ACK-1 | accepted, answers CMD-1 (STOP id 5) | `15 85 00` |
| ACK-2 | rejected, answers CMD-4 (opcode 0, id 5 echoed) | `15 05 40` |
| ACK-3 | rejected, command unreadable (wrong length, version or type) | `15 00 40` |

TEL-2 is all zeros after the header because every unsigned field is 0
and every signed field at its minimum is stored as 0 (offset binary).
TEL-3 is all ones except the 4 padding bits.
TEL-F0 to TEL-F4 are TEL-2 with one fault bit set, so the only 1 bit
after the header is that fault's wire offset (3.1): stall (`0x01`) is
the last of the five bits (offset 27, `10` in byte 3) and WiFi loss
(`0x10`) the first (offset 23, `01` in byte 2).
TEL-INDEP was not made by `golden.py`: it is a second independent
calculation from the section 3 table. `golden.py` and the C codec both
reproduce it byte for byte.

Check by hand, CMD-1: header `0001 0100` = `14` (version 1, type 4);
then opcode STOP `10` and cmd_id 5 `000101` = `1000 0101` = `85`.
ACK-1: header `15`, the same `85`, then result `00` and six padding
zeros = `00`.

## 11. Other open points found while writing this

- `mqtt_config.example.h` says the topic prefix may be up to 48
  characters, but `mqtt_link.c` caps it at 32 (`PREFIX_MAX_LEN`).
- The retained status payload is text ("online" / "offline",
  `mqtt_link.h`). It is a session marker rather than application data.
  Decide whether to keep it as text or make it one byte.
- The Week 6 IF-10 design put a one-byte topic ID (T, E, A) at the start
  of each `mbf_telemetry` message. The driver's record carries a topic
  string instead, so that byte is not needed.
- Ack topic: the Week 6 report still says `robot/cmd/ack`; this draft
  uses `<prefix>/ack`.
- `uptime_s` saturates at 65535 s (about 18 h) rather than wrapping.
  `t_ms` wraps instead, because events only need time differences.
  If uptime should wrap too, the S1-U05 row in 10.1 changes with it.

