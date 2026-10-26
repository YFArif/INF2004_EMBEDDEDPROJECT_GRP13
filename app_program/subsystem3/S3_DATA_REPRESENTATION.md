# Subsystem 3 (Buddy 3, Sean): Minimum-Bit Data Representation (Change Requirement 1)

This covers firmware v14 (v14b after the review fixes). All numbers come from the build in `build_make` (`arm-none-eabi-size`) and the host tests in `testing/subsystem3/host`. Anything not measured is marked **UNVERIFIED**.

## 1. Summary

| Item | v13 (before) | v14 (after) | Change |
|---|---|---|---|
| S3 static RAM (data + bss of s3_*.o, RUN build) | 67,790 B | 9,678 B | −85.7 % |
| Whole-firmware bss (RUN) | 92,260 B | 34,148 B | −58,112 B |
| `S3_NAV_MSG` (S3 → Vehicle Controller) | 12 B | 4 B | −67 % |
| `mbf_nav` buffer (4 messages deep) | 64 B | 32 B | −50 % |
| `S3_STATUS` (telemetry) | 32 B (16-bit steer/pos/n, 32-bit counters, 4-char map) | 16 B | −50 % |
| `steer_sp` (S3 → Motion) | part of the 32 B status, read under a mutex | one 32-bit word, lock-free | — |
| `s3_line_t` (line-follow state) | 80 B | 36 B | −55 % |
| `s3_cal_t` (one sensor's calibration) | 14 B | 8 B | −43 % |
| Digital edge buffer | 64 × 32-bit timestamps = 256 B + 32-bit count | 64 × 16-bit gaps = 128 B + 3 index bytes | −50 % |
| Analogue sample ring (RUN) | 16,384 × 16-bit = 32 KB (8 s) | 4,096 × 8-bit = 4 KB (2 s) | −87.5 % |
| Analogue pass copy `g_win` (RUN) | 16,384 × 16-bit = 32 KB | 4,096 × 8-bit = 4 KB | −87.5 % |
| Analogue extrema table | 640 B | 320 B | −50 % |

Size checks in code: compile-time asserts in `s3_bringup.c` (NAV = 4 B, STATUS = 16 B, line ≤ 40 B), and host test S3-L26 (cal = 8 B, line ≤ 44 B; prints 36).

## 2. Field table

The rule: each field gets the fewest bits that still hold its full range. Signed fields use `INT` (signed) bit-fields.

### IF-01 `S3_NAV_MSG`: 32 bits, copied through message buffer `mbf_nav`

| Field | Range | Bits | Representation |
|---|---|---|---|
| cmd | NONE, LEFT, RIGHT, STRAIGHT, U-TURN (5 values) | 3 | enum. The tribunal sheet said 2 bits, but 5 values need 3 (2 bits holds only 4). |
| ch_idx | 44 Code 39 characters | 6 | index into the Code 39 table; `s3_nav_msg_char()` converts it back to a char |
| reversed | 0/1 | 1 | barcode crossed backwards |
| source | 1 = both paths agree, 2 = analogue, 3 = digital | 2 | |
| seq | 0..15 | 4 | wraps; a gap shows a dropped message |
| odom_mm | 0..65,535 mm | 16 | odometry when the barcode was decoded; wraps every 65.5 m, and receivers use the difference |

### IF-02 `steer_sp`: one aligned 32-bit word, single writer

| Bits | Field | Range |
|---|---|---|
| 0–9 | dv (mm/s, + = turn right) | −200..+200 used; 10-bit signed holds ±511 |
| 10–17 | seq | 0..255, +1 every 10 ms (stops = stale) |
| 18 | valid | 0 when uncalibrated, disabled or searching |

Steering keeps full precision: dv is computed in 32-bit and only the result is packed.

### IF-S3b `S3_STATUS`: 4 × 32 bits = 16 B

| Field | Range | Bits |
|---|---|---|
| steer_dv | ±511 mm/s | 10 signed |
| pos | ±500 ‰ | 10 signed |
| state | 6 states | 3 |
| last_cmd | 5 values | 3 |
| last_rev, overshoot | 0/1 | 1 each |
| n_outer, n_centre | 0..255 (white..black) | 8 each |
| last_ch | 0..43, 63 = none | 6 |
| seq | 0..255 | 8 |
| map_left/right/str/uturn | Code 39 index | 6 each |
| junctions, losts, barcodes | counters, wrap at 256 (receivers use the difference) | 8 each |
| updated_ms | wraps every 65.5 s | 16 |

### IF-07 `flg_line` (Vehicle Controller → S3) and IF-S3c `flg_s3` (S3 → Vehicle Controller)

These are event-flag bitmasks. `flg_line` uses 3 bits (`LF_ENABLE`, `LF_SEARCH_L`, `LF_SEARCH_R`). `flg_s3` uses 4 bits (`JUNCTION`, `LOST`, `BARCODE`, `LINE_FOUND`).

### Internal

| Object | Representation | Why it is enough |
|---|---|---|
| IR samples (barcode, Centre, outer) | 8-bit, `(adc + 8) >> 4` rounded, clamped to 255 | The black/white difference is ≥ 800 counts (12-bit), which is 50 steps at 8-bit. Quantisation error is ≤ 8 counts (0.2 % of full scale). |
| Line sensor pair `g_line_lc` | outer bits 0–7, Centre bits 8–15, one 16-bit store | The ISR writes both with one store, so the task can never read a torn pair. |
| Edge gaps `g_gap[64]` | 16-bit, 16 µs units, saturating | Holds up to 1.048 s. That is above the 300 ms end-of-pass quiet time (RUN) and the 1 s quiet time (desk). The narrowest bar at 0.3 m/s is about 3 ms, which is about 190 units, so resolution is 0.5 %. |
| Edge ring indices | `g_gap_wr` (ISR only) and `g_gap_rd` (task only) in separate bytes | single-producer, single-consumer; no lock |
| `s3_cal_t` | white:12, black:12, valid:1, started:1, filt:12, win_lo:8, win_hi:8 | 12-bit ADC range |
| `s3_line_t` | int16 gains/error, saturating uint16 µm distances (max 65 mm, longer than any rule threshold), state:3, mode:2, req_mode:2, 6 one-bit flags | |
| Activity flags `g_act` | bit0 open, bit1 AOUT, bit2 DOUT | |

## 3. Concurrency rule per shared object

Packing more than one field into a word makes it a shared object, so each one needs exactly one rule.

| Object | Writers | Readers | Rule |
|---|---|---|---|
| `steer_sp` word | line task only | Motion (any core) | One aligned 32-bit store and one 32-bit load. This is atomic on the Cortex-M0+, so no lock is needed. |
| `S3_STATUS` | line task and barcode task | anyone through `s3_get_line_status()` | Every write and every copy is done under `mtx_s3`. Never read the struct directly. |
| `S3_NAV_MSG` | barcode task | Vehicle Controller | The message buffer copies the 4 bytes, so the receiver owns its copy. |
| `g_gap[]` + wr/rd | GPIO ISR (data, then `wr`) | barcode task (`rd`) | SPSC ring: data is stored before the index, both are volatile, and the ISR and task run on the same core. Full ring → newest edge dropped and counted in `g_gap_lost`. |
| `g_glitches`, `g_gap_lost` | GPIO ISR | barcode task | free-running 8-bit counters; the task takes differences |
| `g_ring[]`, `g_idx` | sampler ISR | barcode task | Indices are snapshotted with interrupts off. The copy runs with interrupts on and stays at least 64 samples (32 ms) ahead of the writer (`S3_COPY_GUARD`). |
| `g_line_lc` | sampler ISR | line task | one 16-bit store and one 16-bit load |
| `g_act` | GPIO ISR and sampler ISR (same priority level, so they cannot nest) | barcode task | read and clear with interrupts off |
| `flg_line` | Vehicle Controller | line task (`tk_ref_flg`) | S3 acts only when the requested mode changes (v14b fix; see section 5) |

## 4. Message size (S3 → Vehicle Controller and MQTT)

| | v13 | v14 |
|---|---|---|
| NAV message | 12 B (32-bit seq, 32-bit time, 4 × 8-bit cmd/char/rev/source) | 4 B, and now also carries odometry-at-decode |
| Status for MQTT telemetry | 32 B | 16 B (Buddy 1 can send the raw 16 B as a binary payload) |

The 16 B MQTT figure is UNVERIFIED end to end. It depends on Buddy 1 publishing the struct as binary rather than as JSON text.

## 5. Correctness evidence

| Test | What it proves | Result |
|---|---|---|
| `test_code39` | 8 decode cases (forward, reverse, bad ratio, bad count) | 8/8 pass |
| `test_nav` C01 | all 43 characters decode forwards and backwards at ratio 3 and 2 | pass |
| `test_nav` C02 | 6-bit index ↔ char round trip for all 44 characters | pass |
| `test_analog` | 9 cases on 8-bit samples, including duration overflow | 9/9 pass |
| `test_line` L01–L27, N01 | line following, junction, lost, search, sizes | all pass |
| `test_line` L27 | Regression for the v14 review defect: a found line stays in FOLLOW while `LF_SEARCH_x` is still set (5/5 steps, 0 extra LINE_FOUND), and a new request starts a new search | Passes with the fix. **Fails** with the old raw-pattern logic (checked). |
| Firmware build | modes 0–4 | exit 0, 0 warnings |
| Size asserts | NAV 4 B, STATUS 16 B, line 36 B, cal 8 B | hold on ARM |

All of this is host or compile evidence. **No hardware run of v14 yet**, so the on-robot behaviour is UNVERIFIED.

## 6. Performance

- **RAM:** measured; see section 1.
- **ISR time:** smaller samples remove a 32-bit store per sample, and gaps replace timestamps. The effect on sampler ISR time is **UNVERIFIED**. Measure it by toggling a spare pin and reading the scope (checklist E2).
- **Decode time:** unchanged algorithm, UNVERIFIED on target.

## 7. Deviations from the team tribunal sheet

1. `cmd` is 3 bits, not 2, because there are 5 values including NONE.
2. IF-01 is a message buffer (`mbf_nav`), not a mailbox. It copies 4 bytes, so there is no buffer-lifetime rule.
3. The edge buffer is 128 B (64 × 16-bit), which leaves room for 2 barcodes of 30 edges.
4. The RUN analogue ring is 2 s. That covers a 141 mm barcode at ≥ 0.10 m/s plus margins. Slower passes are flagged `TOO SLOW, start lost`, and the digital path still decodes them.

## 8. Review fixes in v14b

| Severity | Defect | Fix |
|---|---|---|
| HIGH | After LINE FOUND, any change in the flag pattern (for example the Vehicle Controller adding `LF_ENABLE` while `LF_SEARCH_L` was still set) restarted the search. From the wrong side this left the car in SEARCH with steer 0. | `s3_line_request_mode()` acts only when the mapped mode changes. The flag order is now documented in `s3_api.h`. Test L27. |
| LOW | A clipped analogue window started at the slot the ISR writes next | 64-sample guard (`S3_COPY_GUARD`) |
| LOW | Edge-ring overflow was invisible | `dropped=` is printed, and the digital path is skipped when edges were dropped |
| LOW | Truncating 12→8 bit shifted the trigger threshold | rounding `(adc + 8) >> 4`, clamped |
