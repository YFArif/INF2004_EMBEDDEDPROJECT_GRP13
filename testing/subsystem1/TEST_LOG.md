# Subsystem 1 — test log

**Subsystem:** LED heartbeat (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (not busy) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | LED on GP19 toggles every 250 ms | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |
| S1-U04 | mqtt_link.c public API: publish validation, status seqlock, init ordering (good config, repeat call, queue-creation failure), the single-writer gate between mqtt_link_poll() and start_link(), and last_poll_msec / b_stale staleness including a 32-bit clock wraparound | | host | `testing/subsystem1/mqtt_link_host_test.c` -- build and run commands are in the file's own header comment | `RESULT: all passed`, exit 0 |
| S1-U05 | msg_codec and msg_layout.h: the 18 golden vectors of docs/S1_binary_frame.md 10.2 (encode and decode), the 17 pitch/roll rounding cases (3.3), every field at its minimum and maximum, and the wrap, reconstruction, clamp and reject cases of 10.1; golden.py agrees with the document and the C vectors; 10 deliberate mutations (4 compile-time, 6 run-time) are each caught; msg_codec.c compiles for the target without warnings | docs/S1_binary_frame.md sections 3 to 6 and 10; minimum-bit change requirement (MQTT payloads) | host | From the repository root: `sh testing/subsystem1/run_codec_test.sh` (strict warnings with -Werror, UBSan/ASan; the target compile needs arm-none-eabi-gcc on the PATH) -- stages are described in the script header | Last line `S1-U05: PASS (...)`, exit 0 |
| S1-U07 | mqtt_link.c with the caps from msg_layout.h (MQTT_LINK_PAYLOAD_MAX 25, MQTT_LINK_COMMAND_MAX 2): the S1-U04 cases with the publish boundary at 25 B accepted and 26 B refused; incoming commands through incoming_publish_cb/incoming_data_cb: 2 B whole and as two 1 B fragments reach the queue intact, 3 B whole and as three 1 B fragments are dropped and raise command_dropped by exactly 1, 0 B reaches the queue; after a clean compile of an unmutated copy, changing either cap stops the build at mqtt_link_check_caps_t (2 mutations) | docs/S1_binary_frame.md 5.1, 9.2 and 10.1; replaces S1-U04 | host | From the repository root with PICO_SDK_PATH set: `sh testing/subsystem1/run_link_test.sh` -- stages are described in the script header | Last line `S1-U07: PASS (...)`, exit 0 |
| S1-U08 | msg_codec on the Pico W, standalone image (NET=lwip MQTT=1, placeholder WiFi config): put and get of telemetry and command, 1000 timed calls each, on core 1 and on core 0 at priority 14, with the 1 MHz timer (TIMERAWL): first call after an XIP cache flush (cold, deployed), min/mean/max, warm batch mean, timer overhead, and the CPU share at the real rate (telemetry at 2 Hz of one core; a command against the 300 ms NFR15 budget); the 25 to 40 us telemetry estimate is printed as information. Flash, report only: the image with the codec against the baseline, the placed msg_codec.o sections, the test image, and msg_codec.o at -Os | docs/S1_binary_frame.md section 9; change requirement item 4 (correctness and performance); NFR15 | auto | Build from the repository root with PICO_SDK_PATH set: `sh testing/subsystem1/run_codec_timing.sh`; flash the UF2 it names (BOOTSEL), capture the console as testing/README.md says; gated on core 1 only, core 0 is reported | Core 1: all 8 `[TEST] S1-U08` checks PASS (each of the four operations round-trips in all 1000 calls, and its max is at most 1000 us), then `[TEST] s1u08 SUMMARY 8 passed, 0 failed` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…);
**host** = a standalone program built and run on the development machine
(not on the Pico, no kernel needed) -- see the test file's own header for
exact build/run commands.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-14 | S1-U04 | `01d8fee` | 2502893 | PASS | `logs/2026-10-14_host_test_mqtt_link.txt` | driver differs from b307336 by comments only (ae4758c), builds byte-identical; 4/4 mutations caught |
| 2026-10-16 | S1-U04 | `429c7e9` | 2502893 | PASS | `logs/2026-10-16_host_test_mqtt_link.txt` | constants aligned with Week 6 (back-off 8 s, keep-alive 5 s, connect timeout 5 s, payload 160 B, prefix 32); 5/5 mutations caught (connect timeout via compile-time pin); default and NET=lwip builds byte-identical to b307336 |
| 2026-10-20 | S1-U05 | `1858c7d` | 2502893 | PASS | `logs/2026-10-20_host_test_msg_codec.txt` | 248 checks, 18/18 vectors agree, 10/10 mutations caught (4 compile-time, 6 run-time), target compile ok; Apple clang 17.0.0 with UBSan/ASan, arm-none-eabi-gcc 15.3.1 |
| 2026-10-20 | S1-U07 | `d604644` | 2502893 | PASS | `logs/2026-10-20_host_test_mqtt_link_caps.txt` | host test all passed (13 groups, including the incoming-command and fragment cases); clean compile of the unmutated copy; 2/2 cap mutations caught by mqtt_link_check_caps_t |
| 2026-10-20 | S1-U04 | `d604644` | 2502893 | RETIRED | - | Superseded by S1-U07: from d604644 the caps come from msg_layout.h, and S1-U04's 160/161 B boundary pinned the Week 6 caps (its old test fails exactly that one check against d604644, by design). The earlier S1-U04 rows stand as history; S1-U04 can still be re-run by checking out 429c7e9. |
| 2026-10-20 | S1-U08 | `3814f93` | 2502893 | PASS | `logs/2026-10-20_s1u08_codec_timing.txt` (console), `logs/2026-10-20_s1u08_build.txt` (build) | UF2 sha256 e4892b008e66ef49fca0819fb0b2938d97793bc9bc36c60652ed8bf36f5b0fca. Rounds in the log: 1 and 3 to 9 (8 rounds; round 2 was lost at the console, log not edited). Core 1 gate: 8/8 PASS in every round, 1000/1000 round trips for every operation. Core 1, first call (cold) / warm mean: put_telemetry 165-166 / 26.778-27.444 us; get_telemetry 110-111 / 21.173-21.283 us; put_command 10-11 / 0.954 us; get_command 19 / 1.402 us. Core 1 worst max over all rounds (limit 1000 us): put_telemetry 257 us, get_telemetry 231 us, put_command 3 us, get_command 32 us. Core 0 (reported, not gated) worst max over all rounds: put_telemetry 8303 us, get_telemetry 571 us, put_command 14 us, get_command 66 us. Flash: image with the codec +4124 B text, 0 data, 0 bss at -O2 against the NET=lwip MQTT=1 baseline (339368 -> 343492); msg_codec.o alone at -Os 1774 B. |
