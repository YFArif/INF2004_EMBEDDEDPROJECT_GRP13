# Subsystem 2 — test log

**Subsystem:** Motion control · **Student ID:** 2501261 · **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID     | What is tested                                                                                    | Requirement / interface | Method        | Steps / how                                                                                                                                                                                                               | Expected result                                                                                                                                                                                      |
| ------ | ------------------------------------------------------------------------------------------------- | ----------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| S2-U01 | Encoder ticks per wheel revolution, both wheels                                                   | FR7                     | manual + auto | Set `S2_U01_BENCH` to 1 in `subsystem2_test.c`. `make TEST=1`, flash, open the serial port. Follow the prompts: turn ONE wheel by hand exactly 10 revolutions inside the 40 s window (left first, then right), the other wheel still. Repeat the whole run slow and fast. | `[TEST] S2-U01 PASS` for each wheel; the `INFO` line gives debounced ticks per revolution and the rejected glitch count; repeat runs agree within 1%                                                  |
| S2-U02 | PID step response, FOLLOW 0 -> 250 mm/s with no steering                                          | FR7, NFR4               | auto          | Bench: set `S2_U02_BENCH` to 1, lift the wheels, `make TEST=1`, flash. The CSV lines `[S2-U02] t_ms,speed_l,speed_r,duty_l,duty_r` are the step response (paste into a spreadsheet). Floor version later, at full and low battery. | `[TEST] S2-U02 PASS`: both wheels within +/-25 mm/s (10 %) of 250 mm/s from 0.5 s to 1.5 s                                                                                                          |
| S2-U05 | `MF_ESTOP` -> brake latency                                                                        | NFR6, FR15, IF-05       | auto          | Software part, runs in every `make TEST=1` image with the motors braked: 20 trials of `subsystem2_estop_set()`, latency = timer at the brake minus timer at the set. Hardware part (GPIO toggle + scope on GP8-GP11) still to do. | `[TEST] S2-U05 PASS` twice: 48-52 motion cycles per second, and worst latency <= 20000 us (one motion cycle); the `INFO` lines give the cycle count, the longest cycle (NFR11), and worst/mean latency                                                                                       |
| S2-U06 | Open-loop motor drive: each motor turns its own wheel, forward and reverse, faster at higher duty | FR7                     | manual + auto | Set `S2_U06_BENCH` to 1. Lift the wheels. `make TEST=1`, flash, watch the serial output and the wheels: each motor runs forward, reverse, then forward faster, one at a time. Note whether FORWARD really drives the robot forward. | `[TEST] S2-U06 PASS` for each motor; ticks/s rise with duty; the other wheel stays still; FORWARD matches the robot's forward direction (otherwise set `DRV_MOTOR_INVERT_LEFT/RIGHT` in `drv_motor.h` or swap that motor's wires) |
| S2-U07 | E-stop latch and CLEAR protocol                                                                   | IF-04, IF-05, FR15      | auto          | Runs in every `make TEST=1` image, motors stay braked: set `MF_ESTOP`; send MOVE 100 (must be discarded); send CLEAR while still latched (must be ignored); clear `MF_ESTOP`, send CLEAR then MOVE 0 (both must run). | `[TEST] S2-U07 PASS`; `INFO 4 of 4 steps passed`                                                                                                                                                    |
| S2-U08 | The RP2040 TIMER counts at 1 MHz (all edge times, speeds, the encoder glitch filter and S2-U05 depend on it) | NFR4, NFR6, A4          | auto          | Runs in every `make TEST=1` image: count the timer across a 1000 ms kernel delay. The `INFO` line also shows the WATCHDOG TICK register at boot; `drv_encoder_init()` enforces 12 cycles of the 12 MHz crystal if it is not already set. | `[TEST] S2-U08 PASS`: 970000 ... 1040000 counts                                                                                                                                                      |

_Method:_ **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date       | Test ID | Commit    | Tester | Result      | Log / evidence          | Notes |
| ---------- | ------- | --------- | ------ | ----------- | ----------------------- | ----- |
| YYYY-MM-DD | S2-U01  | `abc1234` |        | PASS / FAIL | `logs/YYYY-MM-DD_….txt` |       |
