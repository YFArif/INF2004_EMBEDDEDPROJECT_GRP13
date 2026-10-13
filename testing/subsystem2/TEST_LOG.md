# Subsystem 2 — test log

**Subsystem:** Motion control · **Student ID:** 2501261 · **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID     | What is tested                                                                                    | Requirement / interface | Method        | Steps / how                                                                                                                                                                                                               | Expected result                                                                                                                                                                                      |
| ------ | ------------------------------------------------------------------------------------------------- | ----------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| S2-U01 | Encoder ticks per wheel revolution, both wheels                                                   | FR7                     | manual + auto | `make TEST=1`, flash, open the serial port. Follow the prompts: turn ONE wheel by hand exactly 10 revolutions inside the 40 s window (left first, then right), the other wheel still. Repeat the whole run slow and fast. | `[TEST] S2-U01 PASS` for each wheel; the `INFO` line gives ticks per revolution (divide the totals by 10); repeat runs agree within 1%                                                               |
| S2-U06 | Open-loop motor drive: each motor turns its own wheel, forward and reverse, faster at higher duty | FR7                     | manual + auto | Lift the wheels. `make TEST=1`, flash, watch the serial output and the wheels: each motor runs forward, reverse, then forward faster, one at a time. Note whether FORWARD really drives the robot forward.                | `[TEST] S2-U06 PASS` for each motor; ticks/s rise with duty; the other wheel stays still; FORWARD matches the robot's forward direction (otherwise set S2_MOTOR_INVERT_x or swap that motor's wires) |

_Method:_ **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date       | Test ID | Commit    | Tester | Result      | Log / evidence          | Notes |
| ---------- | ------- | --------- | ------ | ----------- | ----------------------- | ----- |
| YYYY-MM-DD | S2-U01  | `abc1234` |        | PASS / FAIL | `logs/YYYY-MM-DD_….txt` |       |
