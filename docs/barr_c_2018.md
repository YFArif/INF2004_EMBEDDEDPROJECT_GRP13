# BARR-C:2018 compliance

**Result.** The firmware's C code follows the Barr Group *Embedded C
Coding Standard*, BARR-C:2018, which the course makes mandatory. That
covers all 105 files in `app_program/`: the shared files (`app.h`,
`app_main.c`, `test_report.h`) and subsystems S1 to S5. A few rules
could only be met by changing what the code does. In those places the
code was left as it is, and each one is listed in section 5 with the
reason.

**No logic was changed.** Every firmware image built from this code is
byte-for-byte identical to the image built from `main` before the pass.
For test builds, which print source line numbers, this holds once the
line numbers are fixed (section 3).

Contents:
[1. Scope](#1-scope) ·
[2. How it was checked](#2-how-it-was-checked) ·
[3. Proof that no logic changed](#3-proof-that-no-logic-changed) ·
[4. What changed](#4-what-changed) ·
[5. Exceptions](#5-exceptions) ·
[6. Readings](#6-readings) ·
[7. Renamed kernel objects](#7-renamed-kernel-objects)

## 1. Scope

| Part of the repository | Status |
|---|---|
| `app_program/` (55 `.c` and 50 `.h` files): shared code, S1 to S5, on-target tests | Checked rule by rule and brought into line |
| `kernel/`, `lib/`, `include/`, `device/`, `config/`, `etc/`, `build_make/` | Not changed. This is the course's micro T-Kernel 3.0 port, the Pico SDK glue and the build system, not the team's code. The one exception is a line of S1's own WiFi rejoin code in `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`, which uses the renamed field `to_state`. |
| `testing/` (PC-side host tests and tools, not in the firmware) | Only updated where a production name changed (one structure field, see section 4). |

## 2. How it was checked

1. The standard was read in full, rule by rule (sections 1 to 8 and the
   appendices).
2. The work went subsystem by subsystem: shared code, then S1, S2, S3,
   S4 and S5. After each one the firmware was rebuilt and compared with
   `main` (section 3).
3. Two local scanners ran over every source file in all 35 build
   configurations, so code inside `#if` blocks was checked too:
   - a libclang syntax-tree checker: names and prefixes (7.1), type names
     (5.1), fixed-width types (5.2), casts and their comments (1.6),
     parentheses (1.4), `if`/`else` chains (8.2), `switch` layout (3.4.b),
     constants on the left of `==` (8.6), function length (6.2),
     Doxygen headers, macros (6.3) and initialisers (7.2);
   - a text checker: line length (1.2), braces (1.3), comments (2.1),
     white space (3.1 to 3.5) and the end-of-file marker (3.3.c).

   The scanners are local tools and are not in the repository.
4. The Clang static analyzer ran over every configuration, as the
   enforcement note of rule 7.2 asks. It reports no variable used before
   it is initialised. Its only findings are 4 pointers whose initial
   value is replaced before it is read, which rule 7.2.d asks for.
5. GCC ran over every configuration with `-Wall -Wextra -Wshadow
   -Wcast-qual -Wmissing-prototypes -Wstrict-prototypes -Wconversion`.
   It gives the same 9 warnings as on `main`: implicit conversions in S3,
   two `volatile` casts in S1, and one unused variable in S4. One more,
   the missing prototype of `usermain()`, is fixed (6.2.d).
6. Every finding was read by hand. Each one was fixed, or listed as an
   exception (section 5), or explained as a reading of the rule
   (section 6).

## 3. Proof that no logic changed

The 24 firmware configurations the team uses (13 normal builds and 11
test builds) were each built twice: once from `main` (commit `b2b10be`)
and once from this branch. Each image was converted with
`arm-none-eabi-objcopy -O binary`, and the two files were compared byte
by byte with `cmp`.

The test builds print the source line number (`__LINE__`) of a failing
check, so moving a line changes the text inside the image. Each test
build was therefore built a second time with `-D__LINE__=0`, which
leaves only real code changes visible. That makes 35 builds per side.

| Build | What it is | Image SHA-256 (first 16 hex digits), `main` and branch | Result |
|---|---|---|---|
| `default` | Normal firmware (`make`) | `9590e3d9d77a49fc` | Identical |
| `mqtt` | MQTT telemetry (`NET=lwip MQTT=1`) | `b055faf2d45e6112` | Identical |
| `smp0` | Single core (`SMP=0`) | `f3feed0f5d5adbd2` | Identical |
| `smp0-mqtt` | Single core with MQTT | `88dbcd42ae967884` | Identical |
| `uart-nowifi` | UART console, no WiFi | `453a43797574dec1` | Identical |
| `crash` | `CRASH_REPORT=1` | `41048437e2f3241b` | Identical |
| `drive` | S3 drive test (`DRIVE=1`) | `09203cfb945ab39e` | Identical |
| `drive-straight` | S3 straight-line drive test | `926f31d6897fcb01` | Identical |
| `bcverbose` | S3 barcode verbose log | `2f9b6d412bf649c8` | Identical |
| `s1-stack` | MQTT, S1 stack report | `c314ffe13459f63e` | Identical |
| `s1-leave` | MQTT, S1 leave-before-join | `0e29299d4f624d67` | Identical |
| `s3-isr-ram` | S3 interrupt handler in RAM | `5adec27751bb111e` | Identical |
| `s5-nolog-boot` | S5 boot monitoring, no ping log | `1371c099f38fbe22` | Identical |
| `test` | On-target tests (`TEST=1`) | `26624b42b68457b0` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `mqtt-test` | Tests with MQTT | `2f9acc3be36f44ab` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `smp0-test` | Tests on a single core | `dead7865fe560c0d` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s2-bench` | S2 bench tests | `8b2a0d17277e8ca8` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s2-floor1` | S2 floor test 1 | `c08263c0cc774b62` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s2-floor2` | S2 floor test 2 | `2cd1586a638aac9f` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s2-floor2-11` | S2 floor test 2, run 11 | `5760c392b95eed10` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s2-floor3` | S2 floor test 3 | `5ec3b310487c793e` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s5-bench` | S5 bench tests | `475ba1a73ac90a2f` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s5-stop` | S5 stop test | `7504224afa1c7865` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |
| `s5-avoid` | S5 avoidance test | `8451cb94f5288ffa` with `__LINE__` = 0 | Identical with `__LINE__` = 0; as built, only printed line numbers differ |

All 13 normal images are identical as built, and all 11 test images are
identical once the line numbers are fixed. No build gives a warning.

The PC-side host tests give the same results on `main` and on this
branch: S1-U05 (codec), S1-U28 (WiFi rejoin), S1-U14 (telemetry), S2-U09
(motion model) and the S3 host suite. The S3 track simulator fails the
same 5 checks (S3-S05, S3-S14, S3-S25, S3-S35, S3-S92) on `main`, so
those failures come from the simulator's settings, not from this pass.

## 4. What changed

Only form changed: names, comments, white space, parentheses, braces and
the layout of declarations.

| Part | Main changes |
|---|---|
| Shared code | The course template's files were changed from tabs to spaces, with braces on their own lines, Doxygen headers and end-of-file markers. Kernel object IDs got the `gh_` prefix. |
| S1 | The template files (`subsystem1.c`, `subsystem1.h`, `subsystem1_test.c`, `task_led.c`) were brought into S1's style. `tid_led` and `cyc_led` became `gh_tid_led` and `gh_cyc_led`. The 2-letter field `to` became `to_state`; the host test and S1's line in `cyw43_utk.c` were updated to match. Parentheses were added to `&&` and `\|\|` operands. |
| S2 | The template example (`task_producer.c`/`.h`) was rewritten in Barr-C form. `s2_wire.h` got Doxygen headers and cast notes. 15 names were renamed (prefixes `g`, `gh_`, `gb_`, `gp_`). 18 cast comments were added. |
| S3 | 106 names were renamed (short names such as `er`, `i`, `st` and `ch`, and the `gh_`, `gb_` and `gp_` prefixes). 140 cast comments and 28 Doxygen headers were added. Two final `else` branches were added, one assignment was moved out of an `if` test (8.2.c), and the template example (`task_consumer.c`/`.h`) was rewritten. |
| S4 | 46 names were renamed. `float` became `float32_t` (5.4.b.i). The trigonometry polynomials were fully parenthesised. 36 cast comments were added. Two function-like address macros became plain constants (6.3.a). A Doxygen header, a final `else` and a constant on the left of `==` were added. |
| S5 | 23 names were renamed (`gh_` for kernel IDs, `gb_` for booleans, `h_` for task IDs, `_t` on the compile-time check types). 31 operands were parenthesised and 14 cast comments added. |
| All files | Each file now ends with `/*** end of file ***/` followed by a blank line (3.3.c). Lines are at most 80 characters (1.2.a). Comments have no `\` characters (2.1.b) and no trailing white space. |

Documents that name the renamed objects were updated: `TEAM.md`
(interface and kernel object tables), `docs/MQTT.md` and
`docs/S1_barr_c_notes.md`. Test logs and handover notes are records of
past runs and keep the old names; section 7 maps old names to new ones.

## 5. Exceptions

Each of these is left as it is because changing it would change what
the code does or the compiled image.

| Rule | Where | Why it stays |
|---|---|---|
| 6.2.a, functions of at most 100 lines | `s3_line_step` (225 lines), `drive_run` (205), `imu_hump` (144), `next_leg` (134), `pass_decode` (133), `subsystem2_floor_test` (129), `imu_cycle` (121) | Each is one state machine or test sequence. Splitting it into functions changes the code layout and the stack use, so the image would not be identical. |
| 6.2.c, one exit point (preferred practice) | 61 functions with an early `return`, mostly guard clauses in S3 | Removing an early return means restructuring the control flow. |
| 8.2.b, `if`/`else` nested at most two deep | 10 functions: `read_rssi`, `encoder_event`, `wheel_measure`, `s3_line_step`, `drive_run`, `imu_cycle`, `imu_hump`, `imu_side_hump`, `watch_step`, `stress_run` | Flattening them would change the control flow. Each has three levels at one point. |
| 1.7.d, `continue` (preferred practice) | `task_producer.c`, `task_barcode.c`, `task_consumer.c`, `task_drive_test.c` (one each) | Removing it means restructuring the loop. |
| 6.3.a, function-like macros | `CHECK` (`app.h`); `TEST_INIT`, `TEST_CHECK`, `TEST_SUMMARY` (`test_report.h`); `BC_LOG` (`task_barcode.c`); `S3_RAM_FUNC` (`drv_adc.h`); `MM_TO_UM` (`s3_line.c`) | `TEST_CHECK` must print the caller's `__FILE__` and `__LINE__`. `TEST_INIT` declares the caller's counters. `BC_LOG` compiles away when verbose logging is off. `S3_RAM_FUNC` is a section attribute, which a function cannot be. `CHECK` and `MM_TO_UM` as functions would add calls to the image. |
| 7.2.d, pointers initialised to `NULL` | `p_msg` in `task_consumer.c` | The mailbox call writes the pointer before it is read. Adding `= NULL` adds a store to the image. |
| 7.1 and 5.1.a, legacy names | `flg_motion`, `ODOM` and `s2_odom_get()` in `subsystem2.h` | S2 kept these for an older S5 branch. The comment there says to remove them once S5 has switched, and S5 on `main` has. Deleting them is a team decision, not a style change. |
| 5.6.a, `bool` for booleans | `s3_get_steer_sp()` and `s3_barcode_reading()` return the kernel's `BOOL` | They match S2's existing declaration of the S3 interface (`subsystem3.h` explains why). |
| 4.2.a, one header per source file | Source files without a header: `app_main.c`, the `subsystemN_test.c` files, `subsystem2_floor.c`, `subsystem2_step.c`, `task_led.c`, `task_line.c`, `task_barcode.c`, `task_drive_test.c`, `imu_task.c`. Headers without a source file: `app.h`, `test_report.h`, `msg_layout.h`, `s2_wire.h`, `s3_config.h`, `s3_private.h` | The course template sets the file layout: tasks and tests are declared in the subsystem's header or in `app.h`, and shared definitions live in headers of their own. |
| 4.1.b, module names unique in the first 8 characters | `subsystem1` to `subsystem5` and their `_test`, `_floor` and `_step` files all start with `subsyste` | These are the template's file names, which `TEAM.md` and the makefile expect. |
| 4.2.c.i, no `extern` variables in headers (preferred practice) | The template interface IDs (`gh_mbx_samples`, `gh_sem_slots`, `gh_mpf_samples`, `gh_mbf_summary`, `gh_flg_events`, `gh_mtx_stats`, `g_app_stats`, `gh_tid_led`, `gh_cyc_led`), S3's IDs in `s3_api.h`, `flg_motion`; `static inline` functions in `s2_wire.h` | The template publishes each interface ID in the owner's header (TEAM.md section 5). `s2_wire.h` packs one shared word for S2 and S3, and a function call there would change the image. |
| 4.3.b, order of sections in a source file | `mqtt_link.c`: the compile-time check `mqtt_link_check_table_t` comes after the static data | It measures `gp_topic_suffix`, which must be declared first. |
| 5.4.b.v, `isfinite()` after floating-point maths | S4's `s4_math.c` and `imu_task.c` | Adding the checks, and deciding what to do when one fails, adds code and new behaviour. |
| 1.1.a, C99 | The makefile compiles with `-std=gnu11` | The build system is the course's. The code is C99 plus GCC attributes in two places (`weak` on two declarations in `subsystem2.c`, and the section attribute in `S3_RAM_FUNC`), which rule 1.1.c allows when kept to a minimum. |
| 4.2.b, header guard named after the file | `mqtt_config.example.h` uses `MQTT_CONFIG_H` | It is a template that is copied to `mqtt_config.h`, so it carries that file's guard. |
| 4.2.c, declarations belong in a header | `extern UW tm_usb_state(void);` in `subsystem2_test.c` (test builds only) | The function is in the course's USB console code, which has no header for it. Adding one means changing `lib/`, which is not the team's code. |

## 6. Readings

Where the standard leaves room, this is the reading used:

- **1.5, abbreviations.** Rule 1.5.a is advice ("should"). The binding
  part is the table that 1.5.b asks for: S1's table is
  `docs/abbreviations.md`, and S2 to S5 list theirs at the top of
  `subsystemN.h`.
- **1.6.a, cast comments.** One comment can cover the casts of the
  statements right below it, for example a `tm_printf` call or a
  `switch` whose cases all cast the same way. The comment says why each
  value fits.
- **5.2.a, fixed-width types, and bit-fields.** Bit-fields are declared
  `unsigned int` or `signed int`, the only types C99 guarantees for a
  bit-field. The width is given by the field size.
- **5.2.a, `int` for status codes.** The rule asks for a fixed-width
  type wherever the width matters. S3's helpers that return a small
  status code (for example `-1`, `0` or `1` from `recovery_move()`,
  `search_step()` and the `s3_code39` lookups), and the locals that hold
  those codes, keep `int`: the width of a status code does not matter.
- **6.1.i, module prefix on public functions.** A function named exactly
  like its module, for example the task `imu_task` in `imu_task.c`, has
  the prefix. Subsystem N's functions may also use `sN_` or
  `subsystemN_`, as the template does.
- **7.1, kernel objects.** A micro T-Kernel ID (task, flag, mutex,
  buffer, cyclic handler) is a handle: `gh_` for a global, `h_` for a
  local. Kernel types (`ID`, `ER`, `INT`, `UW`, `UB`) are kept wherever
  the code calls the kernel. Cyclic handlers are not interrupt handlers
  (rule 6.1.h does not apply).
- **7.2.a, initialised before use.** A local that a kernel call or a
  helper fills before it is read (for example a `T_RFLG` reference, or
  a struct filled by `take_reading()`) counts as initialised. The static
  analyzer confirms that no variable is read before it is written.
- **3.4.b, `switch` with braces.** A case body in braces, with the brace
  under the `case` and the statements one level in, is indented once
  from the label. `break` stays aligned with the `case`.
- **1.3.b, `do` loops.** `} while (...);` stays on one line, so the end
  of a `do` loop cannot be mistaken for a `while` loop with an empty
  body.
- **3.1.l and 3.1.i, empty third part of a `for`.** `for (idx = 1u;
  (idx + 1u) < count; )` keeps the space after the semicolon (3.1.l),
  which is the more specific rule.
- **3.3.c, end of file.** Each file ends with `/*** end of file ***/`, a
  newline and an empty line, the literal reading of "followed by a blank
  line".

## 7. Renamed kernel objects

Older documents and test logs use the left-hand names. The objects are
the same; only the variable that holds the ID was renamed (the text
printed in error messages is unchanged).

| Old name | New name | Owner |
|---|---|---|
| `tid_led`, `cyc_led` | `gh_tid_led`, `gh_cyc_led` | S1 |
| `sem_slots`, `mpf_samples` | `gh_sem_slots`, `gh_mpf_samples` | S2 (template IF-02) |
| `g_mbf_motion`, `g_mtx_odom`, `g_mtx_steer`, `g_cyc_motion` | `gh_mbf_motion`, `gh_mtx_odom`, `gh_mtx_steer`, `gh_cyc_motion` | S2 |
| `mbx_samples`, `mtx_stats`, `app_stats` | `gh_mbx_samples`, `gh_mtx_stats`, `g_app_stats` | S3 (template IF-01, IF-04) |
| `mbf_nav`, `mtx_s3`, `flg_s3`, `flg_line` | `gh_mbf_nav`, `gh_mtx_s3`, `gh_flg_s3`, `gh_flg_line` | S3 |
| `g_tid_line`, `g_cyc_line` | `gh_tid_line`, `gh_cyc_line` | S3 |
| `mbf_summary`, `flg_events` | `gh_mbf_summary`, `gh_flg_events` | S4 (template IF-03) |
| `g_flg_imu`, `g_cyc_imu`, `g_mtx_terrain` | `gh_flg_imu`, `gh_cyc_imu`, `gh_mtx_terrain` | S4 |
| `g_flg_mission`, `g_flg_fault`, `g_flg_obst`, `g_mtx_obst`, `g_mbf_cmd` | `gh_flg_mission`, `gh_flg_fault`, `gh_flg_obst`, `gh_mtx_obst`, `gh_mbf_cmd` | S5 |
| `SAMPLE_MSG`, `APP_STATS` | `task_producer_sample_t`, `task_consumer_stats_t` | Template types |

