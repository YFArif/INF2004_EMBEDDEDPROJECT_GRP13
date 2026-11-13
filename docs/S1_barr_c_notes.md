# S1 Barr-C notes

How subsystem 1's C code is checked against the coding standard, which
readings were chosen where the standard leaves room, and what was left as
it is.

## 1. Standard

- Barr Group, *Embedded C Coding Standard*, BARR-C:2018, which the course
  briefing makes mandatory. The PDF is kept locally as
  `docs/reference/barr_c_coding_standard_2018.pdf`; it is git-ignored and
  not in the repository.
- Scope: S1's C files, which are everything in `app_program/subsystem1/`
  except the course template's files (section 4),
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, and the host tests in
  `testing/subsystem1/`.

## 2. Naming scan

A local clang AST scan of every variable and parameter in
`app_program/subsystem1/` and `testing/subsystem1/` checks rules 7.1.e
(names of at least 3 characters), 7.1.k (`p` and `pp` prefixes), 5.6.a
(`bool`, not `BOOL`) and 7.1.m (`b` prefix). The scanner is a local tool
and is not in the repository.

| Point | Hits |
|---|---:|
| origin/main at 7b6c162, before branch s1-barr-c | 45 |
| Branch s1-barr-c | 4 |

The 4 left are all `exinf`, a parameter of the course template's code:
2 in `subsystem1.h` and 2 in `task_led.c` (section 4).

The same branch also aligns declarations and assignments (3.2), adds
parentheses to `&&` operands (1.4.b), fixes comments (2.1.b, 1.6.a),
uses `const` for read-only pointer parameters (1.8.b), makes one
debug-only function private (1.8.a), adds one zero initialiser (7.2.a),
and fixes one mixed-sign comparison (5.3.c). The image (`objcopy -O
binary`) of each of the five build profiles is byte-identical before and
after.

## 3. Readings

Three rules leave room for interpretation. The reading chosen for each:

- **1.5.a, abbreviations.** Taken as advice; 1.5.b, the table, is the
  binding part. Every S1-chosen abbreviation that Appendix A does not
  accept has a row in `docs/abbreviations.md`, and no name was changed.
  Reason: the rule says "should", and the table makes each abbreviation
  easy to look up.
- **6.2.b, each function on a new printed page.** Not applied; there are
  no form feeds. Reason: the rule says "whenever possible", and S1's code
  is reviewed on screen, not on paper.
- **3.3.c, end of file.** Every S1 file ends with
  `/*** end of file ***/`, a newline, and then an empty line. Reason: it
  is the literal reading of "followed by a blank line".

## 4. Left as it is

- **The course template's files:** the template lines of `subsystem1.c`,
  and all of `subsystem1.h`, `subsystem1_test.c`, `task_led.c` and the
  header block of `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h`. They
  keep the template's style (tabs, braces on the same line, no
  end-of-file marker), including the 4 `exinf` hits. Reason: they are
  the course's code, not S1's.
- **S1's lines inside `subsystem1.c`.** These start the MQTT link and
  telemetry_task, and follow that file's template style, including two
  uncommented `(UB*)` casts. Reason: the file is left whole.
- **`tid_led` and `cyc_led`.** These globals lack the `g` and `h`
  prefixes. Reason: they are template names, and `tid_led` is interface
  IF-05 (TEAM.md section 5), so a rename needs team agreement.
- **4.3.b in `mqtt_link.c`:** `mqtt_link_check_table_t`, a compile-time
  check, comes after the static data. Reason: it measures
  `gp_topic_suffix`, which must be declared first.
- **4.3.e in `subsystem1.c`:** `<stdbool.h>` is included in every build
  but used only when `TM_MQTT` is 1. Reason: a template-origin file.
- **`app_program/subsystem5/s5_event.h`:** its comment still names
  `msg_event_t`, which is now `msg_layout_event_t`. Reason: S5's file, so
  S5's owner updates it.
- **Host-test layout:** assignment alignment, parentheses, Doxygen
  blocks, cast comments, literal loop bounds, and the test file names and
  structure (4.1.b, 4.1.d, 4.2.a, 4.3.f). Only names were changed in the
  tests. Reason: test code is not in the image, and this pass kept to the
  production code.
- **"Proposed deviations" in `docs/abbreviations.md`:** `mms`, `dps` and
  `uptime_s` still await approval.
