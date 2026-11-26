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
  (the course template's files there were added later, section 4),
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
replaces tabs with spaces and puts braces on their own lines in S1's
lines of `subsystem1.c` (3.5.a, 1.3.b), uses `const` for read-only
pointer parameters (1.8.b), makes one debug-only function private
(1.8.a), adds one zero initialiser (7.2.a), and fixes one mixed-sign
comparison (5.3.c). The image (`objcopy -O binary`) of each of the five
build profiles is byte-identical before and after.

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

The whole-repository pass on branch `barr-c-2018` (see
`docs/barr_c_2018.md`) has since brought the course template's files in
`app_program/` into the same style: `subsystem1.c`, `subsystem1.h`,
`subsystem1_test.c` and `task_led.c` now use spaces, braces on their own
lines, Doxygen blocks and commented casts; `tid_led` and `cyc_led` are
now `gh_tid_led` and `gh_cyc_led` (IF-05 in TEAM.md is updated);
`exinf` is `p_exinf`; the unused `<stdbool.h>` include in `subsystem1.c`
is gone (4.3.e); and the `s5_event.h` comment names `msg_layout_event_t`.
Every firmware image stayed byte-identical. What is still left:

- **The course template's files outside `app_program/`:** the header
  block of `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h` keeps the
  template's style. Reason: library and kernel code is the course's, not
  the team's.
- **Mixed style in `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`.** S1's
  lines there, which call the WiFi rejoin policy in MQTT=1 builds (branch
  `s1-fr17`, from `244bef8`), use Barr-C style: spaces, braces on their
  own lines, Doxygen blocks and commented casts. The template's lines
  around them keep their own style. Reason: template lines in the library
  are left exactly as they are. (S1's print of the rejoin change there now
  reads the renamed field `to_state`.)
- **4.1.b, first 8 characters of a test's name.** `s1_wifi_rejoin_test`
  shares its first 8 characters (`s1_wifi_`) with `s1_wifi_rejoin`, as
  `s1_telem_test` does with `s1_telem`. Reason: each host test is named
  after the module it tests, and no test is built into the image.
- **4.3.b in `mqtt_link.c`:** `mqtt_link_check_table_t`, a compile-time
  check, comes after the static data. Reason: it measures
  `gp_topic_suffix`, which must be declared first.
- **Host-test layout:** assignment alignment, parentheses, Doxygen
  blocks, cast comments, literal loop bounds, and the test file names and
  structure (4.1.b, 4.1.d, 4.2.a, 4.3.f). Only names were changed in the
  tests. Reason: test code is not in the image, and this pass kept to the
  production code.
- **"Proposed deviations" in `docs/abbreviations.md`:** `mms`, `dps` and
  `uptime_s` still await approval.
