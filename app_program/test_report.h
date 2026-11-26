/** @file test_report.h
 *
 * @brief Minimal on-target test helpers.
 *
 * @par
 * Every check prints ONE line in a fixed format, so a saved console log
 * can be searched for results (every result line starts with "[TEST]"):
 *
 *     [TEST] S1-U01 PASS  ADC reads in range
 *     [TEST] S1-U02 FAIL  queue rejects overflow   (task_x.c:57)
 *     [TEST] subsystem1 SUMMARY 1 passed, 1 failed
 *
 * @par
 * Usage, inside a test function:
 *     TEST_INIT();
 *     TEST_CHECK("S1-U01", (value >= 0) && (value <= 4095),
 *                "ADC reads in range");
 *     TEST_SUMMARY("subsystem1");
 *
 * @par
 * Test IDs must match the ones in testing/<subsystem>/TEST_LOG.md.
 *
 * @par
 * NOTE: These are macros, not functions (rule 6.3.a deviation).
 * TEST_CHECK() must print the file and line of the check that failed,
 * which only a macro can see, and TEST_INIT() declares the two counters
 * in the calling test function, which TEST_CHECK() and TEST_SUMMARY()
 * then update and print.  The casts are safe: the format strings are
 * ASCII.
 */

#ifndef TEST_REPORT_H
#define TEST_REPORT_H

#include <tm/tmonitor.h>

#define TEST_INIT()                                                     \
    INT test_passed = 0;                                                \
    INT test_failed = 0

#define TEST_CHECK(id, cond, desc)                                      \
    do                                                                  \
    {                                                                   \
        if (cond)                                                       \
        {                                                               \
            test_passed++;                                              \
            tm_printf((UB *)"[TEST] %s PASS  %s\n", (id), (desc));      \
        }                                                               \
        else                                                            \
        {                                                               \
            test_failed++;                                              \
            tm_printf((UB *)"[TEST] %s FAIL  %s   (%s:%d)\n",           \
                      (id), (desc), __FILE__, __LINE__);                \
        }                                                               \
    } while (0)

#define TEST_SUMMARY(name)                                              \
    tm_printf((UB *)"[TEST] %s SUMMARY %d passed, %d failed\n",         \
              (name), test_passed, test_failed)

#endif /* TEST_REPORT_H */

/*** end of file ***/

