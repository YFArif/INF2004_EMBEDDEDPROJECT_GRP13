/** @file subsystem1_test.c
 *
 * @brief On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 * @par
 * Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 * console output to testing/subsystem1/logs/ and record the result
 * in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

#define S1_U02_SAMPLES      10      /* LED task states sampled in S1-U02 */

/*!
 * @brief Run subsystem 1's on-target tests (S1-U01, S1-U02).
 */
EXPORT void
subsystem1_test (void)
{
    T_RCYC rcyc;
    T_RTSK rtsk;
    ER     err;

    TEST_INIT();

    /* S1-U01: the cyclic handler exists and is running. */
    err = tk_ref_cyc(gh_cyc_led, &rcyc);
    TEST_CHECK("S1-U01", (E_OK == err) && (rcyc.cycstat & TCYC_STA),
               "cyclic handler is running");

    /* S1-U02: between ticks the LED task is asleep, not busy-looping.
     * Sample it 10 times over ~1 s; it should almost always be waiting.
     */
    {
        INT sample;
        INT waiting = 0;

        for (sample = 0; sample < S1_U02_SAMPLES; sample++)
        {
            tk_dly_tsk(97);     /* not a multiple of the tick */
            if ((E_OK == tk_ref_tsk(gh_tid_led, &rtsk))
                && (rtsk.tskstat & TTS_WAI))
            {
                waiting++;
            }
        }
        TEST_CHECK("S1-U02", waiting >= 9, "LED task sleeps between ticks");
    }

    TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */

/*** end of file ***/

