/** @file subsystem3_test.c
 *
 * @brief On-target tests for subsystem 3.  Built only with: make TEST=1
 *
 * @par
 * Record unit results in testing/subsystem3/TEST_LOG.md and the INT-
 * result in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "s3_code39.h"
#include "s3_private.h"
#include "../test_report.h"

#if APP_TEST

/*!
 * @brief Copy the template statistics under their mutex.
 *
 * @return A copy of g_app_stats.
 */
LOCAL task_consumer_stats_t
read_stats (void)
{
    task_consumer_stats_t stats;

    tk_loc_mtx(gh_mtx_stats, TMO_FEVR);
    stats = g_app_stats;
    tk_unl_mtx(gh_mtx_stats);
    return stats;
}

/*!
 * @brief Run subsystem 3's on-target tests (INT-01, S3-U01, S3-U11).
 */
EXPORT void
subsystem3_test (void)
{
    task_consumer_stats_t before;
    task_consumer_stats_t after;

    TEST_INIT();

    /* INT-01 (integration, IF-01): samples from subsystem 2 arrive.
     * At one sample per 300 ms, 2 s should bring at least 5.
     */
    before = read_stats();
    tk_dly_tsk(2000);
    after  = read_stats();
    TEST_CHECK("INT-01", (after.received >= (before.received + 5)),
               "samples flow from subsystem 2 to subsystem 3");

    /* S3-U01: the statistics are self-consistent. */
    TEST_CHECK("S3-U01",
               ((after.received > 0) && (after.min <= after.max) &&
                (after.min >= 0) && (after.max <= 100)),
               "min <= max, both within 0..100");

    /* S3-U11: the day-of letter map changes live, through the same call
     * S1 makes from another task (IF-S3f); a bad map keeps the old one.
     */
    TEST_CHECK("S3-U11",
               (E_OK == s3_set_nav_map("DCBA")) &&
               (S3_NAV_LEFT == s3_nav_lookup('D')) &&
               (S3_NAV_UTURN == s3_nav_lookup('A')) &&
               (E_PAR == s3_set_nav_map("DDBA")) &&
               (S3_NAV_LEFT == s3_nav_lookup('D')) &&
               (E_OK == s3_set_nav_map("ABCD")) &&
               (S3_NAV_LEFT == s3_nav_lookup('A')),
               "map DCBA applied at once, bad map refused, ABCD restored");

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */

/*** end of file ***/

