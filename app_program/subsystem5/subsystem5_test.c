/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5. Built only with: make TEST=1
 *
 * @par Bench set-up (testing/subsystem5/TEST_LOG.md)
 * Robot on blocks, wheels free. A flat box about 30 cm straight ahead
 * of the sensor for S5-U01 and S5-U04; nothing else within 1 m.
 *
 * @par
 * The tests never touch the sensor or servo directly: obstacle_task owns
 * them, so the tests send it requests and read its published results,
 * as vehicle_ctrl_task will. Each helper returns its result, because
 * TEST_CHECK only works in the function that called TEST_INIT.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "s5_event.h"
#include "s5_math.h"
#include "s5_profile.h"
#include "s5_scan.h"
#include "subsystem5.h"
#include "subsystem5_test.h"

#if APP_TEST

#define TEST_START_MARGIN_MS (1500u)    /* Servo settle after boot wait. */
#define STATE_WAIT_MS       (300u)      /* Covers one ping cycle.        */
#define RANGE_SAMPLES       (20u)
#define RANGE_GAP_MS        (100u)      /* Longer than one ping cycle.   */
#define RANGE_MIN_VALID     (18u)       /* Of RANGE_SAMPLES.             */
#define RANGE_SPREAD_MM     (20)        /* Max - min, a still target.    */
#define SCAN_WAIT_MS        (8000u)     /* Scan budget ~2.5 s, x 3.      */
#define CAL_WAIT_MS         (14000u)    /* 5 x (2 s hold + move + ping). */

/* S5-U06 fixture: the readings of a simulated 150 mm box 150 mm ahead,
 * with a wall 310 mm to its left (15 degree beam), in scan order.
 */
#define FIXTURE_COUNT       (9u)

static const s5_profile_point_t g_fixture[FIXTURE_COUNT] =
{
    {  30, S5_PROFILE_NO_ECHO }, {  60, 162u }, {  90, 150u },
    { 120, 162u }, { 150, 336u }, {  55, S5_PROFILE_NO_ECHO },
    {  58, 165u }, { 125, 460u }, { 122, 165u }
};

static bool                    math_is_right(void);
static bool                    fixture_scan_is_right(void);
static bool                    fixture_profile_is_right(void);
static subsystem5_obst_state_t state_after_request(uint32_t request);
static bool                    range_is_steady(void);
static bool                    scan_publishes_profile(void);
static s5_scan_step_t          fixture_measure(int16_t angle_deg,
                                               uint16_t * p_dist_mm);
static bool                    is_between(int32_t value, int32_t low,
                                          int32_t high);

/*!
 * @brief Run every subsystem 5 test (called by usermain() in a test
 *        build).
 */
void
subsystem5_test (void)
{
    TEST_INIT();

    // Wait until obstacle_task has finished its own boot wait and set up
    // the sensor and servo, so the tests below find it monitoring.
    subsystem5_delay_ms(S5_BOOT_WAIT_MS + TEST_START_MARGIN_MS);

    // NOTE: The casts are safe: both waits are a few thousand ms.
    s5_event_trace("TEST S5 tests start: boot_wait_ms margin_ms",
                   (int32_t) S5_BOOT_WAIT_MS, (int32_t) TEST_START_MARGIN_MS);

    // S5-U06: pure computation, no hardware needed.
    TEST_CHECK("S5-U06a", math_is_right(),
               "sin/cos, isqrt and Q15 multiply");
    TEST_CHECK("S5-U06b", fixture_scan_is_right(),
               "fixture scan visits 9 angles, finds both edges");
    TEST_CHECK("S5-U06c", fixture_profile_is_right(),
               "fixture profile: width within 10 %, wall on the left");

    // S5-U07: obstacle_task obeys requests.
    TEST_CHECK("S5-U07a", S5_OBST_HOLD == state_after_request(OB_HOLD),
               "OB_HOLD stops monitoring");

    subsystem5_obst_state_t state = state_after_request(OB_RESUME);

    TEST_CHECK("S5-U07b", (S5_OBST_MONITORING == state)
                          || (S5_OBST_SLOW == state),
               "OB_RESUME restarts monitoring");

    // S5-U01 and S5-U04 need the bench target (see file header).
    TEST_CHECK("S5-U01", range_is_steady(),
               "20 pings at a still target: >= 18 echoes, spread <= 2 cm");
    TEST_CHECK("S5-U04", scan_publishes_profile(),
               "scan request publishes a new profile within 8 s");

    // S5-U03: hold 30 ... 150 degrees for 2 s each; check the angles
    // with a protractor and note the printed "CAL" ranges.
    subsystem5_obst_request(OB_CAL_SWEEP);
    subsystem5_delay_ms(CAL_WAIT_MS);

    subsystem5_obst_status_t status = {0};

    subsystem5_obst_status_get(&status);
    TEST_CHECK("S5-U03", S5_OBST_HOLD == status.state,
               "calibration sweep completes (angles checked by hand)");

    // Leave the sensor monitoring, as after power-up.
    subsystem5_obst_request(OB_RESUME);

    TEST_SUMMARY("subsystem5");
}

/*!
 * @brief Check the integer trigonometry and square root.
 *
 * @return true if every value is exact.
 */
static bool
math_is_right (void)
{
    return ((16384 == s5_math_sin_q15(30))
            && (16384 == s5_math_cos_q15(60))
            && (-32768 == s5_math_sin_q15(-90))
            && (0 == s5_math_cos_q15(90))
            && (1000u == s5_math_isqrt(1000000u))
            && (65535u == s5_math_isqrt(0xFFFFFFFFu))
            && (-50 == s5_math_mul_q15(-100, 16384)));
}

/*!
 * @brief Run the scan against the fixture and check where it looked.
 *
 * @return true if the scan finished with both edges where expected.
 */
static bool
fixture_scan_is_right (void)
{
    s5_scan_result_t  result  = {0};
    s5_scan_outcome_t outcome = s5_scan_run(fixture_measure, &result);

    return ((S5_SCAN_DONE == outcome) && (FIXTURE_COUNT == result.count)
            && (result.left.b_found) && (result.right.b_found)
            && (122 == result.left.last_hit.angle_deg)
            && (58 == result.right.last_hit.angle_deg));
}

/*!
 * @brief Build the fixture's profile and check it against the box.
 *
 * @return true if width, path and clearances match the simulated scene.
 */
static bool
fixture_profile_is_right (void)
{
    s5_scan_result_t result  = {0};
    s5_profile_t     profile = {0};

    // The outcome was checked in S5-U06b; the cast discards it here.
    (void) s5_scan_run(fixture_measure, &result);
    s5_profile_build(result.points, result.count, &result.left,
                     &result.right, &profile);

    s5_event_trace("S5-U06 width_mm clear_left_mm", profile.width_mm,
                   profile.clear_left_mm);

    return ((profile.b_on_path)
            && (is_between(profile.width_mm, 135, 165))
            && (is_between(profile.clear_left_mm, 150, 250))
            && (S5_CLEAR_OPEN_MM == profile.clear_right_mm));
}

/*!
 * @brief Send one request and read obstacle_task's state shortly after.
 *
 * @param[in] request One OB_xxx bit.
 *
 * @return The state STATE_WAIT_MS later.
 */
static subsystem5_obst_state_t
state_after_request (uint32_t request)
{
    subsystem5_obst_status_t status = {0};

    subsystem5_obst_request(request);
    subsystem5_delay_ms(STATE_WAIT_MS);
    subsystem5_obst_status_get(&status);

    return (status.state);
}

/*!
 * @brief Sample the live range and check it is steady. Prints the mean
 *        to compare with a ruler (S5-U01: +/-1 cm at 10, 30 and 50 cm).
 *
 * @return true for enough echoes with a small spread.
 */
static bool
range_is_steady (void)
{
    uint32_t valid  = 0u;
    int32_t  sum_mm = 0;
    int32_t  min_mm = INT32_MAX;
    int32_t  max_mm = 0;

    for (uint32_t sample = 0u; sample < RANGE_SAMPLES; sample++)
    {
        subsystem5_obst_status_t status = {0};

        subsystem5_delay_ms(RANGE_GAP_MS);
        subsystem5_obst_status_get(&status);

        if (status.b_range_valid)
        {
            // NOTE: The cast widens uint16_t, so no value is lost.
            int32_t range_mm = (int32_t) status.range_mm;

            valid++;
            sum_mm += range_mm;
            min_mm  = (range_mm < min_mm) ? range_mm : min_mm;
            max_mm  = (range_mm > max_mm) ? range_mm : max_mm;
        }
    }

    // NOTE: The cast is safe: valid is at most RANGE_SAMPLES.
    int32_t mean_mm = (0u == valid) ? 0 : (sum_mm / (int32_t) valid);

    s5_event_trace("S5-U01 mean_mm spread_mm", mean_mm, max_mm - min_mm);

    return ((valid >= RANGE_MIN_VALID)
            && ((max_mm - min_mm) <= RANGE_SPREAD_MM));
}

/*!
 * @brief Request a live scan and wait for the profile. The profile is
 *        printed by obstacle_task; compare its width with a ruler.
 *
 * @return true if a new valid profile arrived in time.
 */
static bool
scan_publishes_profile (void)
{
    s5_profile_t before = {0};
    s5_profile_t after  = {0};
    uint32_t     events = 0u;

    subsystem5_profile_get(&before);
    subsystem5_obst_request(OB_SCAN_REQ);

    bool b_ready = subsystem5_mission_wait(EV_PROFILE_READY, SCAN_WAIT_MS,
                                           &events);

    subsystem5_profile_get(&after);

    return (b_ready && (after.b_valid) && (after.seq == (before.seq + 1u)));
}

/*!
 * @brief Measurement function serving the S5-U06 fixture.
 *
 * @param[in]  angle_deg Requested angle.
 * @param[out] p_dist_mm Fixture range at that angle, or NO_ECHO.
 *
 * @return Always S5_SCAN_STEP_OK.
 */
static s5_scan_step_t
fixture_measure (int16_t angle_deg, uint16_t * p_dist_mm)
{
    *p_dist_mm = S5_PROFILE_NO_ECHO;

    for (uint32_t idx = 0u; idx < FIXTURE_COUNT; idx++)
    {
        if (g_fixture[idx].angle_deg == angle_deg)
        {
            *p_dist_mm = g_fixture[idx].dist_mm;
        }
    }

    return (S5_SCAN_STEP_OK);
}

/*!
 * @brief Tell whether low <= value <= high.
 *
 * @param[in] value The value.
 * @param[in] low   Lower limit.
 * @param[in] high  Upper limit.
 *
 * @return true if inside the limits.
 */
static bool
is_between (int32_t value, int32_t low, int32_t high)
{
    return ((value >= low) && (value <= high));
}

#endif /* APP_TEST */

/*** end of file ***/
