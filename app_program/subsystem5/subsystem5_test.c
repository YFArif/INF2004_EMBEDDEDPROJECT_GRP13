/** @file subsystem5_test.c
 *
 * @brief On-target tests for subsystem 5. Built only with: make TEST=1
 *
 * @par What runs (S5_TEST_MODE, set for one build by s5_test.ps1)
 * - 0 auto  : S5-U06, U07, U09, U10. No set-up; the motors stay still.
 * - 1 bench : auto, then S5-U01, U04, U03. Robot on blocks, a flat box
 *             about 30 cm straight ahead.
 * - 2 stop  : auto, then S5-U08 x3 on the floor: drive at a box, stop,
 *             scan and plan, but do not drive round it.
 * - 3 avoid : auto, then S5-U05 x3 on the floor: the full bypass.
 * The floor runs wait for Button 1 (GP20) before each run, like the S2
 * floor tests ("[S5-STEP]" lines, which s5_test.ps1 reacts to).
 *
 * @par
 * The tests never touch the sensor, servo or motors directly. They send
 * the same requests and commands vehicle_ctrl_task and the dashboard
 * use, and read the published results. Each helper returns its result,
 * because TEST_CHECK only works in the function that called TEST_INIT.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "s5_event.h"
#include "s5_math.h"
#include "s5_plan.h"
#include "s5_profile.h"
#include "s5_scan.h"
#include "subsystem5.h"
#include "subsystem5_test.h"

#if APP_TEST

/* Which tests run; s5_test.ps1 changes this for one build only. */
#define S5_TEST_MODE        (0)

#define MODE_BENCH          (1)
#define MODE_STOP           (2)
#define MODE_AVOID          (3)
#define WITH_BENCH          (MODE_BENCH == S5_TEST_MODE)
#define WITH_FLOOR          ((MODE_STOP == S5_TEST_MODE) \
                             || (MODE_AVOID == S5_TEST_MODE))

#if WITH_FLOOR
# include <tm/tmonitor.h>   /* tm_printf() for the step prompts.        */

# include "drv_button.h"
# include "s5_avoid.h"
# include "s5_links.h"
# include "s5_mission.h"
#endif

#define TEST_START_MARGIN_MS (1500u)    /* Servo settle after boot wait. */
#define STATE_WAIT_MS       (300u)      /* Covers one ping cycle.        */
#define RANGE_SAMPLES       (20u)
#define RANGE_GAP_MS        (100u)      /* Longer than one ping cycle.   */
#define RANGE_MIN_VALID     (18u)       /* Of RANGE_SAMPLES.             */
#define RANGE_SPREAD_MM     (20)        /* Max - min, a still target.    */
#define SCAN_WAIT_MS        (8000u)     /* Scan budget ~2.5 s, x 3.      */
#define CAL_WAIT_MS         (14000u)    /* 5 x (2 s hold + move + ping). */
#define POLL_MS             (50u)
#define FLOOR_RUNS          (3u)
#define STOP_RUN_MS         (30000u)    /* 1 m at 150 mm/s, plus scan.   */
#define AVOID_RUN_MS        (60000u)    /* Approach, scan and bypass.    */
#define DRIVE_ON_MS         (1500u)     /* After the bypass, then STOP.  */
#define PRESS_MS            (50u)
#define COUNTDOWN_S         (3u)
#define MS_PER_S            (1000u)

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
static bool                    plan_is_right(void);
static subsystem5_obst_state_t state_after_request(uint32_t request);
static s5_scan_step_t          fixture_measure(int16_t angle_deg,
                                               uint16_t * p_dist_mm);
static bool                    is_between(int32_t value, int32_t low,
                                          int32_t high);
#if WITH_BENCH
static bool                    range_is_steady(void);
static bool                    scan_publishes_profile(void);
static bool                    cal_sweep_completes(void);
#endif
#if WITH_FLOOR
static uint8_t g_cmd_id = 0u;           /* Numbers the test commands.    */

static bool                    stop_run(uint32_t run);
static bool                    avoid_run(uint32_t run);
static subsystem5_ctrl_state_t wait_state(uint32_t limit_ms,
                                          bool b_after_avoid);
static void                    back_to_ready(void);
static void                    command(uint8_t opcode);
static void                    step_ready(char const * p_next);
#endif

/*!
 * @brief Run the subsystem 5 tests (called by usermain() in a test
 *        build).
 */
void
subsystem5_test (void)
{
    TEST_INIT();

    // Wait until obstacle_task has finished its boot wait and set up the
    // sensor and servo.
    subsystem5_delay_ms(S5_BOOT_WAIT_MS + TEST_START_MARGIN_MS);

    // NOTE: The casts are safe: the mode is 0 ... 3 and the wait 5000 ms.
    s5_event_trace("TEST S5 tests start: mode boot_wait_ms",
                   (int32_t) S5_TEST_MODE, (int32_t) S5_BOOT_WAIT_MS);

    // Automatic checks: no set-up needed.
    TEST_CHECK("S5-U06a", math_is_right(),
               "sin/cos, isqrt and Q15 multiply");
    TEST_CHECK("S5-U06b", fixture_scan_is_right(),
               "fixture scan visits 9 angles, finds both edges");
    TEST_CHECK("S5-U06c", fixture_profile_is_right(),
               "fixture profile: width within 10 %, wall on the left");
    TEST_CHECK("S5-U09", plan_is_right(),
               "planner: continue, bypass left/right, reverse");
    TEST_CHECK("S5-U10", S5_CTRL_READY == subsystem5_ctrl_state_get(),
               "vehicle_ctrl_task reached READY after power-up");

    subsystem5_obst_state_t state = state_after_request(OB_RESUME);

    TEST_CHECK("S5-U07a", (S5_OBST_MONITORING == state)
                          || (S5_OBST_SLOW == state),
               "OB_RESUME starts monitoring");
    TEST_CHECK("S5-U07b", S5_OBST_HOLD == state_after_request(OB_HOLD),
               "OB_HOLD stops monitoring");

#if (MODE_BENCH == S5_TEST_MODE)
    TEST_CHECK("S5-U01", range_is_steady(),
               "20 pings at a still target: >= 18 echoes, spread <= 2 cm");
    TEST_CHECK("S5-U04", scan_publishes_profile(),
               "scan request publishes a new profile within 8 s");
    TEST_CHECK("S5-U03", cal_sweep_completes(),
               "calibration sweep completes (angles checked by hand)");
#elif (MODE_STOP == S5_TEST_MODE)
    for (uint32_t run = 1u; run <= FLOOR_RUNS; run++)
    {
        TEST_CHECK("S5-U08", stop_run(run),
                   "drive at a box: stop, scan and plan");
    }
#elif (MODE_AVOID == S5_TEST_MODE)
    for (uint32_t run = 1u; run <= FLOOR_RUNS; run++)
    {
        TEST_CHECK("S5-U05", avoid_run(run),
                   "drive at a box: stop, scan, bypass, carry on");
    }
#endif

    TEST_SUMMARY("subsystem5");
}

/*!
 * @brief S5-U06a: integer trigonometry and square root.
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
 * @brief S5-U06b: run the scan against the fixture.
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
 * @brief S5-U06c: build the fixture's profile and check it.
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
 * @brief S5-U09: the planner's four decisions on fixed profiles.
 *
 * @return true if every decision and leg is as expected.
 */
static bool
plan_is_right (void)
{
    s5_profile_t profile = {0};
    s5_plan_t    plan    = {S5_PLAN_CONTINUE, 0, 0u, 0u, 0u, 0u};
    bool         b_ok    = true;

    // Wall close on the left (192 < 200 mm needed): go right; the 45
    // degree leg covers a 197 mm shift with about 279 mm.
    profile.b_on_path      = true;
    profile.b_left_edge    = true;
    profile.b_right_edge   = true;
    profile.clear_left_mm  = 192u;
    profile.clear_right_mm = S5_CLEAR_OPEN_MM;
    profile.shift_left_mm  = 197u;
    profile.shift_right_mm = 197u;
    s5_plan_make(&profile, &plan);
    b_ok = b_ok && (S5_PLAN_RIGHT == plan.action) && (-45 == plan.turn_deg)
           && (is_between(plan.out_mm, 270, 290));

    // Both sides wide enough: the larger gap (left).
    profile.clear_left_mm  = 600u;
    profile.clear_right_mm = 400u;
    s5_plan_make(&profile, &plan);
    b_ok = b_ok && (S5_PLAN_LEFT == plan.action) && (45 == plan.turn_deg);

    // Neither side wide enough: reverse and rescan.
    profile.clear_left_mm  = 120u;
    profile.clear_right_mm = 150u;
    s5_plan_make(&profile, &plan);
    b_ok = b_ok && (S5_PLAN_REVERSE == plan.action);

    // Not in the robot's path: carry on.
    profile.b_on_path = false;
    s5_plan_make(&profile, &plan);
    b_ok = b_ok && (S5_PLAN_CONTINUE == plan.action);

    return (b_ok);
}

/*!
 * @brief Send one request to obstacle_task and read its state shortly
 *        after (S5-U07).
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

#if WITH_BENCH
/*!
 * @brief S5-U01: sample the live range for 2 s and check it is steady.
 *        Prints the mean to compare with a ruler.
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

    subsystem5_obst_request(OB_RESUME);
    subsystem5_delay_ms(STATE_WAIT_MS);

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

    subsystem5_obst_request(OB_HOLD);

    // NOTE: The cast is safe: valid is at most RANGE_SAMPLES.
    int32_t mean_mm = (0u == valid) ? 0 : (sum_mm / (int32_t) valid);

    s5_event_trace("S5-U01 mean_mm spread_mm", mean_mm, max_mm - min_mm);

    return ((valid >= RANGE_MIN_VALID)
            && ((max_mm - min_mm) <= RANGE_SPREAD_MM));
}

/*!
 * @brief S5-U04: request a scan and wait for a newer profile. obstacle_task
 *        prints it; compare the width with a ruler.
 *
 * NOTE: The test polls the profile instead of waiting for
 * EV_PROFILE_READY: flg_mission has one waiter, vehicle_ctrl_task.
 *
 * @return true if a new valid profile arrived in time.
 */
static bool
scan_publishes_profile (void)
{
    s5_profile_t before = {0};
    s5_profile_t after  = {0};
    uint32_t     waited = 0u;

    subsystem5_profile_get(&before);
    subsystem5_obst_request(OB_SCAN_REQ);

    do
    {
        subsystem5_delay_ms(POLL_MS);
        waited += POLL_MS;
        subsystem5_profile_get(&after);
    } while ((after.seq == before.seq) && (waited < SCAN_WAIT_MS));

    return ((after.b_valid) && (after.seq == (before.seq + 1u)));
}

/*!
 * @brief S5-U03: slow sweep holding 30 ... 150 degrees for 2 s each.
 *
 * @return true if the sweep finished and the task went back to HOLD.
 */
static bool
cal_sweep_completes (void)
{
    subsystem5_obst_status_t status = {0};

    subsystem5_obst_request(OB_CAL_SWEEP);
    subsystem5_delay_ms(CAL_WAIT_MS);
    subsystem5_obst_status_get(&status);

    return (S5_OBST_HOLD == status.state);
}
#endif /* WITH_BENCH */

#if WITH_FLOOR
/*!
 * @brief S5-U08, one run: drive at a box with the bypass switched off.
 *        The robot should slow down, stop, scan and plan, then finish.
 *
 * @param[in] run Run number, for the log.
 *
 * @return true if the run ended in FINISHED (stopped and planned).
 */
static bool
stop_run (uint32_t run)
{
    // NOTE: The casts are safe: run is 1 ... 3 and the result a small
    // enum; driven distance is a few metres at most.
    s5_event_trace("RUN U08 start: run 0", (int32_t) run, 0);
    step_ready("S5-U08 stop run: robot on the floor 1 m from the box, "
               "pointing straight at it");

    int32_t start_mm = s5_links_odom_dist_mm();

    s5_mission_arm(true);
    s5_avoid_set_bypass(false);
    command(S5_CMD_START);

    subsystem5_ctrl_state_t end = wait_state(STOP_RUN_MS, false);

    s5_event_trace("RESULT U08 state driven_mm", (int32_t) end,
                   s5_links_odom_dist_mm() - start_mm);
    back_to_ready();

    return (S5_CTRL_FINISHED == end);
}

/*!
 * @brief S5-U05, one run: drive at a box and go round it.
 *
 * @param[in] run Run number, for the log.
 *
 * @return true if the bypass finished and line following resumed.
 */
static bool
avoid_run (uint32_t run)
{
    // NOTE: The casts are safe: run is 1 ... 3 and the result a small
    // enum.
    s5_event_trace("RUN U05 start: run 0", (int32_t) run, 0);
    step_ready("S5-U05 bypass run: robot on the floor 1 m from the box, "
               "pointing straight at it, 0.5 m clear on both sides");

    s5_mission_arm(true);
    s5_avoid_set_bypass(true);
    command(S5_CMD_START);

    subsystem5_ctrl_state_t end = wait_state(AVOID_RUN_MS, true);
    bool b_ok = (S5_CTRL_LINE_FOLLOWING == end);

    if (b_ok)
    {
        // Let it drive on a little, then end the run.
        subsystem5_delay_ms(DRIVE_ON_MS);
        command(S5_CMD_STOP);
        subsystem5_delay_ms(STATE_WAIT_MS);
    }

    s5_event_trace("RESULT U05 state ok", (int32_t) end, b_ok ? 1 : 0);
    back_to_ready();

    return (b_ok);
}

/*!
 * @brief Wait for the controller to reach an end of a test run.
 *
 * @param[in] limit_ms      Longest wait; the run is then stopped.
 * @param[in] b_after_avoid true: also end when line following resumes
 *                          after OBSTACLE_AVOIDANCE.
 *
 * @return The state the run ended in.
 */
static subsystem5_ctrl_state_t
wait_state (uint32_t limit_ms, bool b_after_avoid)
{
    subsystem5_ctrl_state_t state   = subsystem5_ctrl_state_get();
    bool                    b_avoid = false;
    bool                    b_end   = false;
    uint32_t                waited  = 0u;

    while ((!b_end) && (waited < limit_ms))
    {
        subsystem5_delay_ms(POLL_MS);
        waited += POLL_MS;
        state   = subsystem5_ctrl_state_get();
        b_avoid = b_avoid || (S5_CTRL_OBSTACLE_AVOIDANCE == state);
        b_end   = (S5_CTRL_FINISHED == state) || (S5_CTRL_FAULT == state)
                  || (b_after_avoid && b_avoid
                      && (S5_CTRL_LINE_FOLLOWING == state));
    }

    if (!b_end)
    {
        // No obstacle seen in time: stop the robot before it goes far.
        s5_event_trace("RUN timed out after ms, stopping", (int32_t) waited,
                       0);
        command(S5_CMD_STOP);
        subsystem5_delay_ms(STATE_WAIT_MS);
    }

    return (state);
}

/*!
 * @brief Bring the controller back to READY for the next run.
 */
static void
back_to_ready (void)
{
    subsystem5_ctrl_state_t state = subsystem5_ctrl_state_get();

    if (S5_CTRL_FAULT == state)
    {
        command(S5_CMD_RESET);
    }
    else if (S5_CTRL_FINISHED == state)
    {
        command(S5_CMD_START);      // From FINISHED, START means READY.
    }
    else
    {
        command(S5_CMD_STOP);
        subsystem5_delay_ms(STATE_WAIT_MS);
        command(S5_CMD_START);
    }

    subsystem5_delay_ms(STATE_WAIT_MS);
}

/*!
 * @brief Send a command as the dashboard would.
 *
 * @param[in] opcode S5_CMD_xxx.
 */
static void
command (uint8_t opcode)
{
    g_cmd_id++;
    CHECK(subsystem5_command_post(opcode, g_cmd_id), "command_post");
}

/*!
 * @brief Announce the next floor step and wait for Button 1, then count
 *        down 3 s (the same "[S2-STEP]"-style prompt as the S2 tests).
 *
 * @param[in] p_next What the next step does.
 */
static void
step_ready (char const * p_next)
{
    uint32_t held = 0u;

    // NOTE: The casts are safe: the strings are ASCII and the numbers
    // are below 10.
    tm_printf((UB *) "\n[S5-STEP] NEXT: %s\n", p_next);
    tm_printf((UB *) "[S5-STEP] press BUTTON 1 (GP20) on the robot when"
              " ready\n");

    // A press left over from before must be let go first.
    while (drv_button_is_down(DRV_BUTTON_START))
    {
        subsystem5_delay_ms(POLL_MS);
    }

    while (held < PRESS_MS)
    {
        subsystem5_delay_ms(POLL_MS);
        held = drv_button_is_down(DRV_BUTTON_START) ? (held + POLL_MS) : 0u;
    }

    while (drv_button_is_down(DRV_BUTTON_START))
    {
        subsystem5_delay_ms(POLL_MS);
    }

    for (uint32_t sec = COUNTDOWN_S; sec > 0u; sec--)
    {
        tm_printf((UB *) "[S5-STEP] %d\n", (int) sec);
        subsystem5_delay_ms(MS_PER_S);
    }

    tm_printf((UB *) "[S5-STEP] GO\n");
}
#endif /* WITH_FLOOR */

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
