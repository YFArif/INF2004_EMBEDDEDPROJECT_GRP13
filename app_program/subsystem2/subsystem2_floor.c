/** @file subsystem2_floor.c
 *
 * @brief On-the-floor tests for subsystem 2, built only with make TEST=1
 *        and run only when S2_FLOOR_TEST below is not 0. Each one needs
 *        nothing from the other subsystems.
 *
 * @par Groups (set S2_FLOOR_TEST, rebuild, flash, open the serial port)
 * - 1  S2-U11 distance and straightness: MOVE 1000 x3, MOVE 1000 slow,
 *      MOVE -300. Needs about 1.2 m of clear floor. Measure every run with
 *      a tape: distance travelled and sideways offset.
 * - 2  S2-U03 turn accuracy: TURN +90 x5, -90 x5, 180 x3 (NFR5). Mark the
 *      robot's heading with a strip of tape or a protractor.
 * - 3  S2-U12 braking distance from 300 and 150 mm/s (STOP and E-stop),
 *      the slowest speeds it holds, and stall detection with a wheel held
 *      by hand. Needs about 0.8 m of clear floor.
 *
 * @par
 * Every step starts with a countdown of PLACE_S seconds to measure the
 * last step and put the robot back on its start mark, and ends with one
 * "[S2-FLOOR]" line with the robot's own odometry. Write your tape and
 * protractor readings next to those lines in the saved log.
 *
 * @par Safety
 * The robot moves on its own. Keep a hand near the Robo Pico's power
 * switch and the cable slack. With S2_FLOOR_TEST at 0 nothing here moves.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "subsystem2.h"

#if APP_TEST

#define S2_FLOOR_TEST       (0)     /* 0 off; 1 U11; 2 U03; 3 U12.       */

#define PLACE_S             (10u)   /* Countdown before each step.       */
#define POLL_MS             (20u)
#define SETTLE_MS           (600u)  /* Let it come to rest, then read.   */
#define DONE_TIMEOUT_MS     (15000u)

#define U11_TOL_MM_PERCENT  (2)     /* Own odometry reached the target.  */
#define U11_DRIFT_CDEG      (300)   /* Heading change on a straight run. */
#define U03_TOL_CDEG        (200)   /* Own odometry, per turn.           */

#define U12_RUN_MS          (2000u)     /* Driving before each stop.     */
#define U12_BRAKE_BUDGET_MM (58)        /* NFR6: 150 - 42 - 50 mm.       */
#define U12_SAMPLE_MS       (1000u)     /* Speed averaged over 1 s.      */
#define U12_SLOW_TOL_PCT    (10)
#define U12_STALL_WAIT_MS   (10000u)

/** Odometry difference over one step. */
typedef struct
{
    int32_t dist_mm;
    int32_t dist_l_mm;
    int32_t dist_r_mm;
    int32_t turn_cdeg;
    int32_t time_ms;
    bool    b_done;         /* EV_MOTION_DONE seen (done_count rose).    */
} floor_result_t;

#if (0 != S2_FLOOR_TEST)
static void    countdown(char const * p_next);
static void    travel(subsystem2_cmd_type_t type, int16_t value,
                      uint16_t speed_mms, floor_result_t * p_result);
static int32_t brake_from(uint16_t speed_mms, bool b_estop);
static int32_t hold_speed(uint16_t speed_mms);
static bool    stall_check(void);
static void    send(subsystem2_cmd_type_t type, int16_t value,
                    uint16_t speed_mms);
static int32_t cdeg_diff(int32_t to_cdeg, int32_t from_cdeg);
static int32_t magnitude(int32_t value);
static void    sleep_ms(uint32_t ms);
#endif

/*!
 * @brief Run the floor test group chosen by S2_FLOOR_TEST (called from
 *        subsystem2_test()).
 */
void
subsystem2_floor_test (void)
{
#if (0 != S2_FLOOR_TEST)
#if (1 == S2_FLOOR_TEST)
    floor_result_t result = {0};
    uint32_t       run    = 0u;
#elif (2 == S2_FLOOR_TEST)
    floor_result_t result = {0};
    uint32_t       run    = 0u;
    int32_t        error  = 0;
#else
    int32_t        brake  = 0;
    int32_t        speed  = 0;
#endif

    TEST_INIT();

#if (1 == S2_FLOOR_TEST)
    // S2-U11: three straight runs at cruise, one slow, one reverse.
    for (run = 1u; run <= 3u; run++)
    {
        countdown("MOVE 1000 mm at 300 mm/s: mark the start, ~1.2 m clear");
        travel(S2_CMD_MOVE, 1000, 0u, &result);
        TEST_CHECK("S2-U11", result.b_done &&
                   (magnitude(result.dist_mm - 1000) <=
                    (10 * U11_TOL_MM_PERCENT)) &&
                   (magnitude(result.turn_cdeg) <= U11_DRIFT_CDEG),
                   "MOVE 1000: odometry within 2 %, heading within 3 deg;"
                   " now TAPE the real distance and sideways offset");
    }

    countdown("MOVE 1000 mm at 150 mm/s");
    travel(S2_CMD_MOVE, 1000, 150u, &result);
    TEST_CHECK("S2-U11", result.b_done &&
               (magnitude(result.dist_mm - 1000) <= (10 * U11_TOL_MM_PERCENT)),
               "MOVE 1000 slow: odometry within 2 %; TAPE it");

    countdown("MOVE -300 mm (reverse): clear space BEHIND the robot");
    travel(S2_CMD_MOVE, -300, 0u, &result);
    TEST_CHECK("S2-U11", result.b_done &&
               (magnitude(result.dist_mm + 300) <= 6),
               "MOVE -300: odometry within 2 %; TAPE it");

#elif (2 == S2_FLOOR_TEST)
    // S2-U03: NFR5 wants +/-5 deg at 90 and +/-8 deg at 180, measured by
    // hand; the odometry check here only proves the command completed.
    for (run = 1u; run <= 13u; run++)
    {
        int16_t angle = 90;

        if (run > 10u)
        {
            angle = 180;
        }
        else if (run > 5u)
        {
            angle = -90;
        }
        else
        {
            // First five: left.
        }

        countdown((angle > 0) ? "TURN left: line the robot up on the tape"
                              : "TURN right: line the robot up on the tape");
        travel(S2_CMD_TURN, angle, 0u, &result);
        error = result.turn_cdeg - (angle * 100);
        TEST_CHECK("S2-U03", result.b_done &&
                   (magnitude(error) <= U03_TOL_CDEG),
                   "TURN: odometry within 2 deg; now MEASURE the real angle");
    }

#elif (3 == S2_FLOOR_TEST)
    // S2-U12 a: braking distance. NFR6 leaves 58 mm for braking.
    countdown("FOLLOW 300 mm/s for 2 s, then STOP: ~0.7 m clear");
    brake = brake_from(300u, false);
    TEST_CHECK("S2-U12", brake <= U12_BRAKE_BUDGET_MM,
               "STOP from 300 mm/s within 58 mm (NFR6 budget)");

    countdown("FOLLOW 300 mm/s for 2 s, then E-STOP");
    brake = brake_from(300u, true);
    TEST_CHECK("S2-U12", brake <= U12_BRAKE_BUDGET_MM,
               "MF_ESTOP from 300 mm/s within 58 mm (NFR6 budget)");

    countdown("FOLLOW 150 mm/s for 2 s, then STOP");
    (void) brake_from(150u, false);

    // S2-U12 b: slow speeds. 150 is S5's half speed near an obstacle.
    countdown("slow speeds: 40, 60, 100, 150 mm/s for 2 s each (~0.7 m)");
    (void) hold_speed(40u);
    (void) hold_speed(60u);
    (void) hold_speed(100u);
    speed = hold_speed(150u);
    send(S2_CMD_STOP, 0, 0u);
    TEST_CHECK("S2-U12", magnitude(speed - 150) <= ((150 * U12_SLOW_TOL_PCT)
                                                     / 100),
               "holds 150 mm/s within 10 %");

    // S2-U12 c: stall detection with a wheel held.
    countdown("STALL: FOLLOW 150, then HOLD the LEFT wheel firmly");
    TEST_CHECK("S2-U12", stall_check(),
               "held wheel: STALL, braked and MF_ESTOP within 1 s");
#endif

    TEST_SUMMARY("subsystem2 floor");
#endif /* 0 != S2_FLOOR_TEST */
}

#if (0 != S2_FLOOR_TEST)
/*!
 * @brief Print what comes next and count down PLACE_S seconds.
 *
 * @param[in] p_next What the next step does.
 */
static void
countdown (char const * p_next)
{
    uint32_t sec = 0u;

    // NOTE: The casts are safe: the format strings are ASCII and the
    // seconds are below 60.
    tm_printf((UB *) "\n[S2-FLOOR] NEXT: %s\n", p_next);

    for (sec = PLACE_S; sec > 0u; sec--)
    {
        if (sec <= 3u)
        {
            tm_printf((UB *) "[S2-FLOOR] %d\n", (int) sec);
        }

        sleep_ms(1000u);
    }

    tm_printf((UB *) "[S2-FLOOR] GO\n");
}

/*!
 * @brief Run one MOVE or TURN and report the odometry change.
 *
 * @param[in]  type      S2_CMD_MOVE or S2_CMD_TURN.
 * @param[in]  value     mm or degrees.
 * @param[in]  speed_mms 0 = default.
 * @param[out] p_result  What the robot's own odometry saw.
 */
static void
travel (subsystem2_cmd_type_t type, int16_t value, uint16_t speed_mms,
        floor_result_t * p_result)
{
    subsystem2_odom_t   before = {0};
    subsystem2_odom_t   after  = {0};
    subsystem2_status_t status = {0};
    uint32_t            done   = 0u;
    uint32_t            waited = 0u;

    subsystem2_status_get(&status);
    done = status.done_count;
    subsystem2_odom_get(&before);
    send(type, value, speed_mms);

    do
    {
        sleep_ms(POLL_MS);
        waited += POLL_MS;
        subsystem2_status_get(&status);
    } while ((status.done_count == done) && (waited < DONE_TIMEOUT_MS));

    // NOTE: The cast is safe: waited is at most 15000.
    p_result->time_ms = (int32_t) waited;
    p_result->b_done  = (status.done_count != done);

    if (!p_result->b_done)
    {
        send(S2_CMD_STOP, 0, 0u);
    }

    sleep_ms(SETTLE_MS);
    subsystem2_odom_get(&after);

    p_result->dist_mm   = after.dist_mm - before.dist_mm;
    p_result->dist_l_mm = after.dist_l_mm - before.dist_l_mm;
    p_result->dist_r_mm = after.dist_r_mm - before.dist_r_mm;
    p_result->turn_cdeg = cdeg_diff(after.heading_cdeg, before.heading_cdeg);

    // NOTE: The casts are safe: the format string is ASCII and every
    // value is far inside int; the heading is printed as a sign, whole
    // degrees and hundredths.
    tm_printf((UB *) "[S2-FLOOR] %s %d: odometry %d mm (L %d R %d),"
              " heading change %c%d.%02d deg, %d ms%s\n",
              (S2_CMD_TURN == type) ? "TURN" : "MOVE", (int) value,
              (int) p_result->dist_mm, (int) p_result->dist_l_mm,
              (int) p_result->dist_r_mm,
              (p_result->turn_cdeg < 0) ? '-' : '+',
              (int) (magnitude(p_result->turn_cdeg) / 100),
              (int) (magnitude(p_result->turn_cdeg) % 100),
              (int) p_result->time_ms,
              p_result->b_done ? "" : "  (TIMED OUT, stopped)");
}

/*!
 * @brief Drive at a speed, then STOP or E-stop, and measure the travel
 *        from the stop request until the robot is at rest.
 *
 * @param[in] speed_mms Speed before the stop.
 * @param[in] b_estop   true: MF_ESTOP; false: STOP command.
 *
 * @return Braking distance in mm.
 */
static int32_t
brake_from (uint16_t speed_mms, bool b_estop)
{
    subsystem2_odom_t at_stop = {0};
    subsystem2_odom_t at_rest = {0};
    int32_t           brake   = 0;

    subsystem2_steer_set(0);
    send(S2_CMD_FOLLOW, 0, speed_mms);
    sleep_ms(U12_RUN_MS);

    subsystem2_odom_get(&at_stop);
    if (b_estop)
    {
        subsystem2_estop_set();
    }
    else
    {
        send(S2_CMD_STOP, 0, 0u);
    }

    sleep_ms(1000u);
    subsystem2_odom_get(&at_rest);
    brake = at_rest.dist_mm - at_stop.dist_mm;

    // NOTE: The casts are safe: the format string is ASCII and the
    // values are far inside int.
    tm_printf((UB *) "[S2-FLOOR] %s from %d mm/s (measured %d): braking"
              " distance %d mm (budget %d)\n", b_estop ? "E-STOP" : "STOP",
              (int) speed_mms, (int) at_stop.speed_mms, (int) brake,
              (int) U12_BRAKE_BUDGET_MM);

    if (b_estop)
    {
        subsystem2_estop_clear();
        send(S2_CMD_CLEAR, 0, 0u);
        sleep_ms(100u);
    }

    return (brake);
}

/*!
 * @brief FOLLOW at a speed for 2 s and report the mean measured speed and
 *        duty over the last second.
 *
 * @param[in] speed_mms The set-point.
 *
 * @return Mean measured robot speed, mm/s.
 */
static int32_t
hold_speed (uint16_t speed_mms)
{
    subsystem2_odom_t   odom    = {0};
    subsystem2_status_t status  = {0};
    int32_t             sum_l   = 0;
    int32_t             sum_r   = 0;
    int32_t             sum_dl  = 0;
    int32_t             sum_dr  = 0;
    int32_t             samples = 0;
    uint32_t            waited  = 0u;

    send(S2_CMD_FOLLOW, 0, speed_mms);
    sleep_ms(U12_RUN_MS - U12_SAMPLE_MS);

    for (waited = 0u; waited < U12_SAMPLE_MS; waited += POLL_MS)
    {
        sleep_ms(POLL_MS);
        subsystem2_odom_get(&odom);
        subsystem2_status_get(&status);
        sum_l  += odom.speed_l_mms;
        sum_r  += odom.speed_r_mms;
        sum_dl += status.duty_l;
        sum_dr += status.duty_r;
        samples++;
    }

    // NOTE: The casts are safe: the format string is ASCII and the means
    // are far inside int.
    tm_printf((UB *) "[S2-FLOOR] FOLLOW %d mm/s: measured L %d R %d mm/s,"
              " duty L %d R %d permille\n", (int) speed_mms,
              (int) (sum_l / samples), (int) (sum_r / samples),
              (int) (sum_dl / samples), (int) (sum_dr / samples));

    return (((sum_l / samples) + (sum_r / samples)) / 2);
}

/*!
 * @brief FOLLOW 150 and wait for the user to hold a wheel.
 *
 * @return true if a stall was detected, braked and latched.
 */
static bool
stall_check (void)
{
    subsystem2_status_t status = {0};
    uint32_t            stalls = 0u;
    uint32_t            waited = 0u;
    bool                b_ok   = false;

    subsystem2_status_get(&status);
    stalls = status.stall_count;
    send(S2_CMD_FOLLOW, 0, 150u);
    tm_printf((UB *) "[S2-FLOOR] HOLD the LEFT wheel now (10 s)\n");

    do
    {
        sleep_ms(POLL_MS);
        waited += POLL_MS;
        subsystem2_status_get(&status);
    } while ((status.stall_count == stalls) && (waited < U12_STALL_WAIT_MS));

    b_ok = (status.stall_count != stalls) &&
           (S2_STATE_ESTOP == status.state) && subsystem2_estop_is_set() &&
           (0 == status.duty_l) && (0 == status.duty_r);

    // NOTE: The cast is safe: the format string is ASCII and waited is
    // at most 10000.
    tm_printf((UB *) "[S2-FLOOR] stall %s after %d ms; let go\n",
              b_ok ? "detected" : "NOT detected", (int) waited);

    send(S2_CMD_STOP, 0, 0u);
    sleep_ms(1000u);
    subsystem2_estop_clear();
    send(S2_CMD_CLEAR, 0, 0u);

    return (b_ok);
}

/*!
 * @brief Send one command to motion_task.
 *
 * @param[in] type      Command type.
 * @param[in] value     mm or degrees.
 * @param[in] speed_mms 0 = default.
 */
static void
send (subsystem2_cmd_type_t type, int16_t value, uint16_t speed_mms)
{
    // NOTE: The cast is safe: command types are 0 ... 4.
    subsystem2_cmd_t const cmd =
    {
        .type      = (uint8_t) type,
        .value     = value,
        .speed_mms = speed_mms
    };

    CHECK(subsystem2_motion_send(&cmd), "subsystem2_motion_send");
}

/*!
 * @brief Signed change between two headings, on the circle.
 *
 * @param[in] to_cdeg   Later heading.
 * @param[in] from_cdeg Earlier heading.
 *
 * @return -17999 ... 18000 centidegrees.
 */
static int32_t
cdeg_diff (int32_t to_cdeg, int32_t from_cdeg)
{
    int32_t diff = to_cdeg - from_cdeg;

    if (diff > 18000)
    {
        diff -= 36000;
    }
    else if (diff <= -18000)
    {
        diff += 36000;
    }
    else
    {
        // Already in range.
    }

    return (diff);
}

/*!
 * @brief Absolute value.
 *
 * @param[in] value Any value above INT32_MIN.
 *
 * @return |value|.
 */
static int32_t
magnitude (int32_t value)
{
    return ((value < 0) ? -value : value);
}

/*!
 * @brief Sleep, letting the other tasks run.
 *
 * @param[in] ms Milliseconds.
 */
static void
sleep_ms (uint32_t ms)
{
    // NOTE: The cast is safe: delays here are at most 15 s.
    CHECK(tk_dly_tsk((RELTIM) ms), "tk_dly_tsk");
}
#endif /* 0 != S2_FLOOR_TEST */

#endif /* APP_TEST */

/*** end of file ***/
