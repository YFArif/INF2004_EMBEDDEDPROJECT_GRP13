/** @file s5_links.c
 *
 * @brief Subsystem 5's links to the other subsystems (see s5_links.h).
 *
 * @par S3 rules followed here (s3_api.h, README_S3.md)
 * - flg_line: LF_SEARCH_x wins over LF_ENABLE. To end a search, keep
 *   LF_ENABLE set and then clear LF_SEARCH_x; clearing both at once
 *   leaves S3 DISABLED for a step.
 * - flg_s3 has one consumer, vehicle_ctrl_task, through
 *   s5_links_line_events(). FLG_S3_LINE_FOUND is only raised by a search.
 * - mbf_nav (owned by S3) is read only by vehicle_ctrl_task, through
 *   s5_links_nav_take().
 *
 * @par When another subsystem is merged
 * - S4: return S4's "calibration done" in s5_links_imu_cal_done().
 * - S1: call subsystem1_ack_send() in s5_links_ack().
 */

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem2/subsystem2.h"
#include "../subsystem3/s3_config.h"    /* S3_EXPERIMENT_MODE, DRIVE_TEST. */
#include "../subsystem3/subsystem3.h"   /* s3_api.h.                       */
#include "s5_event.h"
#include "s5_links.h"
#include "subsystem5.h"

#define MOVE_MAX_MM         (32000)     /* int16_t limit of S2's MOVE.   */
#define TURN_MAX_DEG        (720)       /* S2 clips larger turns.        */
#define STILL_POLL_MS       (20u)       /* motion_task's period.         */

/* Line following tuning: S3's drive-test default preset, the settings S3
 * proved on the car (task_drive_test.c g_presets[0], 30 Oct). With S3's
 * built-in defaults at 300 mm/s the robot ran off the curves; at S3's
 * 40 mm/s it crawled. LINE_SPEED_MMS is between: S3's drive test used
 * 100 mm/s as its default until 29 Oct. Tune it here (60 ... 120).
 */
#define LINE_SPEED_MMS      (60u)
#define LINE_KP_X100        (25)
#define LINE_KD_X100        (25)
#define LINE_BOOST_X100     (250)       /* Curve authority.              */
#define LINE_STEER_PER_MMS  (2u)        /* Full lock = 2 x the speed.    */
#define FLOOR_TEST_MMS      (150u)      /* No line to follow: faster.    */
#define LINE_BLACK_U8       (128u)      /* Half way, white 0 to black    */
                                        /* 255: S3_LN_FOUND (500/1000).  */
#define STILL_MAX_MS        (500u)      /* Longest wait for still wheels.*/
#define LF_SEARCH_BOTH      (LF_SEARCH_L | LF_SEARCH_R)
#define FLG_S3_ALL          (FLG_S3_JUNCTION | FLG_S3_LOST \
                             | FLG_S3_BARCODE | FLG_S3_LINE_FOUND)

/* false: the S5 floor tests drive without S3 (s5_links_line_use()). */
static volatile bool g_b_line_used = true;

static int16_t clamp_to_int16(int32_t value, int32_t limit);
static void    wait_wheels_still(void);
static bool    nav_from_s3(s3_nav_msg_t const * p_s3_msg,
                           subsystem5_nav_msg_t * p_msg);
static void    nav_drain(void);

/*!
 * @brief Tell whether S5's tasks may run. An isolated S3 test
 *        (S3_EXPERIMENT_MODE or S3_DRIVE_TEST in Sean's s3_config.h)
 *        drives the motors and reads the buttons itself.
 *
 * @return false while such a test is built in.
 */
bool
s5_links_tasks_allowed (void)
{
    return ((0 == S3_EXPERIMENT_MODE) && (0 == S3_DRIVE_TEST));
}

/*!
 * @brief Leave the emergency stop: clear MF_ESTOP, then send CLEAR, so
 *        motion_task accepts the commands that follow (IF-05).
 */
void
s5_links_motion_release (void)
{
    subsystem2_estop_clear();
    CHECK(subsystem2_clear(), "subsystem2_clear");
}

/*!
 * @brief Brake and cancel any MOVE, TURN or FOLLOW.
 */
void
s5_links_motion_stop (void)
{
    CHECK(subsystem2_stop(), "subsystem2_stop");
}

/*!
 * @brief Brake, then wait until both wheels stand still (at most
 *        STILL_MAX_MS). Call before FOLLOW after a TURN: a wheel still
 *        rolling backwards is otherwise counted the wrong way and driven
 *        flat out, and the robot spins (S3 car test 29 Oct; the same
 *        spin after S5's line recovery, 2026-11-04).
 */
void
s5_links_motion_settle (void)
{
    s5_links_motion_stop();
    wait_wheels_still();
}

/*!
 * @brief Drive along the line at a speed (steered by S3's line_task),
 *        with S3's proven tuning for that speed.
 *
 * @param[in] speed_mms Robot speed in mm/s; 0 = S2's default.
 */
void
s5_links_motion_follow (uint16_t speed_mms)
{
    if (g_b_line_used)
    {
        // NOTE: The cast is safe: speeds are at most a few hundred mm/s.
        s3_set_line_gains(LINE_KP_X100, LINE_KD_X100);
        s3_set_line_shape(LINE_BOOST_X100);
        s3_set_steer_max((INT) (LINE_STEER_PER_MMS * speed_mms));
    }

    CHECK(subsystem2_follow(speed_mms), "subsystem2_follow");
}

/*!
 * @brief The speed to follow the line at, or faster in the floor tests,
 *        where there is no line.
 *
 * @return Robot speed in mm/s.
 */
uint16_t
s5_links_follow_speed_mms (void)
{
    // NOTE: The casts are safe: both speeds are a few hundred mm/s.
    return (g_b_line_used ? (uint16_t) LINE_SPEED_MMS
                          : (uint16_t) FLOOR_TEST_MMS);
}

/*!
 * @brief Drive straight by a distance; EV_MOTION_DONE when finished.
 *
 * @param[in] dist_mm   Distance; negative = reverse. Clamped to S2's
 *                      +/-32000 mm.
 * @param[in] speed_mms Speed in mm/s; 0 = S2's default.
 */
void
s5_links_motion_move (int32_t dist_mm, uint16_t speed_mms)
{
    wait_wheels_still();
    CHECK(subsystem2_move(clamp_to_int16(dist_mm, MOVE_MAX_MM), speed_mms),
          "subsystem2_move");
}

/*!
 * @brief Turn on the spot; EV_MOTION_DONE when finished.
 *
 * @param[in] angle_deg Angle; positive = left.
 * @param[in] speed_mms Wheel speed in mm/s; 0 = S2's default.
 */
void
s5_links_motion_turn (int32_t angle_deg, uint16_t speed_mms)
{
    wait_wheels_still();
    CHECK(subsystem2_turn(clamp_to_int16(angle_deg, TURN_MAX_DEG),
                          speed_mms), "subsystem2_turn");
}

/*!
 * @brief Set the emergency-stop latch (IF-05); motion_task brakes at once.
 */
void
s5_links_motion_estop (void)
{
    subsystem2_estop_set();
}

/*!
 * @brief Distance driven since power-up (IF-06).
 *
 * @return Signed distance of the robot centre in mm.
 */
int32_t
s5_links_odom_dist_mm (void)
{
    subsystem2_odom_t odom = {0};

    subsystem2_odom_get(&odom);

    return (odom.dist_mm);
}

/*!
 * @brief Use S3's line following, or not (the S5 floor tests switch it
 *        off, so a patterned floor cannot steer the robot).
 *
 * @param[in] b_used false: S3 is kept DISABLED and its events ignored.
 */
void
s5_links_line_use (bool b_used)
{
    g_b_line_used = b_used;

    if (!b_used)
    {
        CHECK(tk_clr_flg(flg_line, ~(LF_ENABLE | LF_SEARCH_BOTH)),
              "flg_line");
    }
}

/*!
 * @brief Tell whether S3's line following is in use (the S5 floor tests
 *        switch it off).
 *
 * @return true unless s5_links_line_use(false) was called.
 */
bool
s5_links_line_is_used (void)
{
    return (g_b_line_used);
}

/*!
 * @brief Tell whether S3 can follow and find the line: in use, and its
 *        sensors calibrated on white and black (IF-S3b).
 *
 * @return false otherwise; the controller then works without the line.
 */
bool
s5_links_line_is_ready (void)
{
    s3_line_status_t status  = {0};
    bool             b_ready = false;

    if (g_b_line_used && (E_OK == s3_get_line_status(&status)))
    {
        b_ready = (S3_LINE_UNCAL != status.state);
    }

    return (b_ready);
}

/*!
 * @brief Make S3 forget its white and black levels, so the calibration
 *        sweep that follows learns them afresh (IF-S3e).
 */
void
s5_links_line_recalibrate (void)
{
    if (g_b_line_used)
    {
        s3_request_recalibrate();
    }
}

/*!
 * @brief Switch line following on or off (IF-07 LF_ENABLE).
 *
 * @param[in] b_enable true to follow the line.
 */
void
s5_links_line_enable (bool b_enable)
{
    if (b_enable && g_b_line_used)
    {
        CHECK(tk_set_flg(flg_line, LF_ENABLE), "flg_line");
    }
    else
    {
        CHECK(tk_clr_flg(flg_line, ~LF_ENABLE), "flg_line");
    }
}

/*!
 * @brief Start or end a line search (IF-07 LF_SEARCH_x). S3 drives
 *        straight until a sensor sees the line, then raises
 *        FLG_S3_LINE_FOUND (EV_LINE_FOUND) and follows it.
 *
 * @param[in] side The robot's side of the line, or S5_LINKS_SIDE_NONE to
 *                 end the search. Call s5_links_line_enable(true) first
 *                 when ending a search, so S3 is never DISABLED.
 */
void
s5_links_line_search (s5_links_side_t side)
{
    CHECK(tk_clr_flg(flg_line, ~LF_SEARCH_BOTH), "flg_line");

    if (!g_b_line_used)
    {
        // S3 is not in use: there is nothing to search with.
    }
    else if (S5_LINKS_SIDE_LEFT == side)
    {
        CHECK(tk_set_flg(flg_line, LF_SEARCH_L), "flg_line");
    }
    else if (S5_LINKS_SIDE_RIGHT == side)
    {
        CHECK(tk_set_flg(flg_line, LF_SEARCH_R), "flg_line");
    }
    else
    {
        // S5_LINKS_SIDE_NONE: the search has ended.
    }
}

/*!
 * @brief Which side of the line the robot was on last, from S3's
 *        steering (+ = turn right, so the line is to the right). S3
 *        keeps its last steering while the line is LOST.
 *
 * @return LEFT or RIGHT; LEFT if S3 gives no hint.
 */
s5_links_side_t
s5_links_line_side (void)
{
    s3_line_status_t status = {0};
    s5_links_side_t  side   = S5_LINKS_SIDE_LEFT;

    if (E_OK == s3_get_line_status(&status))
    {
        int32_t toward = (0 != status.steer_dv) ? status.steer_dv
                                                : status.pos;

        side = (toward < 0) ? S5_LINKS_SIDE_RIGHT : S5_LINKS_SIDE_LEFT;
    }

    return (side);
}

/*!
 * @brief Tell whether either of S3's line sensors sees black now
 *        (IF-S3b; it only reads S3's status).
 *
 * @return true if one reads at least half way from white to black.
 */
bool
s5_links_line_sees_black (void)
{
    s3_line_status_t status  = {0};
    bool             b_black = false;

    if (g_b_line_used && (E_OK == s3_get_line_status(&status)))
    {
        b_black = (status.n_outer >= LINE_BLACK_U8)
                  || (status.n_centre >= LINE_BLACK_U8);
    }

    return (b_black);
}

/*!
 * @brief Take S3's events (IF-S3c flg_s3) as mission events. Call it on
 *        every vehicle_ctrl_task wake-up; it never blocks.
 *
 * @return EV_JUNCTION, EV_LINE_LOST, EV_LINE_FOUND and EV_BARCODE bits;
 *         0 while S3 is not in use.
 */
uint32_t
s5_links_line_events (void)
{
    UINT     pattern = 0u;
    uint32_t events  = 0u;
    ER       result  = tk_wai_flg(flg_s3, FLG_S3_ALL, TWF_ORW | TWF_BITCLR,
                                  &pattern, TMO_POL);

    if (E_OK != result)
    {
        // No event since the last call.
    }
    else if (!g_b_line_used)
    {
        // Ignored, but S3's barcode queue is emptied so it never fills.
        nav_drain();
    }
    else
    {
        events |= (0u != (pattern & FLG_S3_JUNCTION)) ? EV_JUNCTION : 0u;
        events |= (0u != (pattern & FLG_S3_LOST)) ? EV_LINE_LOST : 0u;
        events |= (0u != (pattern & FLG_S3_LINE_FOUND)) ? EV_LINE_FOUND
                                                        : 0u;
        events |= (0u != (pattern & FLG_S3_BARCODE)) ? EV_BARCODE : 0u;
    }

    return (events);
}

/*!
 * @brief Take the oldest barcode command from S3's mbf_nav (IF-01).
 *        Letters that are not in today's map are skipped.
 *
 * @param[out] p_msg The command.
 *
 * @return true if one was waiting.
 */
bool
s5_links_nav_take (subsystem5_nav_msg_t * p_msg)
{
    s3_nav_msg_t s3_msg = {0};
    bool         b_got  = false;

    // NOTE: The cast is safe: the message is 4 bytes.
    while ((!b_got)
           && ((INT) sizeof(s3_msg) == tk_rcv_mbf(mbf_nav, &s3_msg,
                                                   TMO_POL)))
    {
        b_got = nav_from_s3(&s3_msg, p_msg);
    }

    return (b_got);
}

/*!
 * @brief Tell whether S4 has finished its IMU calibration.
 *
 * @return true; the stand-in never waits.
 */
bool
s5_links_imu_cal_done (void)
{
    // TODO: S4's "cal_done" (IF-08 terrain.cal_done), once merged.
    return (true);
}

/*!
 * @brief Acknowledge a command from the dashboard (FR13, NFR15).
 *
 * @param[in] opcode     The command (S5_CMD_xxx).
 * @param[in] cmd_id     The dashboard's command number, echoed back.
 * @param[in] b_accepted true if the controller acted on it.
 */
void
s5_links_ack (uint8_t opcode, uint8_t cmd_id, bool b_accepted)
{
    // TODO: subsystem1_ack_send(opcode, cmd_id, result), once merged.
    s5_event_trace("LINK S1 stand-in: ack opcode accepted", opcode,
                   b_accepted ? 1 : 0);
    s5_event_post_args(S5_EVENT_COMMAND, opcode, b_accepted ? 1u : 0u);

    // The id is only needed by the real link.
    (void) cmd_id;
}

/*!
 * @brief Limit a value to +/-limit and convert it to int16_t.
 *
 * @param[in] value The value.
 * @param[in] limit Positive limit, at most 32767.
 *
 * @return The limited value.
 */
static int16_t
clamp_to_int16 (int32_t value, int32_t limit)
{
    int32_t limited = value;

    if (limited > limit)
    {
        limited = limit;
    }
    else if (limited < -limit)
    {
        limited = -limit;
    }
    else
    {
        // Already inside the range.
    }

    // NOTE: The cast is safe: limited is within +/-32767 here.
    return ((int16_t) limited);
}

/*!
 * @brief Wait until both wheels stand still, at most STILL_MAX_MS, before
 *        a MOVE or TURN.
 *
 * S2's encoders cannot see direction: motion_task counts each tick in
 * the direction the wheel last moved, and switches only after 100 ms
 * without a tick. A TURN started while a wheel still rolls the other
 * way is therefore miscounted and never finishes (log
 * 2026-11-04_s5_avoid_2: TURN straight after the reverse, robot kept
 * turning left until the time limit).
 *
 * NOTE: Blocks vehicle_ctrl_task for at most 0.5 s while the robot is
 * braked; mission events wait in flg_mission meanwhile, and obstacle
 * stops do not depend on this task.
 */
static void
wait_wheels_still (void)
{
    subsystem2_odom_t odom   = {0};
    uint32_t          waited = 0u;

    subsystem2_odom_get(&odom);

    while (((0 != odom.speed_l_mms) || (0 != odom.speed_r_mms))
           && (waited < STILL_MAX_MS))
    {
        subsystem5_delay_ms(STILL_POLL_MS);
        waited += STILL_POLL_MS;
        subsystem2_odom_get(&odom);
    }
}

/*!
 * @brief Convert one S3 barcode message to S5's form.
 *
 * @param[in]  p_s3_msg The message from mbf_nav.
 * @param[out] p_msg    The command.
 *
 * @return false for a letter that is not a command today (S3_CMD_NONE).
 */
static bool
nav_from_s3 (s3_nav_msg_t const * p_s3_msg, subsystem5_nav_msg_t * p_msg)
{
    bool b_ok = true;

    // NOTE: The casts below are safe: the commands are 0 ... 3.
    switch (p_s3_msg->cmd)
    {
        case S3_CMD_LEFT:
            p_msg->cmd = (uint8_t) S5_NAV_LEFT;
        break;

        case S3_CMD_RIGHT:
            p_msg->cmd = (uint8_t) S5_NAV_RIGHT;
        break;

        case S3_CMD_STRAIGHT:
            p_msg->cmd = (uint8_t) S5_NAV_STRAIGHT;
        break;

        case S3_CMD_UTURN:
            p_msg->cmd = (uint8_t) S5_NAV_UTURN;
        break;

        default:
            // S3_CMD_NONE: decoded, but not in today's map.
            b_ok = false;
        break;
    }

    p_msg->letter     = s3_nav_msg_char(p_s3_msg);
    p_msg->b_reversed = (0u != p_s3_msg->reversed);
    p_msg->t_ms       = subsystem5_now_ms();

    return (b_ok);
}

/*!
 * @brief Throw away every waiting barcode message (S3 not in use).
 */
static void
nav_drain (void)
{
    s3_nav_msg_t s3_msg = {0};

    // NOTE: The cast is safe: the message is 4 bytes.
    while ((INT) sizeof(s3_msg) == tk_rcv_mbf(mbf_nav, &s3_msg, TMO_POL))
    {
        // Dropped.
    }
}

/*** end of file ***/
