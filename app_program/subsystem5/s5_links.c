/** @file s5_links.c
 *
 * @brief Subsystem 5's links to the other subsystems (see s5_links.h).
 *
 * @par When another subsystem is merged
 * - S3: set LINE_IS_REAL to 1 and call S3's functions in
 *   s5_links_line_enable() and s5_links_line_search() (IF-07).
 * - S4: return S4's "calibration done" in s5_links_imu_cal_done().
 * - S1: call subsystem1_ack_send() in s5_links_ack().
 */

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem2/subsystem2.h"
#include "s5_event.h"
#include "s5_links.h"

/* TODO: Set to 1 once S3's line_task is merged (IF-07 LF_ENABLE and
 * LF_SEARCH, EV_LINE_FOUND). While 0 the line is never really looked for.
 */
#define LINE_IS_REAL        (0)

#define MOVE_MAX_MM         (32000)     /* int16_t limit of S2's MOVE.   */
#define TURN_MAX_DEG        (720)       /* S2 clips larger turns.        */

static int16_t clamp_to_int16(int32_t value, int32_t limit);

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
 * @brief Drive along the line at a speed (steered by S3's line_task).
 *
 * @param[in] speed_mms Robot speed in mm/s; 0 = S2's default.
 */
void
s5_links_motion_follow (uint16_t speed_mms)
{
    CHECK(subsystem2_follow(speed_mms), "subsystem2_follow");
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
    CHECK(subsystem2_move(clamp_to_int16(dist_mm, MOVE_MAX_MM), speed_mms),
          "subsystem2_move");
}

/*!
 * @brief Turn on the spot; EV_MOTION_DONE when finished.
 *
 * @param[in] angle_deg Angle; positive = left.
 */
void
s5_links_motion_turn (int32_t angle_deg)
{
    CHECK(subsystem2_turn(clamp_to_int16(angle_deg, TURN_MAX_DEG), 0u),
          "subsystem2_turn");
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
 * @brief Tell whether S3's line sensing is available yet.
 *
 * @return false while the stand-in is in use.
 */
bool
s5_links_line_is_real (void)
{
    return (0 != LINE_IS_REAL);
}

/*!
 * @brief Switch line following on or off (IF-07 LF_ENABLE).
 *
 * @param[in] b_enable true to follow the line.
 */
void
s5_links_line_enable (bool b_enable)
{
    // TODO: S3's function, once merged.
    s5_event_trace("LINK S3 stand-in: line_enable 1/0, real 1/0",
                   b_enable ? 1 : 0, LINE_IS_REAL);
}

/*!
 * @brief Switch line search on or off (IF-07 LF_SEARCH): line_task
 *        raises EV_LINE_FOUND when a sensor sees the line again.
 *
 * @param[in] b_search true to look for the line.
 */
void
s5_links_line_search (bool b_search)
{
    // TODO: S3's function, once merged.
    s5_event_trace("LINK S3 stand-in: line_search 1/0, real 1/0",
                   b_search ? 1 : 0, LINE_IS_REAL);
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

/*** end of file ***/
