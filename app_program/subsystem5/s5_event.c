/** @file s5_event.c
 *
 * @brief Subsystem 5 event reporting (FR16) and console trace.
 */

#include <stdint.h>

#include <tm/tmonitor.h>    /* tm_printf() console output.              */

#include "s5_event.h"

#define MM_PER_CM           (10u)
#define ARG_MAX             (255u)      /* One argument byte.            */
#define ANGLE_MAX_DEG       (180)

static uint8_t to_arg_cm(uint32_t dist_mm);
static uint8_t to_arg_deg(int32_t angle_deg);

/*!
 * @brief Report one subsystem 5 event.
 *
 * @param[in] code      The event.
 * @param[in] dist_mm   Range, width or depth in mm (see s5_event.h).
 * @param[in] angle_deg Servo angle in degrees.
 *
 * NOTE: The record is built exactly as it will go to S1's mbf_event, so
 * the console line shows the bytes the dashboard will receive.
 */
void
s5_event_post (s5_event_code_t code, uint32_t dist_mm, int32_t angle_deg)
{
    uint8_t arg_cm  = to_arg_cm(dist_mm);
    uint8_t arg_deg = to_arg_deg(angle_deg);

    // TODO: Send {code, arg_cm, arg_deg} to S1's mbf_event with TMO_POL
    // once it exists.
    // NOTE: The casts are safe: format strings are ASCII, and every value
    // is 0 to 255, which an int (%d, %x) always holds.
    tm_printf((UB *) "[S5] EVT 0x%02x d=%dcm a=%ddeg\n",
              (int) code, (int) arg_cm, (int) arg_deg);
}

/*!
 * @brief Print one labelled trace line with two numbers.
 *
 * @param[in] p_label Short label, e.g. "CAL".
 * @param[in] value_a First number.
 * @param[in] value_b Second number.
 *
 * NOTE: The casts are safe: format strings are ASCII, and tm_printf's
 * %d reads an int, which is 32 bits on the RP2040.
 */
void
s5_event_trace (char const * p_label, int32_t value_a, int32_t value_b)
{
    tm_printf((UB *) "[S5] %s %d %d\n", p_label, (int) value_a,
              (int) value_b);
}

/*!
 * @brief Print one ping: where the servo pointed, what it measured, the
 *        raw echo time, and why there was no range if there was none.
 *
 * @param[in] servo_deg Servo angle, 0 (right) to 180 (left).
 * @param[in] range_mm  Range in mm, or a negative value for no range.
 * @param[in] echo_us   Raw width of the ECHO pulse in microseconds.
 * @param[in] p_why     Reason for no range; ignored when there is one.
 *
 * NOTE: The casts are safe: format strings are ASCII, and the angle,
 * range and echo time (at most 65535 us) are far inside an int.
 */
void
s5_event_log_ping (int32_t servo_deg, int32_t range_mm, uint32_t echo_us,
                   char const * p_why)
{
    if (range_mm < 0)
    {
        tm_printf((UB *) "[S5] PING servo=%3d deg  range=  -- mm  "
                  "echo=%5d us  %s\n",
                  (int) servo_deg, (int) echo_us, p_why);
    }
    else
    {
        tm_printf((UB *) "[S5] PING servo=%3d deg  range=%4d mm  "
                  "echo=%5d us\n",
                  (int) servo_deg, (int) range_mm, (int) echo_us);
    }
}

/*!
 * @brief Convert a distance to the one-byte cm argument.
 *
 * @param[in] dist_mm Distance in mm.
 *
 * @return Distance in cm, 0 to 255.
 */
static uint8_t
to_arg_cm (uint32_t dist_mm)
{
    uint32_t dist_cm = dist_mm / MM_PER_CM;

    if (dist_cm > ARG_MAX)
    {
        dist_cm = ARG_MAX;
    }

    // NOTE: The cast is safe: dist_cm was limited to 255.
    return ((uint8_t) dist_cm);
}

/*!
 * @brief Convert an angle to the one-byte degree argument.
 *
 * @param[in] angle_deg Angle in degrees.
 *
 * @return Angle limited to 0 to 180 degrees.
 */
static uint8_t
to_arg_deg (int32_t angle_deg)
{
    int32_t angle = angle_deg;

    if (angle < 0)
    {
        angle = 0;
    }
    else if (angle > ANGLE_MAX_DEG)
    {
        angle = ANGLE_MAX_DEG;
    }
    else
    {
        // Already inside the range.
    }

    // NOTE: The cast is safe: angle was limited to 0 ... 180.
    return ((uint8_t) angle);
}

/*** end of file ***/
