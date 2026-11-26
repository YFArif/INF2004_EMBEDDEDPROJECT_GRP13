/** @file s4_math.c
 *
 * @brief Maths helpers for imu_task (see s4_math.h).
 */

#include <stdint.h>

#include "s4_math.h"

#define PI_F                (3.14159265f)
#define HALF_PI_F           (1.57079633f)
#define CDEG_PER_RAD        (5729.57795f)
#define RAD_PER_CDEG        (0.000174532925f)
#define CDEG_90             (9000)
#define SQRT_ITERATIONS     (12u)

/* atan(z) for 0 <= z <= 1, error about 1e-5 rad (Abramowitz and Stegun
 * 4.4.49 form). */
#define ATAN_C1             (0.9998660f)
#define ATAN_C3             (-0.3302995f)
#define ATAN_C5             (0.1801410f)
#define ATAN_C7             (-0.0851330f)
#define ATAN_C9             (0.0208351f)

static float32_t atan_unit(float32_t ratio);

/*!
 * @brief Angle of the vector (x_val, y_val) in 1/100 degree.
 *
 * @param[in] y_val Component along the "positive angle" axis.
 * @param[in] x_val Component along the "zero angle" axis.
 *
 * @return -18000 ... 18000; 0 if both are 0.
 */
int32_t
s4_math_atan2_cdeg (float32_t y_val, float32_t x_val)
{
    float32_t abs_x = (x_val < 0.0f) ? -x_val : x_val;
    float32_t abs_y = (y_val < 0.0f) ? -y_val : y_val;
    float32_t angle = 0.0f;

    if ((abs_x > 0.0f) || (abs_y > 0.0f))
    {
        if (abs_y <= abs_x)
        {
            angle = atan_unit(abs_y / abs_x);
        }
        else
        {
            angle = HALF_PI_F - atan_unit(abs_x / abs_y);
        }

        if (x_val < 0.0f)
        {
            angle = PI_F - angle;
        }

        if (y_val < 0.0f)
        {
            angle = -angle;
        }
    }

    // NOTE: The cast is safe: |angle| <= pi, so the result is <= 18000.
    return ((int32_t) (angle * CDEG_PER_RAD));
}

/*!
 * @brief Sine of an angle in 1/100 degree, clamped to +/-90 degrees.
 *
 * @param[in] angle_cdeg The angle.
 *
 * @return sin(angle), -1 ... 1.
 */
float32_t
s4_math_sin_cdeg (int32_t angle_cdeg)
{
    // NOTE: The cast is safe: the angle is clamped to +/-9000 cdeg,
    // which float32_t holds exactly.
    float32_t rad    = (float32_t) s4_math_clamp(angle_cdeg, -CDEG_90,
                                                 CDEG_90)
                       * RAD_PER_CDEG;
    float32_t rad_sq = rad * rad;

    // Taylor series to rad^9: error below 4e-6 for |rad| <= pi / 2.
    return (rad * (1.0f - ((rad_sq / 6.0f) * (1.0f - ((rad_sq / 20.0f)
                   * (1.0f - ((rad_sq / 42.0f)
                   * (1.0f - (rad_sq / 72.0f)))))))));
}

/*!
 * @brief Square root by Newton's method.
 *
 * @param[in] value Must be >= 0; a negative value returns 0.
 *
 * @return sqrt(value).
 */
float32_t
s4_math_sqrt (float32_t value)
{
    float32_t root = value;
    uint32_t  idx  = 0u;

    if (value <= 0.0f)
    {
        root = 0.0f;
    }
    else
    {
        if (root < 1.0f)
        {
            root = 1.0f;
        }

        for (idx = 0u; idx < SQRT_ITERATIONS; idx++)
        {
            root = 0.5f * (root + (value / root));
        }
    }

    return (root);
}

/*!
 * @brief Limit a value to [low, high].
 */
int32_t
s4_math_clamp (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (result < low)
    {
        result = low;
    }
    else if (result > high)
    {
        result = high;
    }
    else
    {
        // Already inside the range.
    }

    return (result);
}

/*!
 * @brief Absolute value (INT32_MIN is returned as INT32_MAX).
 */
int32_t
s4_math_abs (int32_t value)
{
    int32_t result = value;

    if (value < 0)
    {
        result = (INT32_MIN == value) ? INT32_MAX : -value;
    }

    return (result);
}

/*!
 * @brief atan(ratio) for 0 <= ratio <= 1.
 */
static float32_t
atan_unit (float32_t ratio)
{
    float32_t ratio_sq = ratio * ratio;

    return (ratio * (ATAN_C1 + (ratio_sq * (ATAN_C3 + (ratio_sq * (ATAN_C5
                     + (ratio_sq * (ATAN_C7 + (ratio_sq * ATAN_C9)))))))));
}

/*** end of file ***/

