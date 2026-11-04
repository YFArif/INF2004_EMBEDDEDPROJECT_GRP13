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

static float atan_unit(float z);

/*!
 * @brief Angle of the vector (x, y) in 1/100 degree.
 *
 * @param[in] y Component along the "positive angle" axis.
 * @param[in] x Component along the "zero angle" axis.
 *
 * @return -18000 ... 18000; 0 if both are 0.
 */
int32_t
s4_math_atan2_cdeg (float y, float x)
{
    float ax    = (x < 0.0f) ? -x : x;
    float ay    = (y < 0.0f) ? -y : y;
    float angle = 0.0f;

    if ((ax > 0.0f) || (ay > 0.0f))
    {
        if (ay <= ax)
        {
            angle = atan_unit(ay / ax);
        }
        else
        {
            angle = HALF_PI_F - atan_unit(ax / ay);
        }

        if (x < 0.0f)
        {
            angle = PI_F - angle;
        }

        if (y < 0.0f)
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
float
s4_math_sin_cdeg (int32_t angle_cdeg)
{
    float x  = (float) s4_math_clamp(angle_cdeg, -CDEG_90, CDEG_90)
               * RAD_PER_CDEG;
    float x2 = x * x;

    // Taylor series to x^9: error below 4e-6 for |x| <= pi / 2.
    return (x * (1.0f - (x2 / 6.0f) * (1.0f - (x2 / 20.0f)
                 * (1.0f - (x2 / 42.0f) * (1.0f - (x2 / 72.0f))))));
}

/*!
 * @brief Square root by Newton's method.
 *
 * @param[in] value Must be >= 0; a negative value returns 0.
 *
 * @return sqrt(value).
 */
float
s4_math_sqrt (float value)
{
    float    root = value;
    uint32_t idx  = 0u;

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
 * @brief Limit a value to [lo, hi].
 */
int32_t
s4_math_clamp (int32_t value, int32_t lo, int32_t hi)
{
    int32_t result = value;

    if (result < lo)
    {
        result = lo;
    }
    else if (result > hi)
    {
        result = hi;
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
 * @brief atan(z) for 0 <= z <= 1.
 */
static float
atan_unit (float z)
{
    float z2 = z * z;

    return (z * (ATAN_C1 + z2 * (ATAN_C3 + z2 * (ATAN_C5 + z2 * (ATAN_C7
                 + z2 * ATAN_C9)))));
}

/*** end of file ***/
