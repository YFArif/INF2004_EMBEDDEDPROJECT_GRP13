/** @file s5_math.c
 *
 * @brief Integer trigonometry and square root for subsystem 5.
 *
 * @par
 * The sine table holds round(sin(d) * 32768) for d = 0 to 90 degrees.
 * Every other angle is folded onto that quarter wave by symmetry.
 */

#include <stdint.h>

#include "s5_math.h"

#define DEG_PER_TURN        (360)
#define DEG_HALF_TURN       (180)
#define DEG_QUARTER_TURN    (90)
#define SIN_TABLE_SIZE      (91u)       /* 0 to 90 degrees inclusive.    */
#define Q15_HALF            (16384)     /* 0.5 in Q15, for rounding.     */
#define ISQRT_TOP_BIT       (0x40000000u)

static const uint16_t g_sin_table[SIN_TABLE_SIZE] =
{
        0u,   572u,  1144u,  1715u,  2286u,  2856u,  3425u,  3993u,
     4560u,  5126u,  5690u,  6252u,  6813u,  7371u,  7927u,  8481u,
     9032u,  9580u, 10126u, 10668u, 11207u, 11743u, 12275u, 12803u,
    13328u, 13848u, 14365u, 14876u, 15384u, 15886u, 16384u, 16877u,
    17364u, 17847u, 18324u, 18795u, 19261u, 19720u, 20174u, 20622u,
    21063u, 21498u, 21926u, 22348u, 22763u, 23170u, 23571u, 23965u,
    24351u, 24730u, 25102u, 25466u, 25822u, 26170u, 26510u, 26842u,
    27166u, 27482u, 27789u, 28088u, 28378u, 28660u, 28932u, 29197u,
    29452u, 29698u, 29935u, 30163u, 30382u, 30592u, 30792u, 30983u,
    31164u, 31336u, 31499u, 31651u, 31795u, 31928u, 32052u, 32166u,
    32270u, 32365u, 32449u, 32524u, 32588u, 32643u, 32688u, 32723u,
    32748u, 32763u, 32768u
};

static int32_t sin_quarter(int32_t angle_deg);

/*!
 * @brief Sine of a whole-degree angle.
 *
 * @param[in] angle_deg Any angle in degrees; negative angles are allowed.
 *
 * @return sin(angle) in Q15, from -32768 to 32768.
 */
int32_t
s5_math_sin_q15 (int32_t angle_deg)
{
    // Fold the angle into 0 to 359 degrees.
    int32_t angle = angle_deg % DEG_PER_TURN;

    if (angle < 0)
    {
        angle += DEG_PER_TURN;
    }

    // The second half turn is the first half turn negated.
    int32_t result = 0;

    if (angle < DEG_HALF_TURN)
    {
        result = sin_quarter(angle);
    }
    else
    {
        result = -sin_quarter(angle - DEG_HALF_TURN);
    }

    return (result);
}

/*!
 * @brief Cosine of a whole-degree angle.
 *
 * @param[in] angle_deg Any angle in degrees; negative angles are allowed.
 *
 * @return cos(angle) in Q15, from -32768 to 32768.
 */
int32_t
s5_math_cos_q15 (int32_t angle_deg)
{
    return (s5_math_sin_q15(angle_deg + DEG_QUARTER_TURN));
}

/*!
 * @brief Multiply a value by a Q15 factor, rounding half away from zero.
 *
 * @param[in] value      The value, e.g. a distance in mm.
 * @param[in] factor_q15 The factor in Q15, normally a sine or cosine.
 *
 * @return value * factor, in the units of value.
 *
 * WARNING: |value| must stay below 65536 so that the product fits in
 * int32_t. Ranges in subsystem 5 are below 5000 mm.
 */
int32_t
s5_math_mul_q15 (int32_t value, int32_t factor_q15)
{
    int32_t product = value * factor_q15;
    int32_t result  = 0;

    // Division truncates toward zero, so round each sign separately.
    if (product >= 0)
    {
        result = (product + Q15_HALF) / S5_MATH_Q15_ONE;
    }
    else
    {
        result = (product - Q15_HALF) / S5_MATH_Q15_ONE;
    }

    return (result);
}

/*!
 * @brief Integer square root, rounded down.
 *
 * @param[in] value Any unsigned 32-bit value.
 *
 * @return The largest root such that root * root <= value.
 */
uint32_t
s5_math_isqrt (uint32_t value)
{
    uint32_t remainder = value;
    uint32_t root      = 0u;
    uint32_t bit       = ISQRT_TOP_BIT;

    // Find the highest power of four that is not above the value.
    while (bit > remainder)
    {
        bit >>= 2u;
    }

    // Build the root one bit at a time (digit-by-digit method).
    while (0u != bit)
    {
        if (remainder >= (root + bit))
        {
            remainder -= (root + bit);
            root       = (root >> 1u) + bit;
        }
        else
        {
            root >>= 1u;
        }

        bit >>= 2u;
    }

    return (root);
}

/*!
 * @brief Absolute value of a signed 32-bit value.
 *
 * @param[in] value Any value above INT32_MIN.
 *
 * @return |value|.
 */
int32_t
s5_math_abs (int32_t value)
{
    return ((value < 0) ? -value : value);
}

/*!
 * @brief Sine of an angle in the first half turn.
 *
 * @param[in] angle_deg Angle from 0 to 179 degrees.
 *
 * @return sin(angle) in Q15, from 0 to 32768.
 */
static int32_t
sin_quarter (int32_t angle_deg)
{
    int32_t index = angle_deg;

    // sin(180 - a) equals sin(a), so mirror the second quarter.
    if (index > DEG_QUARTER_TURN)
    {
        index = DEG_HALF_TURN - index;
    }

    // NOTE: index is 0 to 90 here, so it is a valid table index. The
    // cast is safe: every table entry is at most 32768.
    return ((int32_t) g_sin_table[index]);
}

/*** end of file ***/
