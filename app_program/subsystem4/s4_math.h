/** @file s4_math.h
 *
 * @brief Small maths helpers for imu_task. The build links no maths
 *        library, so atan2 and sine are polynomial approximations
 *        (error below 0.01 degree and 0.0002, well inside the 1 degree
 *        pitch budget of S4-U01).
 */

#ifndef S4_MATH_H
#define S4_MATH_H

#include <stdint.h>

/* BARR-C 5.4b(i): C99 has no float32_t, so it is defined here. float is the
 * IEEE 754 single-precision (32-bit) type on the RP2040's compiler.
 */
typedef float float32_t;

/* Returns -18000 ... 18000. */
int32_t   s4_math_atan2_cdeg(float32_t y_val, float32_t x_val);

/* Clamped to +/-90 deg. */
float32_t s4_math_sin_cdeg(int32_t angle_cdeg);

float32_t s4_math_sqrt(float32_t value);
int32_t   s4_math_clamp(int32_t value, int32_t low, int32_t high);
int32_t   s4_math_abs(int32_t value);

#endif /* S4_MATH_H */

/*** end of file ***/

