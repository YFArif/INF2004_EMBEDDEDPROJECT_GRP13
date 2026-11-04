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

int32_t s4_math_atan2_cdeg(float y, float x);  /* -18000 ... 18000.      */
float   s4_math_sin_cdeg(int32_t angle_cdeg);  /* Clamped to +/-90 deg.  */
float   s4_math_sqrt(float value);
int32_t s4_math_clamp(int32_t value, int32_t lo, int32_t hi);
int32_t s4_math_abs(int32_t value);

#endif /* S4_MATH_H */

/*** end of file ***/
