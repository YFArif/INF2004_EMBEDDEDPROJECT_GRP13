/** @file s5_math.h
 *
 * @brief Integer trigonometry and square root for subsystem 5.
 *
 * @par
 * Subsystem 5 avoids floating point (BARR-C 5.4a). Angles are whole
 * degrees and trigonometric results are Q15 fixed point, in which
 * S5_MATH_Q15_ONE (32768) stands for 1.0. The module uses no kernel or
 * hardware calls, so it can be tested on a PC as well as on the robot.
 */

#ifndef S5_MATH_H
#define S5_MATH_H

#include <stdint.h>

#define S5_MATH_Q15_ONE     (32768)     /* 1.0 in Q15 fixed point.       */

int32_t  s5_math_sin_q15(int32_t angle_deg);
int32_t  s5_math_cos_q15(int32_t angle_deg);
int32_t  s5_math_mul_q15(int32_t value, int32_t factor_q15);
uint32_t s5_math_isqrt(uint32_t value);
int32_t  s5_math_abs(int32_t value);

#endif /* S5_MATH_H */

/*** end of file ***/
