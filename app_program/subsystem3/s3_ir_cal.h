/** @file s3_ir_cal.h
 *
 * @brief Subsystem 3 -- calibration of ONE analogue line sensor (pure C,
 *        no kernel calls, unit-tested on a laptop:
 *        testing/subsystem3/host/test_line.c).
 *
 * @par
 * The Left IR and the Centre sensor each have one s3_ir_cal_t (inside
 * s3_line_t). A sensor starts UNcalibrated (white = 4095, black = 0) and
 * must see real white AND real black once (the start-up sweep) before its
 * reading is trusted.
 *
 * @par Learning and relaxing
 * The calibration learns the lowest (white) and highest (black) FILTERED
 * value (4-sample IIR, so one glitch cannot set an extreme). About once a
 * second s3_ir_cal_relax() moves each end 1/8 of the way towards what the
 * sensor really saw in that second, but only if it saw something on that
 * side: a one-off extreme fades, an unused end does not decay, and the
 * range never shrinks below S3_IR_CAL_MIN_RANGE.
 *
 * @par Units
 * Inputs are 12-bit ADC counts, higher = darker. s3_ir_cal_norm() returns
 * permille: 0 = white, 1000 = black.
 */

#ifndef S3_IR_CAL_H
#define S3_IR_CAL_H

#include <stdint.h>

#define S3_IR_CAL_MIN_RANGE (600u)  /* white..black range needed to trust. */
#define S3_IR_CAL_ADC_MAX   (4095u)

/** One sensor's calibration: 54 bits in 8 bytes (was 14 bytes). */
typedef struct
{
    unsigned int white   : 12;  /* Lowest filtered value.                 */
    unsigned int black   : 12;  /* Highest filtered value.                */
    unsigned int valid   : 1;   /* 1 once black - white >= MIN_RANGE.     */
    unsigned int started : 1;   /* The filter has its first sample.       */
    unsigned int filt    : 12;  /* 4-sample IIR of the raw value.         */
    unsigned int win_lo  : 8;   /* Lowest filtered value since the last   */
                                /* relax, >> 4 (16-count steps are enough */
                                /* for a 1/8 relax step).                 */
    unsigned int win_hi  : 8;   /* Highest, >> 4.                         */
} s3_ir_cal_t;

void     s3_ir_cal_init(s3_ir_cal_t * p_cal);
uint16_t s3_ir_cal_filter(s3_ir_cal_t * p_cal, uint16_t raw);
void     s3_ir_cal_learn(s3_ir_cal_t * p_cal, uint16_t filt);
void     s3_ir_cal_relax(s3_ir_cal_t * p_cal);
int32_t  s3_ir_cal_norm(s3_ir_cal_t const * p_cal, uint16_t value);

#endif /* S3_IR_CAL_H */

/*** end of file ***/
