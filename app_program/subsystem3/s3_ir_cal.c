/** @file s3_ir_cal.c
 *
 * @brief Subsystem 3 -- calibration of one analogue line sensor. See
 *        s3_ir_cal.h.
 */

#include <stdint.h>

#include "s3_ir_cal.h"

#define WIN_EMPTY_LO    (255u)
#define WIN_EMPTY_HI    (0u)
#define WIN_SHIFT       (4u)        /* Window values are 12-bit >> 4.   */
#define WIN_HALF_STEP   (8u)        /* Back to 12 bits, mid-step.       */
#define RELAX_DIV       (8u)        /* Each end moves 1/8 of the gap.   */
#define PERMILLE        (1000)

static void update_valid(s3_ir_cal_t * p_cal);

/*!
 * @brief Forget everything: uncalibrated until white and black are seen.
 */
void
s3_ir_cal_init (s3_ir_cal_t * p_cal)
{
    p_cal->white   = S3_IR_CAL_ADC_MAX;
    p_cal->black   = 0u;
    p_cal->valid   = 0u;
    p_cal->filt    = 0u;
    p_cal->win_lo  = WIN_EMPTY_LO;
    p_cal->win_hi  = WIN_EMPTY_HI;
    p_cal->started = 0u;
}

/*!
 * @brief 4-sample IIR (40 ms at 10 ms steps): one noisy or glitched
 *        sample moves the value by only a quarter.
 *
 * @return The filtered value.
 */
uint16_t
s3_ir_cal_filter (s3_ir_cal_t * p_cal, uint16_t raw)
{
    raw = (raw > S3_IR_CAL_ADC_MAX) ? S3_IR_CAL_ADC_MAX : raw;
    if (0u == p_cal->started)
    {
        p_cal->filt    = raw;
        p_cal->started = 1u;
    }
    else
    {
        p_cal->filt = ((3u * (uint32_t) p_cal->filt) + (uint32_t) raw + 2u)
                      / 4u;
    }
    return (uint16_t) p_cal->filt;
}

/*!
 * @brief Widen the range with one filtered value (never shrinks it).
 */
void
s3_ir_cal_learn (s3_ir_cal_t * p_cal, uint16_t filt)
{
    uint32_t win;

    filt = (filt > S3_IR_CAL_ADC_MAX) ? S3_IR_CAL_ADC_MAX : filt;
    win  = (uint32_t) filt >> WIN_SHIFT;
    if (filt < p_cal->white)
    {
        p_cal->white = filt;
    }
    if (filt > p_cal->black)
    {
        p_cal->black = filt;
    }
    if (win < p_cal->win_lo)
    {
        p_cal->win_lo = win;
    }
    if (win > p_cal->win_hi)
    {
        p_cal->win_hi = win;
    }
    update_valid(p_cal);
}

/*!
 * @brief Move each end 1/8 towards what was seen since the last call, if
 *        the window really held something on that side (>= MIN_RANGE away
 *        from the other end). Call about once a second.
 */
void
s3_ir_cal_relax (s3_ir_cal_t * p_cal)
{
    uint32_t black     = p_cal->black;
    uint32_t white     = p_cal->white;
    uint32_t hi        = ((uint32_t) p_cal->win_hi << WIN_SHIFT) +
                         WIN_HALF_STEP;
    uint32_t lo        = ((uint32_t) p_cal->win_lo << WIN_SHIFT) +
                         WIN_HALF_STEP;
    uint32_t new_black = black;
    uint32_t new_white = white;

    if ((0u != p_cal->valid) && (p_cal->win_hi >= p_cal->win_lo))
    {
        if ((hi >= (white + S3_IR_CAL_MIN_RANGE)) && (hi < black))
        {
            new_black = black - ((black - hi + (RELAX_DIV - 1u)) / RELAX_DIV);
        }
        if (((lo + S3_IR_CAL_MIN_RANGE) <= black) && (lo > white))
        {
            new_white = white + ((lo - white + (RELAX_DIV - 1u)) / RELAX_DIV);
        }
        if ((new_black > new_white) &&
            ((new_black - new_white) >= S3_IR_CAL_MIN_RANGE))
        {
            p_cal->black = new_black;
            p_cal->white = new_white;
        }
    }
    p_cal->win_lo = WIN_EMPTY_LO;
    p_cal->win_hi = WIN_EMPTY_HI;
}

/*!
 * @brief Normalise a reading.
 *
 * @return 0 (white) ... 1000 (black) permille; 0 if there is no range.
 */
int32_t
s3_ir_cal_norm (s3_ir_cal_t const * p_cal, uint16_t value)
{
    int32_t range = (int32_t) p_cal->black - (int32_t) p_cal->white;
    int32_t norm;

    if (range <= 0)
    {
        return 0;
    }
    norm = (((int32_t) value - (int32_t) p_cal->white) * PERMILLE) / range;
    if (norm < 0)
    {
        norm = 0;
    }
    else if (norm > PERMILLE)
    {
        norm = PERMILLE;
    }
    return norm;
}

static void
update_valid (s3_ir_cal_t * p_cal)
{
    p_cal->valid = ((p_cal->black > p_cal->white) &&
                    ((uint32_t) (p_cal->black - p_cal->white) >=
                     S3_IR_CAL_MIN_RANGE)) ? 1u : 0u;
}

/*** end of file ***/
