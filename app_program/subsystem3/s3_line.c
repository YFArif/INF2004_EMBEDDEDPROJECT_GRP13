/** @file s3_line.c
 *
 * @brief Subsystem 3 -- line following with one sensor on each edge
 *        (edge pair, v29) and per-sensor calibration. See s3_line.h.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s3_line.h"

#define UM_PER_MM       (1000u)
#define MM_TO_UM(mm)    ((uint32_t) (mm) * UM_PER_MM)
#define SAT16           (0xFFFFu)
#define X100            (100)
#define PERMILLE        (1000)

/* Calibration window and relax step (s3_ir_cal_relax()). */
#define WIN_EMPTY_LO    (255u)
#define WIN_EMPTY_HI    (0u)
#define WIN_SHIFT       (4u)        /* Window values are 12-bit >> 4.   */
#define WIN_HALF_STEP   (8u)        /* Back to 12 bits, mid-step.       */
#define RELAX_DIV       (8u)        /* Each end moves 1/8 of the gap.   */

/* Distances are saturating 16-bit micrometres (s3_line.h): the lost
 * threshold must stay below 65.5 mm or it could never be reached. */
typedef char s3_line_chk_mm_t[((S3_LN_LOST_MM * UM_PER_MM) < SAT16) ?
                              1 : -1];

static int32_t  clamp_i32(int32_t value, int32_t lo, int32_t hi);
static uint16_t add_sat16(uint16_t acc, uint32_t add);
static void     reset_tracking(s3_line_t * p_ln);
static int32_t  off_line_steer(uint16_t white_um);
static int      search_step(s3_line_t * p_ln, int32_t n_max,
                            s3_line_out_t * p_out);
static int32_t  pd_steer(s3_line_t * p_ln, int32_t err, uint32_t dt_us,
                         s3_ln_state_t prev_state);
static void     update_valid(s3_ir_cal_t * p_cal);

/*!
 * @brief Start uncalibrated, following, with the default gains.
 *
 * @param[in] l_side  S3_LN_L_ON_LEFT or S3_LN_L_ON_RIGHT: the side of
 *                    the robot the GP26 ("Left IR") sensor is on.
 */
void
s3_line_init (s3_line_t * p_ln, int32_t l_side)
{
    s3_ir_cal_init(&p_ln->cal_l);
    s3_ir_cal_init(&p_ln->cal_c);
    p_ln->l_right    = (l_side < 0) ? 1u : 0u;
    p_ln->kp_x100    = S3_LN_KP_X100;
    p_ln->kd_x100    = S3_LN_KD_X100;
    p_ln->boost_x100 = S3_LN_BOOST_X100;
    p_ln->e_prev     = 0;
    p_ln->state      = S3_LN_FOLLOW;
    p_ln->mode       = S3_LN_MODE_FOLLOW;
    p_ln->req_mode   = S3_LN_MODE_FOLLOW;
    p_ln->last_right = 0u;
    reset_tracking(p_ln);
}

/*!
 * @brief Set the PD gains (x100), clamped to 0 ... max.
 */
void
s3_line_set_gains (s3_line_t * p_ln, int32_t kp_x100, int32_t kd_x100)
{
    p_ln->kp_x100      = (int16_t) clamp_i32(kp_x100, 0, S3_LN_KP_MAX_X100);
    p_ln->kd_x100      = (int16_t) clamp_i32(kd_x100, 0, S3_LN_KD_MAX_X100);
    p_ln->e_prev_valid = 0u;
}

/*!
 * @brief Set the curve boost (x100, clamped to 0 ... max; 0 = off).
 */
void
s3_line_set_shape (s3_line_t * p_ln, int32_t boost_x100)
{
    p_ln->boost_x100 = (int16_t) clamp_i32(boost_x100, 0,
                                           S3_LN_BOOST_MAX_X100);
}

/*!
 * @brief Wheel-speed difference for S2 (IF-02), mm/s, from one step's
 *        output. Tracking: steer x steer_max. OFF LINE or LOST: x 5/4
 *        (S3_LN_SEARCH_NUM / DEN), so with steer_max = base speed and S2
 *        applying 50 %, full lock is base -/+ 5/8 base (inner wheel still
 *        forward at 3/8 base). Clamped to the 10-bit field. Shared by
 *        task_line and the host simulation.
 */
int32_t
s3_line_dv (s3_line_out_t const * p_out, int32_t steer_max_mms)
{
    bool    b_search = (0u != p_out->off_line) ||
                       (S3_LN_LOST == p_out->state);
    int32_t num      = b_search ? S3_LN_SEARCH_NUM : 1;
    int32_t den      = b_search ? (S3_LN_STEER_MAX * S3_LN_SEARCH_DEN) :
                                  S3_LN_STEER_MAX;

    return clamp_i32((p_out->steer * steer_max_mms * num) / den,
                     -S3_LN_DV_LIMIT, S3_LN_DV_LIMIT);
}

/*!
 * @brief Forget both calibration ranges (a new start-up sweep is needed).
 */
void
s3_line_recalibrate (s3_line_t * p_ln)
{
    s3_ir_cal_init(&p_ln->cal_l);
    s3_ir_cal_init(&p_ln->cal_c);
    p_ln->e_prev_valid = 0u;
}

/*!
 * @brief Switch mode now (resets the trackers). A search from the LEFT
 *        means the line is to the car's right, so OFF LINE after the
 *        line is found turns right (and the other way round).
 */
void
s3_line_set_mode (s3_line_t * p_ln, s3_ln_mode_t mode)
{
    if ((uint32_t) mode == p_ln->mode)
    {
        return;
    }
    p_ln->mode = (unsigned int) mode;
    reset_tracking(p_ln);
    if (S3_LN_MODE_SEARCH_FROM_LEFT == mode)
    {
        p_ln->last_right = 1u;
    }
    else if (S3_LN_MODE_SEARCH_FROM_RIGHT == mode)
    {
        p_ln->last_right = 0u;
    }
    else if (S3_LN_MODE_FOLLOW == mode)
    {
        p_ln->state = S3_LN_FOLLOW;
    }
    else
    {
        ;   /* DISABLED: state set by the next step. */
    }
}

/*!
 * @brief Call every step with the mode the flags ask for. Acts only when
 *        the REQUEST changes, so a search that has already found the line
 *        (internal mode FOLLOW) is not restarted while LF_SEARCH_x is
 *        still set.
 */
void
s3_line_request_mode (s3_line_t * p_ln, s3_ln_mode_t mode)
{
    if ((uint32_t) mode == p_ln->req_mode)
    {
        return;
    }
    p_ln->req_mode = (unsigned int) mode;
    s3_line_set_mode(p_ln, mode);
}

/*!
 * @brief One control step.
 *
 * @param[in]  raw_l     GP26 ("Left IR"), ADC counts (higher = darker).
 * @param[in]  raw_c     GP27 ("Centre IR"), ADC counts.
 * @param[in]  moved_um  Distance since the previous step (odometry; 0
 *                       while stopped, so a parked car never goes LOST).
 * @param[in]  dt_us     Real time since the previous step (D term).
 * @param[out] p_out     Readings, steering, state and events.
 */
void
s3_line_step (s3_line_t * p_ln, uint16_t raw_l, uint16_t raw_c,
              uint32_t moved_um, uint32_t dt_us, s3_line_out_t * p_out)
{
    s3_ln_state_t prev_state = (s3_ln_state_t) p_ln->state;
    uint16_t      filt_l;
    uint16_t      filt_c;
    int32_t       n_left;
    int32_t       n_right;
    int32_t       err;
    int32_t       steer;

    /* Calibration learns from the FILTERED value; control uses the RAW
     * one (the filter would add 30 ms of delay to the steering). */
    filt_l = s3_ir_cal_filter(&p_ln->cal_l, raw_l);
    filt_c = s3_ir_cal_filter(&p_ln->cal_c, raw_c);
    s3_ir_cal_learn(&p_ln->cal_l, filt_l);
    s3_ir_cal_learn(&p_ln->cal_c, filt_c);

    p_out->junction_event   = 0u;
    p_out->lost_event       = 0u;
    p_out->overshoot        = 0u;
    p_out->line_found_event = 0u;
    p_out->off_line         = 0u;
    p_out->calibrated       = ((0u != p_ln->cal_l.valid) &&
                               (0u != p_ln->cal_c.valid)) ? 1u : 0u;
    p_out->n_l              = s3_ir_cal_norm(&p_ln->cal_l, raw_l);
    p_out->n_c              = s3_ir_cal_norm(&p_ln->cal_c, raw_c);
    p_out->pos              = 0;
    p_out->steer            = 0;

    if (0u == p_out->calibrated)
    {
        p_out->state       = S3_LN_FOLLOW;
        p_ln->e_prev_valid = 0u;    /* No D kick on the first good step. */
        return;                     /* Never steer on guessed levels.    */
    }

    /* 0. The Vehicle Controller has the wheel: disabled or searching. */
    if (S3_LN_MODE_DISABLED == p_ln->mode)
    {
        p_ln->state  = S3_LN_DISABLED;
        p_out->state = S3_LN_DISABLED;
        return;                     /* Steer 0, no events.               */
    }

    n_left  = (0u != p_ln->l_right) ? p_out->n_c : p_out->n_l;
    n_right = (0u != p_ln->l_right) ? p_out->n_l : p_out->n_c;
    if (0 == search_step(p_ln, (n_left > n_right) ? n_left : n_right, p_out))
    {
        return;                     /* Still searching.                  */
    }

    /* 1. Line position: e > 0 = the right sensor is darker = the line is
     *    to the right. */
    err        = n_right - n_left;
    p_out->pos = err / 2;

    if ((n_left + n_right) > S3_LN_JUNC_SUM)    /* Car 30 Oct: 807 + 674. */
    {
        /* 2a. Both black: a junction or crossing line (a plain line is
         *     never under both). Straight across; one event after
         *     S3_LN_JUNC_MM, then JUNCTION until a sensor leaves black. */
        uint16_t prev_um = p_ln->both_black_um;

        p_ln->both_black_um = add_sat16(p_ln->both_black_um, moved_um);
        p_ln->both_white_um = 0u;
        p_ln->e_prev_valid  = 0u;   /* No D kick on the way out.         */
        steer               = 0;
        p_ln->steer_prev    = 0;
        if ((uint32_t) p_ln->both_black_um >= MM_TO_UM(S3_LN_JUNC_MM))
        {
            if ((uint32_t) prev_um < MM_TO_UM(S3_LN_JUNC_MM))
            {
                p_out->junction_event = 1u;
            }
            p_ln->state = S3_LN_JUNCTION;
        }
        else
        {
            p_ln->state = S3_LN_FOLLOW;
        }
    }
    else if ((n_left >= S3_LN_WHITE) || (n_right >= S3_LN_WHITE))
    {
        /* 2. A sensor sees the line: PD, and remember the side. */
        p_ln->both_black_um = 0u;
        steer = pd_steer(p_ln, err, dt_us, prev_state);
        p_ln->steer_prev = (int16_t) steer;
        if ((err > S3_LN_SIDE_MIN) || (err < -S3_LN_SIDE_MIN))
        {
            if (((err > 0) ? 1u : 0u) != p_ln->last_right)
            {
                p_ln->seen_black = 0u;  /* Crossed to the other sensor.  */
            }
            p_ln->last_right = (err > 0) ? 1u : 0u;
        }
        if ((n_left > S3_LN_BLACK) || (n_right > S3_LN_BLACK))
        {
            p_ln->seen_black = 1u;  /* The line is under this sensor.    */
        }
        p_out->overshoot    = ((n_left < S3_LN_WHITE) ||
                               (n_right < S3_LN_WHITE)) ? 1u : 0u;
        p_ln->both_white_um = 0u;
        p_ln->state         = S3_LN_FOLLOW;
    }
    else
    {
        /* 3. Both white. A weak reading still names the side.
         *    CENTRED (no sensor black since the line was last on the
         *    other sensor): it faded in between the spots, keep the
         *    last PD steer (a curve in progress), never ramp.
         *    OFF LINE: keep the last PD steer for S3_LN_HOLD_MM, then
         *    ramp from S3_LN_SOFT_PERMILLE to full lock at
         *    S3_LN_LOST_MM (state LOST from there). */
        p_ln->e_prev_valid  = 0u;   /* No D kick when the line returns.  */
        p_ln->both_black_um = 0u;
        p_out->off_line     = p_ln->seen_black;
        if (err > S3_LN_SIDE_MIN)
        {
            p_ln->last_right = 1u;
        }
        else if (err < -S3_LN_SIDE_MIN)
        {
            p_ln->last_right = 0u;
        }
        else
        {
            ;   /* No evidence: keep the last side. */
        }
        if (0u == p_ln->seen_black)
        {
            p_ln->both_white_um = add_sat16(p_ln->both_white_um,
                                            moved_um / S3_LN_CENTRE_DIV);
            steer               = 0;
        }
        else
        {
            p_ln->both_white_um = add_sat16(p_ln->both_white_um, moved_um);
            steer               = off_line_steer(p_ln->both_white_um);
        }
        if (steer < ((p_ln->steer_prev < 0) ? -p_ln->steer_prev :
                                              p_ln->steer_prev))
        {
            /* Never weaker than the turn in progress (curves). */
            steer = (p_ln->steer_prev < 0) ? -p_ln->steer_prev :
                                             p_ln->steer_prev;
        }
        if (0u == p_ln->last_right)
        {
            steer = -steer;
        }
        if ((uint32_t) p_ln->both_white_um >= MM_TO_UM(S3_LN_LOST_MM))
        {
            if (S3_LN_LOST != prev_state)
            {
                p_out->lost_event = 1u;
            }
            p_ln->state = S3_LN_LOST;
        }
        else
        {
            p_ln->state = S3_LN_FOLLOW;
        }
    }
    p_out->steer = steer;
    p_out->state = (s3_ln_state_t) p_ln->state;
}

/*!
 * @brief Search mode (Vehicle Controller): steer 0 until either sensor
 *        is above S3_LN_FOUND.
 *
 * @return 1 = following (not searching, or the line was just found);
 *         0 = still searching (p_out->state set).
 */
static int
search_step (s3_line_t * p_ln, int32_t n_max, s3_line_out_t * p_out)
{
    if ((S3_LN_MODE_SEARCH_FROM_LEFT != p_ln->mode) &&
        (S3_LN_MODE_SEARCH_FROM_RIGHT != p_ln->mode))
    {
        return 1;
    }
    if (n_max <= S3_LN_FOUND)
    {
        p_ln->state  = S3_LN_SEARCH;
        p_out->state = S3_LN_SEARCH;
        return 0;
    }
    p_out->line_found_event = 1u;
    p_ln->mode              = S3_LN_MODE_FOLLOW;
    reset_tracking(p_ln);
    p_ln->seen_black        = 1u;   /* Losing it again = OFF LINE to the */
                                    /* search side, not CENTRED.          */
    p_ln->state             = S3_LN_FOLLOW;
    return 1;
}

/*!
 * @brief PD steering on e = n_RIGHT - n_LEFT, with the optional curve
 *        boost once one sensor is off the line.
 *
 * @return Steering, permille, + = right.
 */
static int32_t
pd_steer (s3_line_t * p_ln, int32_t err, uint32_t dt_us,
          s3_ln_state_t prev_state)
{
    int32_t p_term;
    int32_t d_term = 0;
    int32_t d_err;

    if (S3_LN_FOLLOW != prev_state)
    {
        p_ln->e_prev_valid = 0u;    /* No D across a state jump.         */
    }
    p_term = (int32_t) p_ln->kp_x100 * err;
    if (err > S3_LN_BOOST_FROM)
    {
        p_term += (int32_t) p_ln->boost_x100 * (err - S3_LN_BOOST_FROM);
    }
    else if (err < -S3_LN_BOOST_FROM)
    {
        p_term += (int32_t) p_ln->boost_x100 * (err + S3_LN_BOOST_FROM);
    }
    else
    {
        ;   /* Near the centre: Kp only. */
    }

    if ((0u != p_ln->e_prev_valid) && (dt_us > 0u))
    {
        /* |de| <= 2000 and REF_DT = 10 000, so de * REF_DT <= 2e7 fits.
         * d_err is clamped BEFORE the multiply: after a late wake-up dt
         * can be a few us, and Kd * d_err would overflow 32 bits. */
        int32_t dt = (dt_us > (uint32_t) INT32_MAX) ? INT32_MAX :
                                                      (int32_t) dt_us;

        d_err  = ((err - p_ln->e_prev) * (int32_t) S3_LN_REF_DT_US) / dt;
        d_err  = clamp_i32(d_err, -S3_LN_D_LIMIT * X100,
                           S3_LN_D_LIMIT * X100);
        d_term = clamp_i32((int32_t) p_ln->kd_x100 * d_err,
                           -S3_LN_D_LIMIT * X100, S3_LN_D_LIMIT * X100);
    }
    p_ln->e_prev       = (int16_t) err;
    p_ln->e_prev_valid = 1u;
    return clamp_i32((p_term + d_term) / X100, -S3_LN_STEER_MAX,
                     S3_LN_STEER_MAX);
}

static void
reset_tracking (s3_line_t * p_ln)
{
    p_ln->e_prev_valid  = 0u;
    p_ln->both_white_um = 0u;
    p_ln->both_black_um = 0u;
    p_ln->steer_prev    = 0;
    p_ln->seen_black    = 0u;
}

/*!
 * @brief Steering size while both sensors read white, from the distance
 *        driven since: 0 up to S3_LN_HOLD_MM, then a ramp from
 *        S3_LN_SOFT_PERMILLE to full lock at S3_LN_LOST_MM.
 *
 * @return Permille, 0 ... S3_LN_STEER_MAX (the caller applies the side).
 */
static int32_t
off_line_steer (uint16_t white_um)
{
    int32_t hold_um = (int32_t) MM_TO_UM(S3_LN_HOLD_MM);
    int32_t lost_um = (int32_t) MM_TO_UM(S3_LN_LOST_MM);
    int32_t d_um    = (int32_t) white_um;

    if (d_um < hold_um)
    {
        return 0;
    }
    if (d_um >= lost_um)
    {
        return S3_LN_STEER_MAX;
    }
    return S3_LN_SOFT_PERMILLE +
           (((S3_LN_STEER_MAX - S3_LN_SOFT_PERMILLE) * (d_um - hold_um)) /
            (lost_um - hold_um));
}

static int32_t
clamp_i32 (int32_t value, int32_t lo, int32_t hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

/* Saturating 16-bit distance accumulator. */
static uint16_t
add_sat16 (uint16_t acc, uint32_t add)
{
    uint32_t sum = (uint32_t) acc + add;

    return (uint16_t) ((sum > SAT16) ? SAT16 : sum);
}

/* ------------------------------------------------------- calibration */

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
    if (filt >= S3_IR_CAL_LIFT_RAW)
    {
        return;     /* No floor under the sensor (car lifted): not black. */
    }
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
