/** @file s3_line.c
 *
 * @brief Subsystem 3 -- line-following combiner. See s3_line.h.
 */

#include <stdint.h>

#include "s3_ir_cal.h"
#include "s3_line.h"

#define UM_PER_MM       (1000u)
#define MM_TO_UM(mm)    ((uint32_t) (mm) * UM_PER_MM)
#define SAT16           (0xFFFFu)
#define X100            (100)

static int32_t  clamp_i32(int32_t value, int32_t lo, int32_t hi);
static uint16_t add_sat16(uint16_t acc, uint32_t add);
static void     reset_tracking(s3_line_t * p_ln);
static int32_t  edge_sign(s3_line_t const * p_ln);
static int      search_step(s3_line_t * p_ln, s3_line_out_t * p_out);
static int32_t  pd_steer(s3_line_t * p_ln, int32_t n_c, uint32_t dt_us,
                         s3_ln_state_t prev_state, int32_t * p_pos);
static void     track_distances(s3_line_t * p_ln, uint8_t b_l_black,
                                uint8_t b_c_black, uint8_t b_l_white,
                                uint8_t b_c_white, uint32_t moved_um);
static void     update_state(s3_line_t * p_ln, s3_ln_state_t prev_state,
                             uint8_t b_l_black, uint8_t b_c_black,
                             s3_line_out_t * p_out);

/*!
 * @brief Start uncalibrated, following, with the default gains.
 *
 * @param[in] edge_side  S3_LN_EDGE_LEFT or S3_LN_EDGE_RIGHT.
 */
void
s3_line_init (s3_line_t * p_ln, int32_t edge_side)
{
    s3_ir_cal_init(&p_ln->cal_l);
    s3_ir_cal_init(&p_ln->cal_c);
    p_ln->edge_right  = (edge_side < 0) ? 1u : 0u;
    p_ln->kp_x100     = S3_LN_KP_X100;
    p_ln->kd_x100     = S3_LN_KD_X100;
    p_ln->e_prev      = 0;
    p_ln->state       = S3_LN_FOLLOW;
    p_ln->mode        = S3_LN_MODE_FOLLOW;
    p_ln->req_mode    = S3_LN_MODE_FOLLOW;
    p_ln->not_junc_um = (uint16_t) MM_TO_UM(S3_LN_REARM_MM);
    p_ln->junc_armed  = 1u;
    p_ln->last_steer  = 0;
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
 * @brief Switch mode now (resets the distance trackers).
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
    if (S3_LN_MODE_FOLLOW == mode)
    {
        p_ln->state = S3_LN_FOLLOW;
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
 * @param[in]  raw_l     Left IR, ADC counts (higher = darker).
 * @param[in]  raw_c     Centre, ADC counts.
 * @param[in]  moved_um  Distance since the previous step (odometry; 0
 *                       while stopped, so a parked car never raises
 *                       junction or lost).
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
    uint8_t       b_l_black;
    uint8_t       b_c_black;
    uint8_t       b_l_white;
    uint8_t       b_c_white;
    int32_t       steer;

    /* Calibration learns from the FILTERED value; control and events use
     * the RAW one (an 18 mm crossing line at 0.3 m/s lasts only 6 steps). */
    filt_l = s3_ir_cal_filter(&p_ln->cal_l, raw_l);
    filt_c = s3_ir_cal_filter(&p_ln->cal_c, raw_c);
    s3_ir_cal_learn(&p_ln->cal_l, filt_l);
    s3_ir_cal_learn(&p_ln->cal_c, filt_c);

    p_out->junction_event   = 0u;
    p_out->lost_event       = 0u;
    p_out->overshoot        = 0u;
    p_out->line_found_event = 0u;
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
    if (0 == search_step(p_ln, p_out))
    {
        return;                     /* Still searching.                  */
    }

    b_l_black = (p_out->n_l > S3_LN_BLACK) ? 1u : 0u;
    b_c_black = (p_out->n_c > S3_LN_BLACK) ? 1u : 0u;
    b_l_white = (p_out->n_l < S3_LN_WHITE) ? 1u : 0u;
    b_c_white = (p_out->n_c < S3_LN_WHITE) ? 1u : 0u;

    /* 1. Line position and PD steering. */
    steer = pd_steer(p_ln, p_out->n_c, dt_us, prev_state, &p_out->pos);

    /* 2. Overshoot: drifted past the line -> outer sensor on it, Centre
     *    on the far white. Steer hard back (design 6.1). */
    if ((0u != b_l_black) && (0u != b_c_white))
    {
        p_out->overshoot = 1u;
        steer            = -edge_sign(p_ln) * S3_LN_STEER_MAX;
    }

    /* 3. and 4. Junction and lost distances, 5. state machine. */
    track_distances(p_ln, b_l_black, b_c_black, b_l_white, b_c_white,
                    moved_um);
    update_state(p_ln, prev_state, b_l_black, b_c_black, p_out);

    /* 6. What to steer in each state. */
    if (S3_LN_FOLLOW == p_ln->state)
    {
        p_ln->last_steer = (int16_t) steer;
    }
    else if (S3_LN_JUNCTION == p_ln->state)
    {
        steer = 0;      /* Cross straight; the VC decides turns.         */
    }
    else
    {
        steer = p_ln->last_steer;   /* Lost: keep turning back.          */
    }
    p_out->steer = steer;
    p_out->state = (s3_ln_state_t) p_ln->state;
}

/*!
 * @brief Search mode: wait for the CORRECT edge. The Centre meets "our"
 *        edge first only when the car comes from that edge's side; from
 *        the other side it must cross the line first, otherwise it would
 *        lock onto the wrong edge and steer backwards.
 *
 * @return 1 = following (not searching, or the line was just found);
 *         0 = still searching (p_out->state set).
 */
static int
search_step (s3_line_t * p_ln, s3_line_out_t * p_out)
{
    uint8_t b_direct;
    uint8_t b_found;

    if ((S3_LN_MODE_SEARCH_FROM_LEFT != p_ln->mode) &&
        (S3_LN_MODE_SEARCH_FROM_RIGHT != p_ln->mode))
    {
        return 1;
    }
    b_direct = ((S3_LN_MODE_SEARCH_FROM_LEFT == p_ln->mode) ^
                (0u != p_ln->edge_right)) ? 1u : 0u;
    if (p_out->n_c > S3_LN_BLACK)
    {
        p_ln->seen_black = 1u;
    }
    if (0u != b_direct)
    {
        b_found = (p_out->n_c > S3_LN_FOUND_DIRECT) ? 1u : 0u;
    }
    else
    {
        b_found = ((0u != p_ln->seen_black) &&
                   (p_out->n_c < S3_LN_FOUND_CROSSED)) ? 1u : 0u;
    }
    if (0u == b_found)
    {
        p_ln->state  = S3_LN_SEARCH;
        p_out->state = S3_LN_SEARCH;
        return 0;
    }
    p_out->line_found_event = 1u;
    p_ln->mode              = S3_LN_MODE_FOLLOW;
    reset_tracking(p_ln);
    p_ln->junc_armed        = 0u;   /* The re-entry is not a junction.   */
    p_ln->not_junc_um       = 0u;
    p_ln->state             = S3_LN_FOLLOW;
    return 1;
}

/*!
 * @brief PD steering. e > 0: more than half black under the Centre =
 *        drifted onto the line.
 *
 * @return Steering, permille, before the overshoot rule.
 */
static int32_t
pd_steer (s3_line_t * p_ln, int32_t n_c, uint32_t dt_us,
          s3_ln_state_t prev_state, int32_t * p_pos)
{
    int32_t err;
    int32_t p_term;
    int32_t d_term = 0;
    int32_t d_err;

    if (S3_LN_FOLLOW != prev_state)
    {
        p_ln->e_prev_valid = 0u;    /* No D across a state jump.         */
    }
    err    = n_c - S3_LN_EDGE_MID;
    p_term = (int32_t) p_ln->kp_x100 * err;

    /* Curve boost: outside the EDGE band the Centre has left the edge,
     * which on a curve means the line is bending away. The extra term
     * grows to full lock at all-white / all-black, while the EDGE band
     * (straight line) keeps the gentle Kp. */
    if (err > (S3_LN_BLACK - S3_LN_EDGE_MID))
    {
        p_term += S3_LN_BOOST_X100 * (err - (S3_LN_BLACK - S3_LN_EDGE_MID));
    }
    else if (err < (S3_LN_WHITE - S3_LN_EDGE_MID))
    {
        p_term += S3_LN_BOOST_X100 * (err - (S3_LN_WHITE - S3_LN_EDGE_MID));
    }
    else
    {
        ;   /* EDGE band: Kp only. */
    }

    if ((0u != p_ln->e_prev_valid) && (dt_us > 0u))
    {
        d_err  = (int32_t) (((int64_t) (err - p_ln->e_prev) *
                             (int64_t) S3_LN_REF_DT_US) / (int64_t) dt_us);
        d_term = clamp_i32((int32_t) p_ln->kd_x100 * d_err,
                           -S3_LN_D_LIMIT * X100, S3_LN_D_LIMIT * X100);
    }
    p_ln->e_prev       = (int16_t) err;
    p_ln->e_prev_valid = 1u;
    *p_pos             = err;
    return clamp_i32(-edge_sign(p_ln) * ((p_term + d_term) / X100),
                     -S3_LN_STEER_MAX, S3_LN_STEER_MAX);
}

/*!
 * @brief Junction and lost distance trackers.
 *
 * Junction: L AND C black over S3_LN_JUNC_MM, and L turned black no
 * later than S3_LN_JUNC_LEAD_MM after the Centre (and not while the
 * Centre was white = overshoot). Drift, a hard curve or overshoot
 * recovery are steered, not reported. Lost: L AND C white.
 */
static void
track_distances (s3_line_t * p_ln, uint8_t b_l_black, uint8_t b_c_black,
                 uint8_t b_l_white, uint8_t b_c_white, uint32_t moved_um)
{
    p_ln->c_black_um = (0u != b_c_black) ?
                       add_sat16(p_ln->c_black_um, moved_um) : 0u;
    if ((0u != b_l_black) && (0u == p_ln->l_was_black))
    {
        p_ln->c_black_at_l = (0u != b_c_white) ? SAT16 : p_ln->c_black_um;
    }
    p_ln->l_was_black = b_l_black;

    if ((0u != b_l_black) && (0u != b_c_black) &&
        ((uint32_t) p_ln->c_black_at_l <= MM_TO_UM(S3_LN_JUNC_LEAD_MM)))
    {
        p_ln->both_black_um = add_sat16(p_ln->both_black_um, moved_um);
    }
    else
    {
        p_ln->both_black_um = 0u;
        p_ln->not_junc_um   = add_sat16(p_ln->not_junc_um, moved_um);
        if ((uint32_t) p_ln->not_junc_um >= MM_TO_UM(S3_LN_REARM_MM))
        {
            p_ln->junc_armed = 1u;
        }
    }

    p_ln->both_white_um = ((0u != b_l_white) && (0u != b_c_white)) ?
                          add_sat16(p_ln->both_white_um, moved_um) : 0u;
}

/*!
 * @brief State machine: JUNCTION, LOST or FOLLOW, with the events.
 */
static void
update_state (s3_line_t * p_ln, s3_ln_state_t prev_state, uint8_t b_l_black,
              uint8_t b_c_black, s3_line_out_t * p_out)
{
    if ((0u != p_ln->junc_armed) &&
        ((uint32_t) p_ln->both_black_um >= MM_TO_UM(S3_LN_JUNC_MM)))
    {
        p_out->junction_event = 1u;
        p_ln->junc_armed      = 0u;
        p_ln->not_junc_um     = 0u;
        p_ln->state           = S3_LN_JUNCTION;
    }
    else if ((uint32_t) p_ln->both_white_um >= MM_TO_UM(S3_LN_LOST_MM))
    {
        if (S3_LN_LOST != prev_state)
        {
            p_out->lost_event = 1u;
        }
        p_ln->state = S3_LN_LOST;
    }
    else if ((S3_LN_JUNCTION == prev_state) && (0u != b_l_black) &&
             (0u != b_c_black))
    {
        p_ln->state = S3_LN_JUNCTION;   /* Still crossing the junction.  */
    }
    else
    {
        p_ln->state = S3_LN_FOLLOW;
    }
}

static void
reset_tracking (s3_line_t * p_ln)
{
    p_ln->e_prev_valid  = 0u;
    p_ln->both_black_um = 0u;
    p_ln->c_black_um    = 0u;
    p_ln->c_black_at_l  = SAT16;
    p_ln->l_was_black   = 0u;
    p_ln->both_white_um = 0u;
    p_ln->seen_black    = 0u;
}

static int32_t
edge_sign (s3_line_t const * p_ln)
{
    return (0u != p_ln->edge_right) ? S3_LN_EDGE_RIGHT : S3_LN_EDGE_LEFT;
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

/*** end of file ***/
