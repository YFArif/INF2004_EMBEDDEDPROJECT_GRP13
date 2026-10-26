/*
 *	s3_line.c -- see s3_line.h
 */
#include "s3_line.h"

#define S3_LN_UM(mm)    ((uint32_t)(mm) * 1000U)
#define S3_SAT16        0xFFFFU

static int32_t clamp_i32 (int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* saturating 16-bit distance accumulator */
static uint16_t add_sat16 (uint16_t acc, uint32_t add)
{
    uint32_t s = (uint32_t)acc + add;
    return (uint16_t)((s > S3_SAT16) ? S3_SAT16 : s);
}

/* ---------------------------------------------------------------- calibration */
static void cal_update_valid (s3_cal_t * p_cal)
{
    p_cal->valid = ((p_cal->black > p_cal->white) &&
                    ((uint32_t)(p_cal->black - p_cal->white) >= S3_LN_MIN_RANGE)) ? 1U : 0U;
}

void s3_cal_init (s3_cal_t * p_cal)
{
    p_cal->white   = 4095U;
    p_cal->black   = 0U;
    p_cal->valid   = 0U;
    p_cal->filt    = 0U;
    p_cal->win_lo  = 255U;
    p_cal->win_hi  = 0U;
    p_cal->started = 0U;
}

/* 4-sample IIR (40 ms at 10 ms steps): one noisy or glitched sample moves
   the value by only a quarter, so it cannot set a calibration extreme. */
uint16_t s3_cal_filter (s3_cal_t * p_cal, uint16_t raw)
{
    raw = (raw > 4095U) ? 4095U : raw;
    if (0U == p_cal->started)
    {
        p_cal->filt    = raw;
        p_cal->started = 1U;
    }
    else
    {
        p_cal->filt = (3U * (uint32_t)p_cal->filt + (uint32_t)raw + 2U) / 4U;
    }
    return (uint16_t)p_cal->filt;
}

void s3_cal_learn (s3_cal_t * p_cal, uint16_t filt)
{
    filt = (filt > 4095U) ? 4095U : filt;
    if (filt < p_cal->white)
    {
        p_cal->white = filt;
    }
    if (filt > p_cal->black)
    {
        p_cal->black = filt;
    }
    if ((uint32_t)(filt >> 4) < p_cal->win_lo)
    {
        p_cal->win_lo = (uint32_t)(filt >> 4);
    }
    if ((uint32_t)(filt >> 4) > p_cal->win_hi)
    {
        p_cal->win_hi = (uint32_t)(filt >> 4);
    }
    cal_update_valid(p_cal);
}

/* Each end moves 1/8 of the way towards what was seen in the last window,
   but only if the window really contained something on that side
   (>= 600 counts away from the other end).  A sensor that saw only
   white keeps its black level; a one-off extreme fades in ~10-20 s once
   normal readings return.  Never shrinks below S3_LN_MIN_RANGE. */
void s3_cal_relax (s3_cal_t * p_cal)
{
    uint32_t black = p_cal->black;
    uint32_t white = p_cal->white;
    uint32_t hi    = ((uint32_t)p_cal->win_hi << 4) + 8U;   /* back to 12-bit, mid-step */
    uint32_t lo    = ((uint32_t)p_cal->win_lo << 4) + 8U;
    uint32_t new_black = black;
    uint32_t new_white = white;

    if ((0U != p_cal->valid) && (p_cal->win_hi >= p_cal->win_lo))
    {
        if ((hi >= (white + S3_LN_MIN_RANGE)) && (hi < black))
        {
            new_black = black - ((black - hi + 7U) / 8U);
        }
        if (((lo + S3_LN_MIN_RANGE) <= black) && (lo > white))
        {
            new_white = white + ((lo - white + 7U) / 8U);
        }
        if ((new_black > new_white) && ((new_black - new_white) >= S3_LN_MIN_RANGE))
        {
            p_cal->black = new_black;
            p_cal->white = new_white;
        }
    }
    p_cal->win_lo = 255U;
    p_cal->win_hi = 0U;
}

int32_t s3_cal_norm (s3_cal_t const * p_cal, uint16_t value)
{
    int32_t range = (int32_t)p_cal->black - (int32_t)p_cal->white;

    if (range <= 0)
    {
        return 0;
    }
    return clamp_i32((((int32_t)value - (int32_t)p_cal->white) * 1000) / range, 0, 1000);
}

/* ---------------------------------------------------------------- line following */
static void reset_tracking (s3_line_t * p_ln)
{
    p_ln->e_prev_valid  = 0U;
    p_ln->both_black_um = 0U;
    p_ln->c_black_um    = 0U;
    p_ln->c_black_at_l  = S3_SAT16;
    p_ln->l_was_black   = 0U;
    p_ln->both_white_um = 0U;
    p_ln->seen_black    = 0U;
}

void s3_line_init (s3_line_t * p_ln, int32_t edge_side)
{
    s3_cal_init(&p_ln->cal_l);
    s3_cal_init(&p_ln->cal_c);
    p_ln->edge_right   = (edge_side < 0) ? 1U : 0U;
    p_ln->kp_x100      = S3_LN_KP_X100;
    p_ln->kd_x100      = S3_LN_KD_X100;
    p_ln->e_prev       = 0;
    p_ln->state        = S3_LN_FOLLOW;
    p_ln->mode         = S3_LN_MODE_FOLLOW;
    p_ln->req_mode     = S3_LN_MODE_FOLLOW;
    p_ln->not_junc_um  = (uint16_t)S3_LN_UM(S3_LN_REARM_MM);
    p_ln->junc_armed   = 1U;
    p_ln->last_steer   = 0;
    reset_tracking(p_ln);
}

void s3_line_set_gains (s3_line_t * p_ln, int32_t kp_x100, int32_t kd_x100)
{
    p_ln->kp_x100      = (int16_t)clamp_i32(kp_x100, 0, 2000);
    p_ln->kd_x100      = (int16_t)clamp_i32(kd_x100, 0, 4000);
    p_ln->e_prev_valid = 0U;
}

void s3_line_recalibrate (s3_line_t * p_ln)
{
    s3_cal_init(&p_ln->cal_l);
    s3_cal_init(&p_ln->cal_c);
    p_ln->e_prev_valid = 0U;
}

void s3_line_set_mode (s3_line_t * p_ln, s3_ln_mode_t mode)
{
    if ((uint32_t)mode == p_ln->mode)
    {
        return;
    }
    p_ln->mode = (unsigned int)mode;
    reset_tracking(p_ln);
    if (S3_LN_MODE_FOLLOW == mode)
    {
        p_ln->state = S3_LN_FOLLOW;
    }
}

void s3_line_request_mode (s3_line_t * p_ln, s3_ln_mode_t mode)
{
    if ((uint32_t)mode == p_ln->req_mode)
    {
        return;                 /* same request: keep what the line logic decided */
    }
    p_ln->req_mode = (unsigned int)mode;
    s3_line_set_mode(p_ln, mode);
}

void s3_line_step (s3_line_t * p_ln, uint16_t raw_l, uint16_t raw_c,
                   uint32_t moved_um, uint32_t dt_us, s3_line_out_t * p_out)
{
    int32_t       e, p_term, d_term, de, steer;
    int32_t       side = (0U != p_ln->edge_right) ? S3_LN_EDGE_RIGHT : S3_LN_EDGE_LEFT;
    uint16_t      f_l, f_c;
    uint8_t       l_black, c_black, l_white, c_white;
    s3_ln_state_t prev_state = (s3_ln_state_t)p_ln->state;

    f_l = s3_cal_filter(&p_ln->cal_l, raw_l);
    f_c = s3_cal_filter(&p_ln->cal_c, raw_c);
    s3_cal_learn(&p_ln->cal_l, f_l);
    s3_cal_learn(&p_ln->cal_c, f_c);

    p_out->junction_event   = 0U;
    p_out->lost_event       = 0U;
    p_out->overshoot        = 0U;
    p_out->line_found_event = 0U;
    p_out->calibrated       = ((0U != p_ln->cal_l.valid) && (0U != p_ln->cal_c.valid)) ? 1U : 0U;
    /* control and events use the RAW reading (no lag: an 18 mm crossing line
       at 0.3 m/s lasts only 6 steps); only calibration uses the filtered one */
    p_out->n_l              = s3_cal_norm(&p_ln->cal_l, raw_l);
    p_out->n_c              = s3_cal_norm(&p_ln->cal_c, raw_c);
    p_out->pos              = 0;
    p_out->steer            = 0;

    if (0U == p_out->calibrated)
    {
        p_out->state       = S3_LN_FOLLOW;
        p_ln->e_prev_valid = 0U;                 /* no D kick on the first calibrated step */
        return;                                  /* never steer on guessed levels */
    }

    /* 0. Vehicle Controller has the wheel: disabled or searching */
    if (S3_LN_MODE_DISABLED == p_ln->mode)
    {
        p_ln->state  = S3_LN_DISABLED;
        p_out->state = S3_LN_DISABLED;
        return;                                  /* steer 0, no junction/lost during a turn or bypass */
    }
    if ((S3_LN_MODE_SEARCH_FROM_LEFT == p_ln->mode) || (S3_LN_MODE_SEARCH_FROM_RIGHT == p_ln->mode))
    {
        /* Centre meets "our" edge first only when the car comes from the side
           that edge is on; from the other side it must cross the line first,
           otherwise it would lock onto the wrong edge and steer backwards. */
        uint8_t direct = ((S3_LN_MODE_SEARCH_FROM_LEFT == p_ln->mode) ^ (0U != p_ln->edge_right)) ? 1U : 0U;
        uint8_t found;

        if (p_out->n_c > S3_LN_BLACK)
        {
            p_ln->seen_black = 1U;
        }
        found = (0U != direct) ? ((p_out->n_c > 400) ? 1U : 0U)
                               : (((0U != p_ln->seen_black) && (p_out->n_c < 600)) ? 1U : 0U);
        if (0U != found)
        {
            p_out->line_found_event = 1U;
            p_ln->mode        = S3_LN_MODE_FOLLOW;
            reset_tracking(p_ln);
            p_ln->junc_armed  = 0U;              /* do not call the re-entry a junction */
            p_ln->not_junc_um = 0U;
            p_ln->state       = S3_LN_FOLLOW;
        }
        else
        {
            p_ln->state  = S3_LN_SEARCH;
            p_out->state = S3_LN_SEARCH;
            return;
        }
    }

    l_black = (p_out->n_l > S3_LN_BLACK) ? 1U : 0U;
    c_black = (p_out->n_c > S3_LN_BLACK) ? 1U : 0U;
    l_white = (p_out->n_l < S3_LN_WHITE) ? 1U : 0U;
    c_white = (p_out->n_c < S3_LN_WHITE) ? 1U : 0U;

    /* 1. Line position estimate and PD steering.
          e > 0: more black than half under Centre = drifted onto the line.
          D is "change per 10 ms", so the same Kd works if the period varies. */
    if (S3_LN_FOLLOW != prev_state)
    {
        p_ln->e_prev_valid = 0U;                 /* leaving LOST/JUNCTION/SEARCH: no D across the jump */
    }
    e      = p_out->n_c - 500;
    p_term = (int32_t)p_ln->kp_x100 * e;
    /* Curve boost: outside the EDGE band the Centre has left the edge, which
       on a curve means the line is bending away.  Extra steering grows to
       full lock at all-white / all-black, so curves get full authority while
       the EDGE band (straight line) keeps the gentle Kp. */
    if (e > (S3_LN_BLACK - 500))
    {
        p_term += S3_LN_BOOST_X100 * (e - (S3_LN_BLACK - 500));
    }
    else if (e < (S3_LN_WHITE - 500))
    {
        p_term += S3_LN_BOOST_X100 * (e - (S3_LN_WHITE - 500));
    }
    d_term = 0;
    if ((0U != p_ln->e_prev_valid) && (dt_us > 0U))
    {
        de     = (int32_t)(((int64_t)(e - p_ln->e_prev) * (int64_t)S3_LN_REF_DT_US) / (int64_t)dt_us);
        d_term = clamp_i32((int32_t)p_ln->kd_x100 * de, -S3_LN_D_LIMIT * 100, S3_LN_D_LIMIT * 100);
    }
    p_ln->e_prev       = (int16_t)e;
    p_ln->e_prev_valid = 1U;
    p_out->pos = e;
    steer      = clamp_i32(-side * ((p_term + d_term) / 100), -1000, 1000);

    /* 2. Overshoot: drifted past the line -> outer sensor on it, Centre on the
          far white.  Steer hard back (design 6.1). */
    if ((0U != l_black) && (0U != c_white))
    {
        p_out->overshoot = 1U;
        steer            = -side * 1000;
    }

    /* 3. Junction: L AND C black over S3_LN_JUNC_MM of travel, and L turned
          black no later than S3_LN_JUNC_LEAD_MM after Centre (and not while
          Centre was white = overshoot).  Drift / hard curve / overshoot
          recovery are steered, not reported as a junction. */
    p_ln->c_black_um = (0U != c_black) ? add_sat16(p_ln->c_black_um, moved_um) : 0U;
    if ((0U != l_black) && (0U == p_ln->l_was_black))
    {
        p_ln->c_black_at_l = (0U != c_white) ? S3_SAT16 : p_ln->c_black_um;
    }
    p_ln->l_was_black = l_black;

    if ((0U != l_black) && (0U != c_black) &&
        ((uint32_t)p_ln->c_black_at_l <= S3_LN_UM(S3_LN_JUNC_LEAD_MM)))
    {
        p_ln->both_black_um = add_sat16(p_ln->both_black_um, moved_um);
    }
    else
    {
        p_ln->both_black_um = 0U;
        p_ln->not_junc_um   = add_sat16(p_ln->not_junc_um, moved_um);
        if ((uint32_t)p_ln->not_junc_um >= S3_LN_UM(S3_LN_REARM_MM))
        {
            p_ln->junc_armed = 1U;
        }
    }

    /* 4. Lost: L AND C white over S3_LN_LOST_MM of travel. */
    p_ln->both_white_um = ((0U != l_white) && (0U != c_white)) ? add_sat16(p_ln->both_white_um, moved_um) : 0U;

    /* 5. State machine */
    if ((0U != p_ln->junc_armed) && ((uint32_t)p_ln->both_black_um >= S3_LN_UM(S3_LN_JUNC_MM)))
    {
        p_out->junction_event = 1U;
        p_ln->junc_armed      = 0U;
        p_ln->not_junc_um     = 0U;
        p_ln->state           = S3_LN_JUNCTION;
    }
    else if ((uint32_t)p_ln->both_white_um >= S3_LN_UM(S3_LN_LOST_MM))
    {
        if (S3_LN_LOST != prev_state)
        {
            p_out->lost_event = 1U;
        }
        p_ln->state = S3_LN_LOST;
    }
    else if ((S3_LN_JUNCTION == prev_state) && (0U != l_black) && (0U != c_black))
    {
        p_ln->state = S3_LN_JUNCTION;            /* still crossing the junction */
    }
    else
    {
        p_ln->state = S3_LN_FOLLOW;
    }

    /* 6. What to steer in each state */
    if (S3_LN_FOLLOW == p_ln->state)
    {
        p_ln->last_steer = (int16_t)steer;
    }
    else if (S3_LN_JUNCTION == p_ln->state)
    {
        steer = 0;                               /* cross straight; the Vehicle Controller decides turns */
    }
    else
    {
        steer = p_ln->last_steer;                /* lost: keep the last correction (turns back towards the line) */
    }
    p_out->steer = steer;
    p_out->state = (s3_ln_state_t)p_ln->state;
}
