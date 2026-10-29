/** @file s3_probe.c
 * @brief Brake on GP27 loss, bounded search, optionally one armed crossing.
 *
 * A fully black single sensor has no lateral direction information.
 * Search is therefore explicit, not a claim of continuous black tracking.
 */
#include "s3_probe.h"

#define BRAKE_MS (120u)
#define MOTION_TIMEOUT_MS (8000u)
#define SEARCH_STEPS (5u)
#define SEARCH_MAX_CDEG (1800)
#define FOUND_SAMPLES (3u)
#define CLEAR_SAMPLES (5u)
#define JUNCTION_CONFIRM_MM (2)
#define ALIGN_MAX_MM (160)

static int16_t const g_search_deg[SEARCH_STEPS] = {4, -8, 16, -24, 12};

static void stop_with(s3_probe_t * p, s3_probe_reason_t reason,
                       s3_probe_output_t * p_out);
static void brake_begin(s3_probe_t * p, s3_probe_input_t const * p_in,
                         bool b_verify, s3_probe_output_t * p_out);
static void search_next(s3_probe_t * p, s3_probe_input_t const * p_in,
                         s3_probe_output_t * p_out);

void
s3_probe_start (s3_probe_t * p, s3_probe_input_t const * p_in,
                 bool b_junctions)
{
    *p = (s3_probe_t) {0};
    p->state = S3_PROBE_FOLLOW;
    p->since_ms = p_in->now_ms;
    p->start_mm = p_in->dist_mm;
    p->junctions = b_junctions ? 1u : 0u;
}

void
s3_probe_step (s3_probe_t * p, s3_probe_input_t const * p_in,
                s3_probe_output_t * p_out)
{
    int32_t angle;
    int32_t travel;

    *p_out = (s3_probe_output_t) {S3_PROBE_NONE, 0};
    if ((S3_PROBE_STOPPED == p->state) || (S3_PROBE_FAULT == p->state))
    {
        return;
    }
    if (p_in->b_estop || p_in->b_stop)
    {
        stop_with(p, p_in->b_estop ? S3_PROBE_ESTOP : S3_PROBE_OK, p_out);
        return;
    }
    if ((S3_PROBE_FOLLOW != p->state) &&
        ((p_in->now_ms - p->since_ms) >= MOTION_TIMEOUT_MS))
    {
        stop_with(p, S3_PROBE_TIMEOUT, p_out);
        return;
    }
    switch (p->state)
    {
        case S3_PROBE_FOLLOW:
            if (p_in->b_arm && (0u != p->junctions) && (0u == p->armed))
            {
                p->armed = 1u;
                p->eligible = 0u;
                p->candidate = 0u;
                p->clear = 0u;
            }
            if (p_in->left < S3_PROBE_LOSS)
            {
                p->heading_cdeg = p_in->heading_cdeg;
                p->search_step = 0u;
                brake_begin(p, p_in, false, p_out);
                break;
            }
            if (p_in->right < S3_PROBE_LOSS)
            {
                if (p->clear < CLEAR_SAMPLES)
                {
                    p->clear++;
                }
                if (p->clear >= CLEAR_SAMPLES)
                {
                    p->eligible = 1u;
                }
            }
            else
            {
                p->clear = 0u;
            }
            if ((0u != p->armed) && (0u != p->eligible) &&
                (p_in->left >= S3_PROBE_BLACK) &&
                (p_in->right >= S3_PROBE_BLACK))
            {
                if (0u == p->candidate)
                {
                    p->mark_mm = p_in->dist_mm;
                    p->candidate = 1u;
                }
                else if ((p_in->dist_mm - p->mark_mm) >= JUNCTION_CONFIRM_MM)
                {
                    p->armed = 0u;
                    p->eligible = 0u;
                    p->candidate = 0u;
                    p->since_ms = p_in->now_ms;
                    p->state = S3_PROBE_ALIGN;
                }
            }
            else
            {
                p->candidate = 0u;
            }
        break;

        case S3_PROBE_ALIGN:
            travel = p_in->dist_mm - p->mark_mm;
            if (p_in->left < S3_PROBE_LOSS)
            {
                stop_with(p, S3_PROBE_ALIGN_LOST, p_out);
            }
            else if ((travel < 0) || (travel > ALIGN_MAX_MM))
            {
                stop_with(p, S3_PROBE_DRIVE_LIMIT, p_out);
            }
            else if (travel >= S3_PROBE_ALIGN_MM)
            {
                p->state = S3_PROBE_TURN_BRAKE;
                p->since_ms = p_in->now_ms;
                p_out->action = S3_PROBE_STOP;
            }
        break;

        case S3_PROBE_TURN_BRAKE:
            if (p_in->b_idle && ((p_in->now_ms - p->since_ms) >= BRAKE_MS))
            {
                p->done = p_in->done;
                p->state = S3_PROBE_TURN;
                p->since_ms = p_in->now_ms;
                p_out->action = S3_PROBE_ROTATE;
                p_out->angle_deg = 90;
            }
        break;

        case S3_PROBE_TURN:
            if (p_in->done != p->done)
            {
                p->heading_cdeg = p_in->heading_cdeg;
                p->search_step = 0u;
                brake_begin(p, p_in, true, p_out);
            }
        break;

        case S3_PROBE_BRAKE:
            if (p_in->b_idle && ((p_in->now_ms - p->since_ms) >= BRAKE_MS))
            {
                if ((0u != p->verify) && (p_in->left >= S3_PROBE_BLACK))
                {
                    if (++p->found >= FOUND_SAMPLES)
                    {
                        p->state = S3_PROBE_FOLLOW;
                        p->eligible = 0u;
                        p->clear = 0u;
                        p_out->action = S3_PROBE_DRIVE;
                    }
                }
                else
                {
                    p->found = 0u;
                    search_next(p, p_in, p_out);
                }
            }
        break;

        case S3_PROBE_SEARCH:
            angle = p_in->heading_cdeg - p->heading_cdeg;
            if (angle > 18000)
            {
                angle -= 36000;
            }
            else if (angle < -18000)
            {
                angle += 36000;
            }
            if ((angle > SEARCH_MAX_CDEG) || (angle < -SEARCH_MAX_CDEG))
            {
                stop_with(p, S3_PROBE_DRIVE_LIMIT, p_out);
            }
            else
            {
                p->found = (p_in->left >= S3_PROBE_BLACK) ?
                           (uint8_t) (p->found + 1u) : 0u;
                if (p->found >= FOUND_SAMPLES)
                {
                    brake_begin(p, p_in, true, p_out);
                }
                else if (p_in->done != p->done)
                {
                    brake_begin(p, p_in, false, p_out);
                }
            }
        break;

        default:
            stop_with(p, S3_PROBE_NOT_FOUND, p_out);
        break;
    }
}

static void
stop_with (s3_probe_t * p, s3_probe_reason_t reason,
            s3_probe_output_t * p_out)
{
    p->state = (S3_PROBE_OK == reason) ? S3_PROBE_STOPPED : S3_PROBE_FAULT;
    p->reason = reason;
    p->armed = 0u;
    p_out->action = S3_PROBE_STOP;
}

static void
brake_begin (s3_probe_t * p, s3_probe_input_t const * p_in,
              bool b_verify, s3_probe_output_t * p_out)
{
    p->state = S3_PROBE_BRAKE;
    p->since_ms = p_in->now_ms;
    p->verify = b_verify ? 1u : 0u;
    p->found = 0u;
    p->candidate = 0u;
    p_out->action = S3_PROBE_STOP;
}

static void
search_next (s3_probe_t * p, s3_probe_input_t const * p_in,
              s3_probe_output_t * p_out)
{
    if (p->search_step >= SEARCH_STEPS)
    {
        stop_with(p, S3_PROBE_NOT_FOUND, p_out);
    }
    else
    {
        p->state = S3_PROBE_SEARCH;
        p->since_ms = p_in->now_ms;
        p->done = p_in->done;
        p->found = 0u;
        p_out->action = S3_PROBE_ROTATE;
        p_out->angle_deg = g_search_deg[p->search_step++];
    }
}
