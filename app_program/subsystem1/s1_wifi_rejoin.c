/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_wifi_rejoin.c
 *
 * @brief S1's WiFi rejoin policy (see s1_wifi_rejoin.h). No calls to the
 *        kernel, the radio driver or lwIP.
 */

#include "s1_wifi_rejoin.h"

#if TM_MQTT

#include <stddef.h>

/* The back-off ladder, indexed by the attempt number clamped to the last
 * entry (s1_wifi_rejoin_wait_msec()).
 */
static const uint32_t g_wait_msec[] =
{
    S1_WIFI_REJOIN_WAIT_1_MSEC,
    S1_WIFI_REJOIN_WAIT_2_MSEC,
    S1_WIFI_REJOIN_WAIT_CAP_MSEC
};

/* The table has exactly S1_WIFI_REJOIN_WAIT_STEPS entries, so the clamped
 * index always stays inside it.
 */
typedef uint8_t s1_wifi_rejoin_check_wait_t[
    ((sizeof(g_wait_msec) / sizeof(g_wait_msec[0]))
     == S1_WIFI_REJOIN_WAIT_STEPS) ? 1 : -1];

static bool elapsed_at_least(uint32_t now_msec, uint32_t since_msec,
                             uint32_t limit_msec);
static void no_change(const s1_wifi_rejoin_t * p_rejoin,
                      s1_wifi_rejoin_change_t * p_change);
static void enter(s1_wifi_rejoin_t * p_rejoin, s1_wifi_rejoin_state_t next,
                  s1_wifi_rejoin_why_t why, uint32_t now_msec,
                  s1_wifi_rejoin_change_t * p_change);
static void fail_attempt(s1_wifi_rejoin_t * p_rejoin, s1_wifi_rejoin_why_t why,
                         uint32_t now_msec,
                         s1_wifi_rejoin_change_t * p_change);
static s1_wifi_rejoin_action_t start_join(s1_wifi_rejoin_t * p_rejoin,
                                          s1_wifi_rejoin_why_t why,
                                          uint32_t now_msec,
                                          s1_wifi_rejoin_change_t * p_change);
static void step_up(s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
                    int32_t link_status, s1_wifi_rejoin_change_t * p_change);
static void step_lost(s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
                      int32_t link_status,
                      s1_wifi_rejoin_change_t * p_change);
static s1_wifi_rejoin_action_t step_backoff(s1_wifi_rejoin_t * p_rejoin,
                                            uint32_t now_msec,
                                            int32_t link_status,
                                            s1_wifi_rejoin_change_t *
                                                p_change);
static void step_joining(s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
                         int32_t link_status,
                         s1_wifi_rejoin_change_t * p_change);
static s1_wifi_rejoin_action_t step_leaving(s1_wifi_rejoin_t * p_rejoin,
                                            uint32_t now_msec,
                                            int32_t link_status,
                                            s1_wifi_rejoin_change_t *
                                                p_change);

/*!
 * @brief Start the policy, just after the boot join was requested.
 *
 * The boot join is the first JOINING: it gets the same time limit as any
 * other join, and a failure is retried on the ladder.
 *
 * @param[out] p_rejoin         The policy's state.
 * @param[in]  now_msec      The millisecond clock (it may wrap).
 * @param[in]  b_leave_first True to leave before every rejoin (LEAVING).
 */
void
s1_wifi_rejoin_init (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
                     bool b_leave_first)
{
    if (NULL != p_rejoin)
    {
        p_rejoin->state         = S1_WIFI_REJOIN_JOINING;
        p_rejoin->since_msec    = now_msec;
        p_rejoin->wait_msec     = 0U;
        p_rejoin->attempt       = 0U;
        p_rejoin->last_status   = S1_WIFI_REJOIN_LINK_DOWN;
        p_rejoin->b_stable      = false;
        p_rejoin->b_leave_first = b_leave_first;
    }
}

/*!
 * @brief Advance the policy by one pass of the radio service task.
 *
 * @param[in,out] p_rejoin       The policy's state.
 * @param[in]     now_msec    The millisecond clock (it may wrap).
 * @param[in]     link_status The status just read (CYW43_LINK_xxx).
 * @param[out]    p_change    Receives what changed; b_changed is false if
 *                            the state did not change.
 *
 * @return The action to carry out now: a join, a leave, or nothing. Always
 *         nothing if an argument is NULL.
 */
s1_wifi_rejoin_action_t
s1_wifi_rejoin_step (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
                     int32_t link_status, s1_wifi_rejoin_change_t * p_change)
{
    s1_wifi_rejoin_action_t action = S1_WIFI_REJOIN_DO_NONE;

    if ((NULL != p_rejoin) && (NULL != p_change))
    {
        p_rejoin->last_status = link_status;
        no_change(p_rejoin, p_change);

        switch (p_rejoin->state)
        {
            case S1_WIFI_REJOIN_UP:
                step_up(p_rejoin, now_msec, link_status, p_change);
            break;

            case S1_WIFI_REJOIN_LOST:
                step_lost(p_rejoin, now_msec, link_status, p_change);
            break;

            case S1_WIFI_REJOIN_BACKOFF:
                action = step_backoff(p_rejoin, now_msec, link_status,
                                      p_change);
            break;

            case S1_WIFI_REJOIN_JOINING:
                step_joining(p_rejoin, now_msec, link_status, p_change);
            break;

            case S1_WIFI_REJOIN_LEAVING:
                action = step_leaving(p_rejoin, now_msec, link_status,
                                      p_change);
            break;

            default:
                /* Only a corrupted state gets here: start the ladder
                 * again from BACKOFF, which always ends in a join.
                 */
                enter(p_rejoin, S1_WIFI_REJOIN_BACKOFF,
                      S1_WIFI_REJOIN_WHY_BAD_STATE, now_msec, p_change);
            break;
        }
    }

    return (action);
}

/*!
 * @brief Report that the driver refused the join request just made.
 *
 * Call it in the same pass as the s1_wifi_rejoin_step() that returned
 * S1_WIFI_REJOIN_DO_JOIN. The attempt counts as failed at once, so the
 * policy goes to BACKOFF without waiting for the join limit. It does
 * nothing in any state other than JOINING.
 *
 * @param[in,out] p_rejoin    The policy's state.
 * @param[in]     now_msec The millisecond clock (it may wrap).
 * @param[out]    p_change Receives what changed.
 */
void
s1_wifi_rejoin_refused (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
                        s1_wifi_rejoin_change_t * p_change)
{
    if ((NULL != p_rejoin) && (NULL != p_change))
    {
        no_change(p_rejoin, p_change);

        if (S1_WIFI_REJOIN_JOINING == p_rejoin->state)
        {
            fail_attempt(p_rejoin, S1_WIFI_REJOIN_WHY_REFUSED, now_msec,
                         p_change);
        }
    }
}

/*!
 * @brief The back-off wait before a join attempt.
 *
 * @param[in] attempt The attempt number (failed joins so far).
 *
 * @return 1000, 2000, then 4000 ms for every later attempt.
 */
uint32_t
s1_wifi_rejoin_wait_msec (uint32_t attempt)
{
    const uint32_t last  = S1_WIFI_REJOIN_WAIT_STEPS - 1U;
    const uint32_t index = (attempt < last) ? attempt : last;

    return (g_wait_msec[index]);
}

/*!
 * @brief The name of a state, for the console.
 *
 * @param[in] state A state.
 *
 * @return A short upper-case name, or "?" for a value that is not a state.
 */
const char *
s1_wifi_rejoin_state_name (s1_wifi_rejoin_state_t state)
{
    const char * p_name = "?";

    switch (state)
    {
        case S1_WIFI_REJOIN_UP:
            p_name = "UP";
        break;

        case S1_WIFI_REJOIN_LOST:
            p_name = "LOST";
        break;

        case S1_WIFI_REJOIN_BACKOFF:
            p_name = "BACKOFF";
        break;

        case S1_WIFI_REJOIN_JOINING:
            p_name = "JOINING";
        break;

        case S1_WIFI_REJOIN_LEAVING:
            p_name = "LEAVING";
        break;

        default:
            /* Not a state: keep "?". */
        break;
    }

    return (p_name);
}

/*!
 * @brief The reason for a state change, for the console.
 *
 * @param[in] why A reason.
 *
 * @return A few lower-case words, or "?" for a value that is not a reason.
 */
const char *
s1_wifi_rejoin_why_text (s1_wifi_rejoin_why_t why)
{
    const char * p_text = "?";

    switch (why)
    {
        case S1_WIFI_REJOIN_WHY_NONE:
            p_text = "no change";
        break;

        case S1_WIFI_REJOIN_WHY_LINK_UP:
            p_text = "link up";
        break;

        case S1_WIFI_REJOIN_WHY_LINK_LOST:
            p_text = "link lost";
        break;

        case S1_WIFI_REJOIN_WHY_NEGATIVE:
            p_text = "negative status";
        break;

        case S1_WIFI_REJOIN_WHY_GRACE_OVER:
            p_text = "grace over";
        break;

        case S1_WIFI_REJOIN_WHY_WAIT_OVER:
            p_text = "wait over";
        break;

        case S1_WIFI_REJOIN_WHY_JOIN_LIMIT:
            p_text = "join limit";
        break;

        case S1_WIFI_REJOIN_WHY_REFUSED:
            p_text = "join request refused";
        break;

        case S1_WIFI_REJOIN_WHY_LEFT:
            p_text = "left";
        break;

        case S1_WIFI_REJOIN_WHY_LEAVE_LIMIT:
            p_text = "leave limit";
        break;

        case S1_WIFI_REJOIN_WHY_BAD_STATE:
            p_text = "bad state";
        break;

        default:
            /* Not a reason: keep "?". */
        break;
    }

    return (p_text);
}

/*!
 * @brief Report whether at least limit_msec have passed since since_msec.
 *
 * Unsigned subtraction is modulo 2^32, so the difference is right across
 * the wrap of the 32-bit millisecond clock, for any interval shorter than
 * about 49.7 days.
 */
static bool
elapsed_at_least (uint32_t now_msec, uint32_t since_msec,
                  uint32_t limit_msec)
{
    return ((now_msec - since_msec) >= limit_msec);
}

/*!
 * @brief Fill p_change for a pass that changes nothing.
 */
static void
no_change (const s1_wifi_rejoin_t * p_rejoin,
           s1_wifi_rejoin_change_t * p_change)
{
    p_change->b_changed   = false;
    p_change->from        = p_rejoin->state;
    p_change->to          = p_rejoin->state;
    p_change->why         = S1_WIFI_REJOIN_WHY_NONE;
    p_change->attempt     = p_rejoin->attempt;
    p_change->wait_msec   = p_rejoin->wait_msec;
    p_change->link_status = p_rejoin->last_status;
}

/*!
 * @brief Enter a state and record the change.
 *
 * Entering UP restarts the stability timer; entering BACKOFF picks the
 * wait for the current attempt number.
 */
static void
enter (s1_wifi_rejoin_t * p_rejoin, s1_wifi_rejoin_state_t next,
       s1_wifi_rejoin_why_t why, uint32_t now_msec,
       s1_wifi_rejoin_change_t * p_change)
{
    p_change->b_changed = true;
    p_change->from      = p_rejoin->state;
    p_change->to        = next;
    p_change->why       = why;

    p_rejoin->state      = next;
    p_rejoin->since_msec = now_msec;
    p_rejoin->wait_msec  = 0U;

    if (S1_WIFI_REJOIN_UP == next)
    {
        p_rejoin->b_stable = false;
    }
    else if (S1_WIFI_REJOIN_BACKOFF == next)
    {
        p_rejoin->wait_msec = s1_wifi_rejoin_wait_msec(p_rejoin->attempt);
    }
    else
    {
        /* LOST, JOINING and LEAVING need only the entry time. */
    }

    p_change->attempt   = p_rejoin->attempt;
    p_change->wait_msec = p_rejoin->wait_msec;
}

/*!
 * @brief Count a failed join attempt and go to BACKOFF.
 */
static void
fail_attempt (s1_wifi_rejoin_t * p_rejoin, s1_wifi_rejoin_why_t why,
              uint32_t now_msec, s1_wifi_rejoin_change_t * p_change)
{
    if (p_rejoin->attempt < S1_WIFI_REJOIN_ATTEMPT_MAX)
    {
        p_rejoin->attempt++;
    }

    enter(p_rejoin, S1_WIFI_REJOIN_BACKOFF, why, now_msec, p_change);
}

/*!
 * @brief Enter JOINING; the caller requests the join in the same pass.
 *
 * @return S1_WIFI_REJOIN_DO_JOIN.
 */
static s1_wifi_rejoin_action_t
start_join (s1_wifi_rejoin_t * p_rejoin, s1_wifi_rejoin_why_t why,
            uint32_t now_msec, s1_wifi_rejoin_change_t * p_change)
{
    enter(p_rejoin, S1_WIFI_REJOIN_JOINING, why, now_msec, p_change);

    return (S1_WIFI_REJOIN_DO_JOIN);
}

/*!
 * @brief UP: watch for a loss; reset the counter once the link is stable.
 */
static void
step_up (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec, int32_t link_status,
         s1_wifi_rejoin_change_t * p_change)
{
    if (S1_WIFI_REJOIN_LINK_UP != link_status)
    {
        enter(p_rejoin, S1_WIFI_REJOIN_LOST, S1_WIFI_REJOIN_WHY_LINK_LOST,
              now_msec, p_change);
    }
    else if ((!p_rejoin->b_stable)
             && (elapsed_at_least(now_msec, p_rejoin->since_msec,
                                  S1_WIFI_REJOIN_STABLE_MSEC)))
    {
        p_rejoin->attempt  = 0U;
        p_rejoin->b_stable = true;
    }
    else
    {
        /* Up, and nothing to do. */
    }
}

/*!
 * @brief LOST: give the chip its grace to recover alone.
 */
static void
step_lost (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec, int32_t link_status,
           s1_wifi_rejoin_change_t * p_change)
{
    if (S1_WIFI_REJOIN_LINK_UP == link_status)
    {
        enter(p_rejoin, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_WHY_LINK_UP, now_msec,
              p_change);
    }
    else if (link_status < 0)
    {
        /* Negative: the driver has declared the association failed. */
        enter(p_rejoin, S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_NEGATIVE,
              now_msec, p_change);
    }
    else if (elapsed_at_least(now_msec, p_rejoin->since_msec,
                              S1_WIFI_REJOIN_GRACE_MSEC))
    {
        enter(p_rejoin, S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_GRACE_OVER,
              now_msec, p_change);
    }
    else
    {
        /* Still within the grace. */
    }
}

/*!
 * @brief BACKOFF: wait, then join (or leave first).
 *
 * @return The action that starts the next attempt, or nothing.
 */
static s1_wifi_rejoin_action_t
step_backoff (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
              int32_t link_status, s1_wifi_rejoin_change_t * p_change)
{
    s1_wifi_rejoin_action_t action = S1_WIFI_REJOIN_DO_NONE;

    if (S1_WIFI_REJOIN_LINK_UP == link_status)
    {
        /* The chip won the race. */
        enter(p_rejoin, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_WHY_LINK_UP, now_msec,
              p_change);
    }
    else if (!elapsed_at_least(now_msec, p_rejoin->since_msec,
                               p_rejoin->wait_msec))
    {
        /* Still waiting. */
    }
    else if (p_rejoin->b_leave_first)
    {
        enter(p_rejoin, S1_WIFI_REJOIN_LEAVING, S1_WIFI_REJOIN_WHY_WAIT_OVER,
              now_msec, p_change);
        action = S1_WIFI_REJOIN_DO_LEAVE;
    }
    else
    {
        action = start_join(p_rejoin, S1_WIFI_REJOIN_WHY_WAIT_OVER, now_msec,
                            p_change);
    }

    return (action);
}

/*!
 * @brief JOINING: wait for the link, within the join limit.
 */
static void
step_joining (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
              int32_t link_status, s1_wifi_rejoin_change_t * p_change)
{
    if (S1_WIFI_REJOIN_LINK_UP == link_status)
    {
        enter(p_rejoin, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_WHY_LINK_UP, now_msec,
              p_change);
    }
    else if (link_status < 0)
    {
        /* -3 included: it is not final, the ladder tries again. */
        fail_attempt(p_rejoin, S1_WIFI_REJOIN_WHY_NEGATIVE, now_msec, p_change);
    }
    else if (elapsed_at_least(now_msec, p_rejoin->since_msec,
                              S1_WIFI_REJOIN_JOIN_LIMIT_MSEC))
    {
        /* Status 0, 1 or 2 for the whole limit: no progress. */
        fail_attempt(p_rejoin, S1_WIFI_REJOIN_WHY_JOIN_LIMIT, now_msec,
                     p_change);
    }
    else
    {
        /* Status 0, 1 or 2 within the limit: keep waiting. */
    }
}

/*!
 * @brief LEAVING: wait for status 0, then join; join anyway at the limit.
 *
 * NOTE: Status 3 is not taken as UP here: the leave request is in flight,
 * so the link is about to go down. The limit, then JOINING, deal with it.
 *
 * @return S1_WIFI_REJOIN_DO_JOIN when the join is due, else nothing.
 */
static s1_wifi_rejoin_action_t
step_leaving (s1_wifi_rejoin_t * p_rejoin, uint32_t now_msec,
              int32_t link_status, s1_wifi_rejoin_change_t * p_change)
{
    s1_wifi_rejoin_action_t action = S1_WIFI_REJOIN_DO_NONE;

    if (S1_WIFI_REJOIN_LINK_DOWN == link_status)
    {
        action = start_join(p_rejoin, S1_WIFI_REJOIN_WHY_LEFT, now_msec,
                            p_change);
    }
    else if (elapsed_at_least(now_msec, p_rejoin->since_msec,
                              S1_WIFI_REJOIN_LEAVE_LIMIT_MSEC))
    {
        action = start_join(p_rejoin, S1_WIFI_REJOIN_WHY_LEAVE_LIMIT, now_msec,
                            p_change);
    }
    else
    {
        /* Waiting for the disassociation event. */
    }

    return (action);
}

#endif /* TM_MQTT */

/*** end of file ***/

