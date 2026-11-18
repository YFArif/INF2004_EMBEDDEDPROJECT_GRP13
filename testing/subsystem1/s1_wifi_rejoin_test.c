/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_wifi_rejoin_test.c
 *
 * @brief S1-U28: host test of S1's WiFi rejoin policy (s1_wifi_rejoin.c).
 *
 * @par What it tests
 * app_program/subsystem1/s1_wifi_rejoin.c through its public API:
 * - the boot join: success; -3, then success; stuck at status 1 for ever;
 * - a drop that keeps status 1 past the grace; a chip that recovers alone
 *   inside the grace (no join); a negative status inside the grace;
 * - -3 then 1 inside one attempt (the retry is not cancelled);
 * - status 0 for ever, past 32 attempts; status 2 stuck, after a join and
 *   after UP;
 * - the 1, 2, 4, 4 s ladder, its cap and the saturating counter;
 * - the counter reset only after 10 s of continuous UP;
 * - no action while UP, including a Mac-only outage (status 3 throughout);
 * - both leave settings: a plain join, and LEAVING with status 0, with its
 *   1 s limit and with a status of 3 that must not count as UP;
 * - the setting the build was configured with (S1_WIFI_LEAVE_BEFORE_JOIN);
 * - a join request the driver refuses (a failed attempt at once);
 * - the 32-bit clock wrap in JOINING and in LOST;
 * - NULL arguments, an invalid stored state, and the console names.
 *
 * @par How
 * A harness calls s1_wifi_rejoin_step() once per pass of cyw43_utk_task(),
 * every 20 ms of fake time (PASS_MSEC), with a scripted link status,
 * and records every state change and every action with its time. When
 * asked to, it plays a driver that refuses every join request. Expected
 * times are written out by hand from the timings in s1_wifi_rejoin.h
 * (waits 1, 2 and 4 s, grace 3 s, join limit 10 s, leave limit 1 s, stable
 * UP 10 s), not computed with the module's constants.
 *
 * @par Build and run
 * Use testing/subsystem1/run_wifi_rejoin_test.sh from the repository root.
 * This program prints one line per failed check and one line per group,
 * then "RESULT: all passed (n checks)" and exits 0, or
 * "RESULT: m FAILED (n checks)" and exits 1.
 *
 * @par Test code
 * This module is never built into the firmware. int main(void) is the
 * form the C standard requires (Barr-C 5.2.a does not apply to it).
 */

#include "s1_wifi_rejoin_test.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "s1_wifi_rejoin.h"

/* Link status values (CYW43_LINK_xxx in cyw43.h), written out by hand. */
#define STATUS_BADAUTH          (-3)
#define STATUS_NONET            (-2)
#define STATUS_DOWN             0
#define STATUS_JOIN             1
#define STATUS_NOIP             2
#define STATUS_UP               3

/* The radio task's pass period. It asks tk_dly_tsk() for CYW43_POLL_MS
 * (2 ms), but the kernel tick is 10 ms (CNF_TIMER_PERIOD) and a delay ends
 * on the first tick at least the delay plus one tick after the last tick
 * (knl_timer_insert(), kernel/tkernel/timer.c), so a pass starts about
 * every 20 ms. Every limit in s1_wifi_rejoin.h is a multiple of it, so the
 * expected times are exact.
 */
#define PASS_MSEC           20U

/* Room for every change and action of the longest group. */
#define RECORDS_MAX         512U

/* An arbitrary start time, and one 5 s before the 32-bit clock wraps. */
#define START_MSEC             100000U
#define WRAP_START_MSEC         (UINT32_MAX - 4999U)

/* Status 0 for ever must run past this many attempts. */
#define LONG_RUN_ATTEMPTS   40U

typedef struct
{
    uint32_t                when_msec;
    s1_wifi_rejoin_change_t change;
} change_rec_t;

typedef struct
{
    uint32_t                when_msec;
    s1_wifi_rejoin_action_t action;
} action_rec_t;

typedef struct
{
    s1_wifi_rejoin_t rejoin;
    uint32_t         now_msec;
    bool             b_refuse_join;
    bool             b_overflow;
    uint32_t         change_count;
    uint32_t         action_count;
    change_rec_t     changes[RECORDS_MAX];
    action_rec_t     actions[RECORDS_MAX];
} harness_t;

static harness_t    g_harness;
static uint32_t     g_checks;
static uint32_t     g_failed;
static uint32_t     g_group_checks;
static uint32_t     g_group_failed;
static const char * gp_group = "";

static void check(bool b_ok, const char * p_what, uint32_t detail);
static void group_begin(const char * p_name);
static void group_end(void);
static void start(harness_t * p_harness, uint32_t start_msec,
                  bool b_leave_first);
static void keep(harness_t * p_harness,
                 const s1_wifi_rejoin_change_t * p_change);
static void pass(harness_t * p_harness, int32_t link_status);
static void drive(harness_t * p_harness, int32_t link_status,
                  uint32_t duration_msec);
static void go_up_stable(harness_t * p_harness);
static uint32_t count_actions(const harness_t * p_harness, uint32_t first,
                              s1_wifi_rejoin_action_t action);
static void expect_change(const harness_t * p_harness, uint32_t index,
                          s1_wifi_rejoin_state_t from_state,
                          s1_wifi_rejoin_state_t to_state,
                          s1_wifi_rejoin_why_t why, uint32_t when_msec,
                          const char * p_what);
static void expect_backoff(const harness_t * p_harness, uint32_t index,
                           uint32_t attempt, uint32_t wait_msec,
                           const char * p_what);
static void expect_action(const harness_t * p_harness, uint32_t index,
                          s1_wifi_rejoin_action_t action, uint32_t when_msec,
                          const char * p_what);

static void test_boot_join_ok(void);
static void test_boot_join_badauth(void);
static void test_boot_join_stuck(void);
static void test_drop_past_grace(void);
static void test_recover_in_grace(void);
static void test_negative_in_grace(void);
static void test_badauth_then_join(void);
static void test_status0_forever(void);
static void test_status2_stuck(void);
static void test_ladder(void);
static void test_counter_reset(void);
static void test_no_action_while_up(void);
static void test_leave_off(void);
static void test_leave_left(void);
static void test_leave_limit(void);
static void test_leave_up_is_not_up(void);
static void test_configured_setting(void);
static void test_refused(void);
static void test_wrap(void);
static void test_null_and_names(void);
static void test_names(void);

/*!
 * @brief Count one check; print it if it failed.
 *
 * NOTE: Callers pass a state as the detail with a (uint32_t) cast. It is
 * safe: every enumerator is small and not negative.
 */
static void
check (bool b_ok, const char * p_what, uint32_t detail)
{
    g_checks++;
    g_group_checks++;

    if (!b_ok)
    {
        g_failed++;
        g_group_failed++;
        printf("FAIL %s: %s (got %" PRIu32 ")\n", gp_group, p_what, detail);
    }
}

/*!
 * @brief Start a group of checks.
 */
static void
group_begin (const char * p_name)
{
    gp_group       = p_name;
    g_group_checks = 0U;
    g_group_failed = 0U;
}

/*!
 * @brief Print the group's totals.
 */
static void
group_end (void)
{
    printf("group %s: %" PRIu32 " checks, %" PRIu32 " failed\n", gp_group,
           g_group_checks, g_group_failed);
}

/*!
 * @brief Clear the harness and start the policy at t0 (the boot join).
 */
static void
start (harness_t * p_harness, uint32_t start_msec, bool b_leave_first)
{
    p_harness->now_msec      = start_msec;
    p_harness->b_refuse_join = false;
    p_harness->b_overflow    = false;
    p_harness->change_count  = 0U;
    p_harness->action_count  = 0U;
    s1_wifi_rejoin_init(&p_harness->rejoin, start_msec, b_leave_first);
}

/*!
 * @brief Record a change, if there was one.
 */
static void
keep (harness_t * p_harness, const s1_wifi_rejoin_change_t * p_change)
{
    if (p_change->b_changed)
    {
        if (p_harness->change_count < RECORDS_MAX)
        {
            p_harness->changes[p_harness->change_count].when_msec =
                p_harness->now_msec;
            p_harness->changes[p_harness->change_count].change = *p_change;
            p_harness->change_count++;
        }
        else
        {
            p_harness->b_overflow = true;
        }
    }
}

/*!
 * @brief One pass of the radio task at the current time, then one pass
 *        period on.
 */
static void
pass (harness_t * p_harness, int32_t link_status)
{
    s1_wifi_rejoin_change_t change;
    s1_wifi_rejoin_action_t action;

    action = s1_wifi_rejoin_step(&p_harness->rejoin, p_harness->now_msec,
                                 link_status, &change);
    keep(p_harness, &change);

    if (S1_WIFI_REJOIN_DO_NONE != action)
    {
        if (p_harness->action_count < RECORDS_MAX)
        {
            p_harness->actions[p_harness->action_count].when_msec =
                p_harness->now_msec;
            p_harness->actions[p_harness->action_count].action = action;
            p_harness->action_count++;
        }
        else
        {
            p_harness->b_overflow = true;
        }
    }

    if ((S1_WIFI_REJOIN_DO_JOIN == action) && (p_harness->b_refuse_join))
    {
        s1_wifi_rejoin_refused(&p_harness->rejoin, p_harness->now_msec,
                               &change);
        keep(p_harness, &change);
    }

    p_harness->now_msec += PASS_MSEC;
}

/*!
 * @brief Passes with one status, at now, now + PASS_MSEC, ... up to but
 *        not including now + duration_msec, which becomes the new time.
 */
static void
drive (harness_t * p_harness, int32_t link_status, uint32_t duration_msec)
{
    uint32_t elapsed_msec;

    for (elapsed_msec = 0U; elapsed_msec < duration_msec;
         elapsed_msec += PASS_MSEC)
    {
        pass(p_harness, link_status);
    }
}

/*!
 * @brief A good boot join (3.3 s), then 12 s of UP: stable, counter 0.
 */
static void
go_up_stable (harness_t * p_harness)
{
    drive(p_harness, STATUS_JOIN, 3300U);
    drive(p_harness, STATUS_UP, 12000U);
}

/*!
 * @brief Count the recorded actions of one kind from index first on.
 */
static uint32_t
count_actions (const harness_t * p_harness, uint32_t first,
               s1_wifi_rejoin_action_t action)
{
    uint32_t count = 0U;
    uint32_t index;

    for (index = first; index < p_harness->action_count; index++)
    {
        if (action == p_harness->actions[index].action)
        {
            count++;
        }
    }

    return (count);
}

/*!
 * @brief Check one recorded change: from, to, reason and time.
 */
static void
expect_change (const harness_t * p_harness, uint32_t index,
               s1_wifi_rejoin_state_t from_state,
               s1_wifi_rejoin_state_t to_state,
               s1_wifi_rejoin_why_t why, uint32_t when_msec,
               const char * p_what)
{
    const change_rec_t * p_rec;
    bool                 b_ok = (index < p_harness->change_count);

    if (b_ok)
    {
        p_rec = &p_harness->changes[index];
        b_ok  = ((from_state == p_rec->change.from)
                 && (to_state == p_rec->change.to)
                 && (why == p_rec->change.why)
                 && (when_msec == p_rec->when_msec));

        if (!b_ok)
        {
            printf("     change %" PRIu32 ": %s -> %s (%s) at %" PRIu32
                   ", expected %s -> %s (%s) at %" PRIu32 "\n", index,
                   s1_wifi_rejoin_state_name(p_rec->change.from),
                   s1_wifi_rejoin_state_name(p_rec->change.to),
                   s1_wifi_rejoin_why_text(p_rec->change.why), p_rec->when_msec,
                   s1_wifi_rejoin_state_name(from_state),
                   s1_wifi_rejoin_state_name(to_state),
                   s1_wifi_rejoin_why_text(why), when_msec);
        }
    }

    check(b_ok, p_what, p_harness->change_count);
}

/*!
 * @brief Check the attempt number and the wait of one recorded change.
 */
static void
expect_backoff (const harness_t * p_harness, uint32_t index,
                uint32_t attempt, uint32_t wait_msec, const char * p_what)
{
    bool b_ok = (index < p_harness->change_count);

    if (b_ok)
    {
        b_ok = ((attempt == p_harness->changes[index].change.attempt)
                && (wait_msec == p_harness->changes[index].change.wait_msec));

        if (!b_ok)
        {
            printf("     change %" PRIu32 ": attempt %" PRIu32 " wait %"
                   PRIu32 ", expected attempt %" PRIu32 " wait %" PRIu32
                   "\n", index, p_harness->changes[index].change.attempt,
                   p_harness->changes[index].change.wait_msec, attempt,
                   wait_msec);
        }
    }

    check(b_ok, p_what, p_harness->change_count);
}

/*!
 * @brief Check one recorded action and its time.
 */
static void
expect_action (const harness_t * p_harness, uint32_t index,
               s1_wifi_rejoin_action_t action, uint32_t when_msec,
               const char * p_what)
{
    bool b_ok = (index < p_harness->action_count);

    if (b_ok)
    {
        b_ok = ((action == p_harness->actions[index].action)
                && (when_msec == p_harness->actions[index].when_msec));

        if (!b_ok)
        {
            printf("     action %" PRIu32 ": %d at %" PRIu32
                   ", expected %d at %" PRIu32 "\n", index,
                   (int)p_harness->actions[index].action,
                   p_harness->actions[index].when_msec, (int)action, when_msec);
        }
    }

    check(b_ok, p_what, p_harness->action_count);
}

/* ------------------------------------------------------------- boot */

/*!
 * @brief The boot join succeeds: JOINING to UP, no action.
 */
static void
test_boot_join_ok (void)
{
    harness_t * p_harness = &g_harness;

    group_begin("boot join succeeds");
    start(p_harness, START_MSEC, false);
    check(S1_WIFI_REJOIN_JOINING == p_harness->rejoin.state,
          "the policy starts in JOINING", (uint32_t)p_harness->rejoin.state);
    drive(p_harness, STATUS_JOIN, 3300U);
    drive(p_harness, STATUS_UP, 20000U);
    expect_change(p_harness, 0U, S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_UP,
                  S1_WIFI_REJOIN_WHY_LINK_UP, START_MSEC + 3300U,
                  "JOINING to UP at the first status 3");
    check(1U == p_harness->change_count, "no other change",
          p_harness->change_count);
    check(0U == p_harness->action_count, "no action (the boot join is "
          "cyw43_utk_task's own)", p_harness->action_count);
    check(0U == p_harness->rejoin.attempt, "attempt 0",
          p_harness->rejoin.attempt);
    group_end();
}

/*!
 * @brief The boot join gives -3, then the retry succeeds.
 */
static void
test_boot_join_badauth (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    negative_msec;

    group_begin("boot join gives -3, then succeeds");
    start(p_harness, START_MSEC, false);
    drive(p_harness, STATUS_JOIN, 400U);
    negative_msec = p_harness->now_msec;
    drive(p_harness, STATUS_BADAUTH, 2000U);
    expect_change(p_harness, 0U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_NEGATIVE,
                  negative_msec,
                  "-3 ends the attempt at once");
    expect_backoff(p_harness, 0U, 1U, 2000U, "attempt 1, wait 2 s");
    check(0U == p_harness->action_count, "no join during the wait",
          p_harness->action_count);
    drive(p_harness, STATUS_BADAUTH, PASS_MSEC);
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, negative_msec + 2000U,
                  "join 2 s after the -3, with the status still -3");
    expect_change(p_harness, 1U, S1_WIFI_REJOIN_BACKOFF,
                  S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_WHY_WAIT_OVER,
                  negative_msec + 2000U, "BACKOFF to JOINING");
    drive(p_harness, STATUS_JOIN, 3500U);
    drive(p_harness, STATUS_UP, PASS_MSEC);
    expect_change(p_harness, 2U, S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_UP,
                  S1_WIFI_REJOIN_WHY_LINK_UP,
                  negative_msec + 2000U + PASS_MSEC + 3500U,
                  "UP after the second join");
    check(1U == p_harness->action_count, "one join in all",
          p_harness->action_count);
    group_end();
}

/*!
 * @brief The boot join stays at status 1: the limit and the ladder.
 */
static void
test_boot_join_stuck (void)
{
    harness_t * p_harness = &g_harness;

    group_begin("boot join stuck at status 1 for ever");
    start(p_harness, START_MSEC, false);
    drive(p_harness, STATUS_JOIN, 60000U);
    expect_change(p_harness, 0U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  START_MSEC + 10000U, "the boot join ends at its 10 s limit");
    expect_backoff(p_harness, 0U, 1U, 2000U, "attempt 1, wait 2 s");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 12000U,
                  "first rejoin at 12 s");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 26000U,
                  "second at 26 s (10 s limit, then 4 s)");
    expect_action(p_harness, 2U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 40000U,
                  "third at 40 s");
    expect_action(p_harness, 3U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 54000U,
                  "fourth at 54 s");
    check(4U == p_harness->action_count, "four rejoins in 60 s",
          p_harness->action_count);
    expect_backoff(p_harness, 2U, 2U, 4000U, "attempt 2, wait 4 s");
    expect_backoff(p_harness, 4U, 3U, 4000U, "attempt 3, wait 4 s (cap)");
    group_end();
}

/* ------------------------------------------------------------ drops */

/*!
 * @brief A drop with status 1 past the grace: LOST, BACKOFF, JOINING.
 */
static void
test_drop_past_grace (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    first;

    group_begin("drop: status 1 past the grace");
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_JOIN, 16000U + PASS_MSEC);
    expect_change(p_harness, first, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_WHY_LINK_LOST, drop_msec,
                  "UP to LOST at the first status 1");
    expect_change(p_harness, first + 1U, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_GRACE_OVER,
                  drop_msec + 3000U, "BACKOFF after the 3 s grace");
    expect_backoff(p_harness, first + 1U, 0U, 1000U, "attempt 0, wait 1 s");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, drop_msec + 4000U,
                  "rejoin at 4 s");
    expect_change(p_harness, first + 3U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  drop_msec + 14000U, "that join ends at its 10 s limit");
    expect_backoff(p_harness, first + 3U, 1U, 2000U, "attempt 1, wait 2 s");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, drop_msec + 16000U,
                  "second rejoin at 16 s");
    drive(p_harness, STATUS_JOIN, 3500U);
    drive(p_harness, STATUS_UP, PASS_MSEC);
    check(S1_WIFI_REJOIN_UP == p_harness->rejoin.state, "UP again",
          (uint32_t)p_harness->rejoin.state);
    check(2U == p_harness->action_count, "two joins, no leave",
          p_harness->action_count);
    group_end();
}

/*!
 * @brief The chip recovers inside the grace: no join.
 */
static void
test_recover_in_grace (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    first;

    group_begin("the chip recovers inside the grace");
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_JOIN, 3000U - PASS_MSEC);
    drive(p_harness, STATUS_UP, 20000U);
    expect_change(p_harness, first, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_WHY_LINK_LOST, drop_msec, "UP to LOST");
    expect_change(p_harness, first + 1U, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_WHY_LINK_UP,
                  drop_msec + 3000U - PASS_MSEC,
                  "LOST to UP, one pass before the grace ends");
    check((first + 2U) == p_harness->change_count, "no other change",
          p_harness->change_count);
    check(0U == p_harness->action_count, "no join",
          p_harness->action_count);
    group_end();
}

/*!
 * @brief A negative status ends the grace at once.
 */
static void
test_negative_in_grace (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    negative_msec;
    uint32_t    first;

    group_begin("a negative status inside the grace");
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    first = p_harness->change_count;
    drive(p_harness, STATUS_JOIN, 500U);
    negative_msec = p_harness->now_msec;
    drive(p_harness, STATUS_NONET, 1000U + PASS_MSEC);
    expect_change(p_harness, first + 1U, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_NEGATIVE,
                  negative_msec,
                  "-2 ends the grace at once");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, negative_msec + 1000U,
                  "rejoin 1 s later");
    group_end();
}

/*!
 * @brief -3 then 1 inside one attempt: the retry is kept.
 */
static void
test_badauth_then_join (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    negative_msec;
    uint32_t    first;

    group_begin("-3 then 1 inside one attempt");
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    drive(p_harness, STATUS_JOIN, 4000U + PASS_MSEC);
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN,
                  START_MSEC + 15300U + 4000U, "rejoin at 4 s");
    drive(p_harness, STATUS_JOIN, 500U);
    first = p_harness->change_count;
    negative_msec = p_harness->now_msec;
    drive(p_harness, STATUS_BADAUTH, PASS_MSEC);
    /* The driver sets ACTIVE again after a good auth: status 1. */
    drive(p_harness, STATUS_JOIN, 2000U);
    expect_change(p_harness, first, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_NEGATIVE,
                  negative_msec,
                  "the -3 counts as a failed attempt");
    expect_backoff(p_harness, first, 1U, 2000U, "attempt 1, wait 2 s");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, negative_msec + 2000U,
                  "the 1 that follows does not cancel the retry");
    check((first + 2U) == p_harness->change_count,
          "nothing between the BACKOFF and the join",
          p_harness->change_count);
    group_end();
}

/*!
 * @brief Status 0 for ever: 40 attempts, the cap, no wrap.
 */
static void
test_status0_forever (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    index;
    uint32_t    expected_msec;
    bool        b_times_ok = true;
    bool        b_ladder_ok = true;
    uint32_t    backoffs = 0U;

    group_begin("status 0 for ever, past 32 attempts");
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    drop_msec = p_harness->now_msec;
    /* Joins at 4 s, 16 s and 30 s, then every 14 s: join n (from 0) at
     * 30 s + (n - 2) x 14 s, so the last of 40 is at 30 s + 37 x 14 s.
     */
    drive(p_harness, STATUS_DOWN,
          30000U + ((LONG_RUN_ATTEMPTS - 3U) * 14000U) + PASS_MSEC);
    check(!p_harness->b_overflow, "the records fit", 0U);
    check(LONG_RUN_ATTEMPTS == p_harness->action_count,
          "one join per cycle, 40 in all", p_harness->action_count);
    check(LONG_RUN_ATTEMPTS
          == count_actions(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN),
          "every action is a join", p_harness->action_count);

    for (index = 0U; index < p_harness->action_count; index++)
    {
        if (0U == index)
        {
            expected_msec = drop_msec + 4000U;
        }
        else if (1U == index)
        {
            expected_msec = drop_msec + 16000U;
        }
        else
        {
            expected_msec = drop_msec + 30000U + ((index - 2U) * 14000U);
        }

        b_times_ok = b_times_ok
                     && (expected_msec == p_harness->actions[index].when_msec);
    }

    check(b_times_ok, "joins at 4, 16, 30, 44 ... s", 0U);

    for (index = 0U; index < p_harness->change_count; index++)
    {
        const s1_wifi_rejoin_change_t * p_rec =
            &p_harness->changes[index].change;

        if (S1_WIFI_REJOIN_WHY_JOIN_LIMIT == p_rec->why)
        {
            backoffs++;
            b_ladder_ok = b_ladder_ok && (backoffs == p_rec->attempt)
                          && (((1U == backoffs) ? 2000U : 4000U)
                              == p_rec->wait_msec);
        }
    }

    check(b_ladder_ok, "attempts 1, 2, 3 ... with waits 2, 4, 4 ... s",
          backoffs);
    check(backoffs > 32U, "more than 32 failed attempts",
          backoffs);
    check(backoffs == p_harness->rejoin.attempt, "the counter kept counting",
          p_harness->rejoin.attempt);
    group_end();
}

/*!
 * @brief Status 2 stuck, after a join and after UP.
 */
static void
test_status2_stuck (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    first;

    group_begin("status 2 stuck");
    start(p_harness, START_MSEC, false);
    drive(p_harness, STATUS_NOIP, 10000U + PASS_MSEC);
    expect_change(p_harness, 0U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  START_MSEC + 10000U, "after a join: the 10 s limit applies");

    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_NOIP, 14000U + PASS_MSEC);
    expect_change(p_harness, first, S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_WHY_LINK_LOST, drop_msec,
                  "after UP: 2 is not UP");
    expect_change(p_harness, first + 1U, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_GRACE_OVER,
                  drop_msec + 3000U, "BACKOFF after the grace");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, drop_msec + 4000U,
                  "rejoin at 4 s");
    expect_change(p_harness, first + 3U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  drop_msec + 14000U, "that join ends at its limit");
    group_end();
}

/* ---------------------------------------------------- ladder, counter */

/*!
 * @brief The 1, 2, 4, 4 s ladder and the saturating counter.
 */
static void
test_ladder (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    first;

    group_begin("the 1, 2, 4, 4 ladder and the saturating counter");
    check(1000U == s1_wifi_rejoin_wait_msec(0U), "attempt 0: 1 s",
          s1_wifi_rejoin_wait_msec(0U));
    check(2000U == s1_wifi_rejoin_wait_msec(1U), "attempt 1: 2 s",
          s1_wifi_rejoin_wait_msec(1U));
    check(4000U == s1_wifi_rejoin_wait_msec(2U), "attempt 2: 4 s",
          s1_wifi_rejoin_wait_msec(2U));
    check(4000U == s1_wifi_rejoin_wait_msec(3U), "attempt 3: 4 s (cap)",
          s1_wifi_rejoin_wait_msec(3U));
    check(4000U == s1_wifi_rejoin_wait_msec(33U), "attempt 33: 4 s",
          s1_wifi_rejoin_wait_msec(33U));
    check(4000U == s1_wifi_rejoin_wait_msec(UINT32_MAX),
          "attempt UINT32_MAX: 4 s",
          s1_wifi_rejoin_wait_msec(UINT32_MAX));

    /* A drop with status 1: waits 1, 2, 4, 4 s. */
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    first = p_harness->change_count;
    drive(p_harness, STATUS_JOIN, 3000U + 1000U + 10000U + 2000U + 10000U
          + 4000U + 10000U + PASS_MSEC);
    expect_backoff(p_harness, first + 1U, 0U, 1000U, "wait 1 s");
    expect_backoff(p_harness, first + 3U, 1U, 2000U, "wait 2 s");
    expect_backoff(p_harness, first + 5U, 2U, 4000U, "wait 4 s");
    expect_backoff(p_harness, first + 7U, 3U, 4000U, "wait 4 s again");

    /* The counter stops at its maximum instead of wrapping to 0. The test
     * sets the field directly: it is the only way to reach it in a test.
     */
    start(p_harness, START_MSEC, false);
    p_harness->rejoin.attempt = UINT32_MAX - 1U;
    drive(p_harness, STATUS_JOIN, 10000U + PASS_MSEC);
    expect_backoff(p_harness, 0U, UINT32_MAX, 4000U,
                   "the counter reaches its maximum");
    drive(p_harness, STATUS_JOIN, 4000U + 10000U + PASS_MSEC);
    expect_backoff(p_harness, 2U, UINT32_MAX, 4000U,
                   "and stays there, still with the 4 s wait");
    group_end();
}

/*!
 * @brief The counter is reset only after 10 s of continuous UP.
 */
static void
test_counter_reset (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    first;

    group_begin("counter reset only after 10 s of UP");
    /* Boot join stuck twice: attempt 2. */
    start(p_harness, START_MSEC, false);
    drive(p_harness, STATUS_JOIN, 26000U + PASS_MSEC);
    check(2U == p_harness->rejoin.attempt, "attempt 2 after two failures",
          p_harness->rejoin.attempt);
    drive(p_harness, STATUS_JOIN, 3000U);
    /* UP for one pass less than 10 s: the last UP pass is 9.96 s after
     * the first.
     */
    drive(p_harness, STATUS_UP, 10000U - PASS_MSEC);
    first = p_harness->change_count;
    drive(p_harness, STATUS_JOIN, 3000U + PASS_MSEC);
    expect_backoff(p_harness, first + 1U, 2U, 4000U,
                   "a short UP keeps the counter: wait 4 s");

    /* The chip wins in BACKOFF; UP through the 10 s mark. */
    drive(p_harness, STATUS_UP, 10000U + PASS_MSEC);
    check(0U == p_harness->rejoin.attempt, "reset at 10 s of UP",
          p_harness->rejoin.attempt);
    first = p_harness->change_count;
    drive(p_harness, STATUS_JOIN, 3000U + PASS_MSEC);
    expect_backoff(p_harness, first + 1U, 0U, 1000U,
                   "after a stable UP: wait 1 s");
    group_end();
}

/*!
 * @brief No action while UP, including a Mac-only outage.
 */
static void
test_no_action_while_up (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    first_change;
    uint32_t    first_action;

    group_begin("no action while UP; Mac-only outage");
    start(p_harness, START_MSEC, false);
    drive(p_harness, STATUS_JOIN, 3300U);
    drive(p_harness, STATUS_UP, PASS_MSEC);
    first_change = p_harness->change_count;
    first_action = p_harness->action_count;
    /* A Mac or broker outage right after UP: the status stays 3. */
    drive(p_harness, STATUS_UP, 30000U);
    check(first_action == p_harness->action_count,
          "Mac-only outage of 30 s: no WiFi action",
          p_harness->action_count);
    drive(p_harness, STATUS_UP, 600000U);
    check(first_change == p_harness->change_count,
          "10 more minutes of UP: no change",
          p_harness->change_count);
    check(first_action == p_harness->action_count, "and no action",
          p_harness->action_count);
    group_end();
}

/* ------------------------------------------------------------ leave */

/*!
 * @brief Leave setting 0: plain joins only.
 */
static void
test_leave_off (void)
{
    harness_t * p_harness = &g_harness;

    group_begin("leave setting 0: plain joins only");
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    drive(p_harness, STATUS_JOIN, 60000U);
    check(0U == count_actions(p_harness, 0U, S1_WIFI_REJOIN_DO_LEAVE),
          "no leave in 60 s of drop", 0U);
    check(0U < count_actions(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN),
          "joins only", p_harness->action_count);
    group_end();
}

/*!
 * @brief Leave setting 1: leave, status 0, then join.
 */
static void
test_leave_left (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    zero_msec;
    uint32_t    first;

    group_begin("leave setting 1: leave, status 0, join");
    start(p_harness, START_MSEC, true);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_JOIN, 4000U + PASS_MSEC);
    expect_change(p_harness, first + 2U, S1_WIFI_REJOIN_BACKOFF,
                  S1_WIFI_REJOIN_LEAVING, S1_WIFI_REJOIN_WHY_WAIT_OVER,
                  drop_msec + 4000U, "BACKOFF to LEAVING at 4 s");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_LEAVE, drop_msec + 4000U,
                  "a leave, not a join");
    drive(p_harness, STATUS_JOIN, 300U);
    zero_msec = p_harness->now_msec;
    drive(p_harness, STATUS_DOWN, PASS_MSEC);
    expect_change(p_harness, first + 3U, S1_WIFI_REJOIN_LEAVING,
                  S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_WHY_LEFT, zero_msec,
                  "status 0 ends LEAVING");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, zero_msec,
                  "the join is in the same pass");
    drive(p_harness, STATUS_JOIN, 3500U);
    drive(p_harness, STATUS_UP, PASS_MSEC);
    check(S1_WIFI_REJOIN_UP == p_harness->rejoin.state, "UP after the join",
          (uint32_t)p_harness->rejoin.state);

    /* Boot join stuck, leave setting 1: leave at 12 s, join at 13 s. */
    start(p_harness, START_MSEC, true);
    drive(p_harness, STATUS_JOIN, 13000U + PASS_MSEC);
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_LEAVE, START_MSEC + 12000U,
                  "boot: the first retry leaves first");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 13000U,
                  "boot: then joins at the 1 s leave limit");
    group_end();
}

/*!
 * @brief Leave setting 1: no status 0 within 1 s, and a refused join.
 */
static void
test_leave_limit (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    first;

    group_begin("leave setting 1: no status 0 within 1 s");
    start(p_harness, START_MSEC, true);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_JOIN, 15000U + PASS_MSEC);
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_LEAVE, drop_msec + 4000U,
                  "leave at 4 s");
    expect_change(p_harness, first + 3U, S1_WIFI_REJOIN_LEAVING,
                  S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_WHY_LEAVE_LIMIT,
                  drop_msec + 5000U, "join anyway at the 1 s limit");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, drop_msec + 5000U,
                  "the join");
    expect_change(p_harness, first + 4U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  drop_msec + 15000U, "the join limit runs from the join");

    /* The driver refuses the join after LEAVING: a failed attempt. */
    start(p_harness, START_MSEC, true);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    p_harness->b_refuse_join = true;
    drive(p_harness, STATUS_JOIN, 5000U + PASS_MSEC);
    expect_change(p_harness, first + 4U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_REFUSED,
                  drop_msec + 5000U, "a refused join after LEAVING");
    expect_backoff(p_harness, first + 4U, 1U, 2000U, "attempt 1, wait 2 s");
    group_end();
}

/*!
 * @brief Leave setting 1: status 3 in LEAVING is not taken as UP.
 */
static void
test_leave_up_is_not_up (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    first;

    group_begin("leave setting 1: status 3 in LEAVING is not UP");
    start(p_harness, START_MSEC, true);
    go_up_stable(p_harness);
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_JOIN, 4000U + PASS_MSEC);
    drive(p_harness, STATUS_UP, 1000U - PASS_MSEC);
    check(S1_WIFI_REJOIN_LEAVING == p_harness->rejoin.state,
          "still LEAVING with status 3", (uint32_t)p_harness->rejoin.state);
    check((first + 3U) == p_harness->change_count, "no change on status 3",
          p_harness->change_count);
    drive(p_harness, STATUS_UP, 2U * PASS_MSEC);
    expect_change(p_harness, first + 3U, S1_WIFI_REJOIN_LEAVING,
                  S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_WHY_LEAVE_LIMIT,
                  drop_msec + 5000U, "joins at the limit");
    expect_change(p_harness, first + 4U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_UP, S1_WIFI_REJOIN_WHY_LINK_UP,
                  drop_msec + 5000U + PASS_MSEC, "then JOINING sees the 3");
    group_end();
}

/*!
 * @brief The setting this build was configured with.
 */
static void
test_configured_setting (void)
{
    harness_t *             p_harness = &g_harness;
    s1_wifi_rejoin_action_t expected  = S1_WIFI_REJOIN_DO_JOIN;

    /* cyw43_utk.c passes the switch the same way. */
    if (0 != S1_WIFI_LEAVE_BEFORE_JOIN)
    {
        expected = S1_WIFI_REJOIN_DO_LEAVE;
    }

    group_begin("the configured setting (S1_WIFI_LEAVE_BEFORE_JOIN)");
    start(p_harness, START_MSEC, (0 != S1_WIFI_LEAVE_BEFORE_JOIN));
    drive(p_harness, STATUS_JOIN, 12000U + PASS_MSEC);
    expect_action(p_harness, 0U, expected, START_MSEC + 12000U,
                  "the first retry follows the switch");
    group_end();
}

/* ---------------------------------------------------- refused, wrap */

/*!
 * @brief A join request the driver refuses is a failed attempt at once.
 */
static void
test_refused (void)
{
    harness_t *             p_harness = &g_harness;
    s1_wifi_rejoin_change_t change;

    group_begin("a join request the driver refuses");
    start(p_harness, START_MSEC, false);
    p_harness->b_refuse_join = true;
    drive(p_harness, STATUS_JOIN, 16000U + PASS_MSEC);
    expect_change(p_harness, 1U, S1_WIFI_REJOIN_BACKOFF,
                  S1_WIFI_REJOIN_JOINING, S1_WIFI_REJOIN_WHY_WAIT_OVER,
                  START_MSEC + 12000U, "the retry at 12 s");
    expect_change(p_harness, 2U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_REFUSED,
                  START_MSEC + 12000U, "refused: BACKOFF in the same pass");
    expect_backoff(p_harness, 2U, 2U, 4000U, "attempt 2, wait 4 s");
    expect_action(p_harness, 1U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 16000U,
                  "next try 4 s later, not after a 10 s limit");

    /* Outside JOINING it changes nothing. */
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    s1_wifi_rejoin_refused(&p_harness->rejoin, p_harness->now_msec, &change);
    check(!change.b_changed, "in UP: no change", (uint32_t)change.to);
    check(S1_WIFI_REJOIN_UP == p_harness->rejoin.state, "still UP",
          (uint32_t)p_harness->rejoin.state);
    check(0U == p_harness->rejoin.attempt, "attempt still 0",
          p_harness->rejoin.attempt);

    /* Right after the boot join, as if the boot request were refused. */
    start(p_harness, START_MSEC, false);
    s1_wifi_rejoin_refused(&p_harness->rejoin, START_MSEC, &change);
    check(change.b_changed && (S1_WIFI_REJOIN_BACKOFF == change.to)
          && (1U == change.attempt) && (2000U == change.wait_msec),
          "boot: BACKOFF, attempt 1, wait 2 s", (uint32_t)change.to);
    group_end();
}

/*!
 * @brief The 32-bit clock wrap in JOINING and in LOST.
 */
static void
test_wrap (void)
{
    harness_t * p_harness = &g_harness;
    uint32_t    drop_msec;
    uint32_t    first;

    group_begin("32-bit clock wrap");
    /* The boot join's limit runs across the wrap. */
    start(p_harness, WRAP_START_MSEC, false);
    drive(p_harness, STATUS_JOIN, 12000U + PASS_MSEC);
    expect_change(p_harness, 0U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  WRAP_START_MSEC + 10000U, "join limit across the wrap");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN,
                  WRAP_START_MSEC + 12000U, "rejoin across the wrap");

    /* The grace runs across the wrap. */
    start(p_harness, START_MSEC, false);
    go_up_stable(p_harness);
    p_harness->now_msec = UINT32_MAX - 999U;
    first  = p_harness->change_count;
    drop_msec = p_harness->now_msec;
    drive(p_harness, STATUS_JOIN, 14000U + PASS_MSEC);
    expect_change(p_harness, first + 1U, S1_WIFI_REJOIN_LOST,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_GRACE_OVER,
                  drop_msec + 3000U, "grace across the wrap");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, drop_msec + 4000U,
                  "wait across the wrap");
    expect_change(p_harness, first + 3U, S1_WIFI_REJOIN_JOINING,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_JOIN_LIMIT,
                  drop_msec + 14000U, "join limit after the wrap");
    group_end();
}

/* ------------------------------------------------------- odds, ends */

/*!
 * @brief NULL arguments and an invalid stored state.
 */
static void
test_null_and_names (void)
{
    harness_t *             p_harness = &g_harness;
    s1_wifi_rejoin_change_t change;
    s1_wifi_rejoin_t        before;

    group_begin("NULL arguments, an invalid state");
    check(S1_WIFI_REJOIN_DO_NONE
          == s1_wifi_rejoin_step(NULL, START_MSEC, STATUS_UP, &change),
          "step with no context: nothing", 0U);
    start(p_harness, START_MSEC, false);
    before = p_harness->rejoin;
    check(S1_WIFI_REJOIN_DO_NONE
          == s1_wifi_rejoin_step(&p_harness->rejoin, START_MSEC + 20000U,
                                 STATUS_UP, NULL),
          "step with no change record: nothing", 0U);
    check((before.state == p_harness->rejoin.state)
          && (before.since_msec == p_harness->rejoin.since_msec),
          "and the context is untouched", (uint32_t)p_harness->rejoin.state);
    s1_wifi_rejoin_init(NULL, START_MSEC, false);
    s1_wifi_rejoin_refused(NULL, START_MSEC, &change);
    s1_wifi_rejoin_refused(&p_harness->rejoin, START_MSEC, NULL);
    check(S1_WIFI_REJOIN_JOINING == p_harness->rejoin.state,
          "refused with no change record: nothing",
          (uint32_t)p_harness->rejoin.state);

    /* An invalid stored state (corrupted memory) starts the ladder. The
     * cast makes a value no enumerator has, on purpose.
     */
    start(p_harness, START_MSEC, false);
    p_harness->rejoin.state = (s1_wifi_rejoin_state_t)99;
    drive(p_harness, STATUS_JOIN, 1000U + PASS_MSEC);
    expect_change(p_harness, 0U, (s1_wifi_rejoin_state_t)99,
                  S1_WIFI_REJOIN_BACKOFF, S1_WIFI_REJOIN_WHY_BAD_STATE,
                  START_MSEC, "an invalid state goes to BACKOFF");
    expect_action(p_harness, 0U, S1_WIFI_REJOIN_DO_JOIN, START_MSEC + 1000U,
                  "and joins after the wait");
    group_end();
}

/*!
 * @brief The console names of every state and reason.
 */
static void
test_names (void)
{
    group_begin("console names");
    /* Every name the console line can print. The casts make values no
     * enumerator has, on purpose.
     */
    check(0 == strcmp("UP", s1_wifi_rejoin_state_name(S1_WIFI_REJOIN_UP)),
          "name UP", 0U);
    check(0 == strcmp("LOST",
                      s1_wifi_rejoin_state_name(S1_WIFI_REJOIN_LOST)),
          "name LOST", 0U);
    check(0 == strcmp("BACKOFF",
                      s1_wifi_rejoin_state_name(S1_WIFI_REJOIN_BACKOFF)),
          "name BACKOFF", 0U);
    check(0 == strcmp("JOINING",
                      s1_wifi_rejoin_state_name(S1_WIFI_REJOIN_JOINING)),
          "name JOINING", 0U);
    check(0 == strcmp("LEAVING",
                      s1_wifi_rejoin_state_name(S1_WIFI_REJOIN_LEAVING)),
          "name LEAVING", 0U);
    check(0 == strcmp("?", s1_wifi_rejoin_state_name(
                               (s1_wifi_rejoin_state_t)99)),
          "an invalid state is \"?\"", 0U);
    check(0 == strcmp("no change",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_NONE)),
          "reason no change", 0U);
    check(0 == strcmp("link up",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_LINK_UP)),
          "reason link up", 0U);
    check(0 == strcmp("link lost",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_LINK_LOST)),
          "reason link lost", 0U);
    check(0 == strcmp("negative status",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_NEGATIVE)),
          "reason negative status", 0U);
    check(0 == strcmp("grace over",
                      s1_wifi_rejoin_why_text(
                          S1_WIFI_REJOIN_WHY_GRACE_OVER)),
          "reason grace over", 0U);
    check(0 == strcmp("wait over",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_WAIT_OVER)),
          "reason wait over", 0U);
    check(0 == strcmp("join limit",
                      s1_wifi_rejoin_why_text(
                          S1_WIFI_REJOIN_WHY_JOIN_LIMIT)),
          "reason join limit", 0U);
    check(0 == strcmp("join request refused",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_REFUSED)),
          "reason join request refused", 0U);
    check(0 == strcmp("left",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_LEFT)),
          "reason left", 0U);
    check(0 == strcmp("leave limit",
                      s1_wifi_rejoin_why_text(
                          S1_WIFI_REJOIN_WHY_LEAVE_LIMIT)),
          "reason leave limit", 0U);
    check(0 == strcmp("bad state",
                      s1_wifi_rejoin_why_text(S1_WIFI_REJOIN_WHY_BAD_STATE)),
          "reason bad state", 0U);
    check(0 == strcmp("?", s1_wifi_rejoin_why_text(
                               (s1_wifi_rejoin_why_t)99)),
          "an invalid reason is \"?\"", 0U);
    group_end();
}

/*!
 * @brief Run every group; exit 0 only if every check passed.
 */
int
main (void)
{
    printf("S1-U28 s1_wifi_rejoin host test, S1_WIFI_LEAVE_BEFORE_JOIN %d\n",
           (int)S1_WIFI_LEAVE_BEFORE_JOIN);

    test_boot_join_ok();
    test_boot_join_badauth();
    test_boot_join_stuck();
    test_drop_past_grace();
    test_recover_in_grace();
    test_negative_in_grace();
    test_badauth_then_join();
    test_status0_forever();
    test_status2_stuck();
    test_ladder();
    test_counter_reset();
    test_no_action_while_up();
    test_leave_off();
    test_leave_left();
    test_leave_limit();
    test_leave_up_is_not_up();
    test_configured_setting();
    test_refused();
    test_wrap();
    test_null_and_names();
    test_names();

    if (0U == g_failed)
    {
        printf("RESULT: all passed (%" PRIu32 " checks)\n", g_checks);
    }
    else
    {
        printf("RESULT: %" PRIu32 " FAILED (%" PRIu32 " checks)\n", g_failed,
               g_checks);
    }

    return ((0U == g_failed) ? EXIT_SUCCESS : EXIT_FAILURE);
}

/*** end of file ***/

