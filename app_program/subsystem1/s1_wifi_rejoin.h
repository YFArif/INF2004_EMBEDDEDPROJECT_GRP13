/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_wifi_rejoin.h
 *
 * @brief S1's WiFi rejoin policy (FR17): from the time and the radio link
 *        status alone, decides when the radio service task joins the
 *        access point again.
 *
 * @par Why it exists
 * The template's retry in cyw43_utk_task() joins again only after a
 * negative link status, and has no time limit on any other status. After
 * an access point loss the driver reads 1 (CYW43_LINK_JOIN) for as long as
 * the access point is away, and often after it is back; a join that hangs
 * reads 1 as well. So recovery depended on the radio chip alone. This
 * policy puts a time limit on every state that is not UP (docs/MQTT.md
 * section 9).
 *
 * @par States
 * - UP: status 3 (CYW43_LINK_UP). The attempt counter is reset only after
 *   S1_WIFI_REJOIN_STABLE_MSEC of continuous UP: after a rejoin, status 3
 *   means "associated, with the old address", not "lease confirmed".
 * - LOST: any other status after UP. The chip gets
 *   S1_WIFI_REJOIN_GRACE_MSEC to recover alone. Status 3 goes back to UP;
 *   a negative status, or the end of the grace, goes to BACKOFF.
 * - BACKOFF: waits 1, 2, 4, 4, 4 ... s by attempt number. Status 3 goes
 *   back to UP. At the end of the wait: a join (JOINING), or, when
 *   S1_WIFI_LEAVE_BEFORE_JOIN is 1, a leave first (LEAVING).
 * - JOINING: S1_WIFI_REJOIN_JOIN_LIMIT_MSEC from the join request. Status 3
 *   goes to UP. A negative status, the limit reached at status 0, 1 or 2,
 *   or a join request the driver refuses, counts as a failed attempt and
 *   goes to BACKOFF. A status of -3 is never taken as final.
 * - LEAVING (only when S1_WIFI_LEAVE_BEFORE_JOIN is 1): waits for status 0
 *   for at most S1_WIFI_REJOIN_LEAVE_LIMIT_MSEC, then joins anyway. A status
 *   of 3 here is not taken as UP, because a leave is in flight.
 * The boot join is the first JOINING, so a first join that fails or hangs
 * is retried on the same ladder.
 *
 * @par How it is used
 * In MQTT=1 builds, cyw43_utk_task() (lib/libwifi/sysdepend/pico_rp2040/
 * cyw43_utk.c) calls s1_wifi_rejoin_init() after its boot join, then
 * s1_wifi_rejoin_step() on every pass with the status it has just read,
 * and carries out the action. When the driver refuses a join request it
 * calls s1_wifi_rejoin_refused(). The template's own retry is compiled out
 * there, so two retry policies never run together.
 *
 * @par No state of its own
 * The module calls nothing and has no variables: everything it keeps is in
 * the s1_wifi_rejoin_t its caller passes in, so the host test S1-U28 drives
 * it with a fake clock. Its only file-scope data is the const ladder table.
 *
 * @par Thread safety
 * One caller: the radio service task.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0), like the rest of
 * the MQTT link, so builds without MQTT are unchanged.
 */

#ifndef S1_WIFI_REJOIN_H
#define S1_WIFI_REJOIN_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

/* Leave before every rejoin. 0 (the default) sends a plain join, which is
 * what the template does after a -3 and what cyw43_wifi_join() accepts
 * while a join is active. 1 adds the LEAVING state.
 * WARNING: cyw43_wifi_leave() only sends a request; the driver clears its
 * join state later, when the disassociation event arrives. An event that
 * arrives after the new join has set the ACTIVE bit clears that bit, and
 * the driver can then never report link-up (the JOINING limit recovers
 * from it). Neither setting has run on hardware; the drop series S1-U29
 * decides the default. Override from the make command line with
 * GCC="arm-none-eabi-gcc -DS1_WIFI_LEAVE_BEFORE_JOIN=1".
 */
#ifndef S1_WIFI_LEAVE_BEFORE_JOIN
#define S1_WIFI_LEAVE_BEFORE_JOIN       0
#endif

/* The two link status values the policy names: CYW43_LINK_DOWN and
 * CYW43_LINK_UP in cyw43.h, which this module cannot include (the host test
 * builds without the SDK). cyw43_utk.c checks both at compile time.
 */
#define S1_WIFI_REJOIN_LINK_DOWN        0
#define S1_WIFI_REJOIN_LINK_UP          3

/* The timings below are starting points; the drop series S1-U29 tunes
 * them.
 */

/* Continuous UP before the attempt counter is reset. */
#define S1_WIFI_REJOIN_STABLE_MSEC      10000U

/* Time the chip gets to recover alone after the link is lost. */
#define S1_WIFI_REJOIN_GRACE_MSEC       3000U

/* Time limit of one join, from the request. Good joins took 3.2 to 3.9 s
 * from the request to link up (S1-U15 and S1-U21 consoles).
 */
#define S1_WIFI_REJOIN_JOIN_LIMIT_MSEC  10000U

/* Longest wait for status 0 after a leave request. */
#define S1_WIFI_REJOIN_LEAVE_LIMIT_MSEC 1000U

/* The back-off ladder: the wait before join attempt n is step n, and every
 * attempt from the last step on waits the last step (the cap).
 */
#define S1_WIFI_REJOIN_WAIT_1_MSEC      1000U
#define S1_WIFI_REJOIN_WAIT_2_MSEC      2000U
#define S1_WIFI_REJOIN_WAIT_CAP_MSEC    4000U
#define S1_WIFI_REJOIN_WAIT_STEPS       3U

/* The attempt counter stops here instead of wrapping to 0. */
#define S1_WIFI_REJOIN_ATTEMPT_MAX      UINT32_MAX

/* The policy's states (see the file comment). */
typedef enum
{
    S1_WIFI_REJOIN_UP = 0,
    S1_WIFI_REJOIN_LOST,
    S1_WIFI_REJOIN_BACKOFF,
    S1_WIFI_REJOIN_JOINING,
    S1_WIFI_REJOIN_LEAVING
} s1_wifi_rejoin_state_t;

/* What the caller must do on this pass. */
typedef enum
{
    S1_WIFI_REJOIN_DO_NONE = 0,
    S1_WIFI_REJOIN_DO_JOIN,
    S1_WIFI_REJOIN_DO_LEAVE
} s1_wifi_rejoin_action_t;

/* Why the state changed. */
typedef enum
{
    S1_WIFI_REJOIN_WHY_NONE = 0,      /* no change */
    S1_WIFI_REJOIN_WHY_LINK_UP,       /* status 3 */
    S1_WIFI_REJOIN_WHY_LINK_LOST,     /* a status other than 3 while UP */
    S1_WIFI_REJOIN_WHY_NEGATIVE,      /* a negative status */
    S1_WIFI_REJOIN_WHY_GRACE_OVER,    /* LOST for the whole grace */
    S1_WIFI_REJOIN_WHY_WAIT_OVER,     /* the back-off wait is over */
    S1_WIFI_REJOIN_WHY_JOIN_LIMIT,    /* JOINING for the whole limit */
    S1_WIFI_REJOIN_WHY_REFUSED,       /* the driver refused the request */
    S1_WIFI_REJOIN_WHY_LEFT,          /* status 0 after the leave */
    S1_WIFI_REJOIN_WHY_LEAVE_LIMIT,   /* no status 0 within the limit */
    S1_WIFI_REJOIN_WHY_BAD_STATE      /* the stored state was invalid */
} s1_wifi_rejoin_why_t;

/* Everything the policy keeps between passes. The caller owns it; only
 * this module's functions write it.
 */
typedef struct
{
    s1_wifi_rejoin_state_t state;
    uint32_t               since_msec;   /* entry time; JOINING: request */
    uint32_t               wait_msec;    /* BACKOFF: this wait; else 0 */
    uint32_t               attempt;      /* failed joins since stable UP */
    int32_t                last_status;  /* status at the last step */
    bool                   b_stable;     /* UP long enough this time */
    bool                   b_leave_first;
} s1_wifi_rejoin_t;

/* What a call changed, for the caller's console line. */
typedef struct
{
    bool                   b_changed;    /* false: nothing to print */
    s1_wifi_rejoin_state_t from;
    s1_wifi_rejoin_state_t to_state;
    s1_wifi_rejoin_why_t   why;
    uint32_t               attempt;      /* the counter after the change */
    uint32_t               wait_msec;    /* BACKOFF: the wait; else 0 */
    int32_t                link_status;  /* the status that was given */
} s1_wifi_rejoin_change_t;

void                    s1_wifi_rejoin_init(s1_wifi_rejoin_t * p_rejoin,
                                            uint32_t now_msec,
                                            bool b_leave_first);
s1_wifi_rejoin_action_t s1_wifi_rejoin_step(s1_wifi_rejoin_t * p_rejoin,
                                            uint32_t now_msec,
                                            int32_t link_status,
                                            s1_wifi_rejoin_change_t *
                                                p_change);
void                    s1_wifi_rejoin_refused(s1_wifi_rejoin_t * p_rejoin,
                                               uint32_t now_msec,
                                               s1_wifi_rejoin_change_t *
                                                   p_change);
uint32_t                s1_wifi_rejoin_wait_msec(uint32_t attempt);
const char *            s1_wifi_rejoin_state_name(
                            s1_wifi_rejoin_state_t state);
const char *            s1_wifi_rejoin_why_text(s1_wifi_rejoin_why_t why);

#endif /* TM_MQTT */

#endif /* S1_WIFI_REJOIN_H */

/*** end of file ***/

