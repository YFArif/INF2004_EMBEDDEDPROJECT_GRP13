/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_event.c
 *
 * @brief S1's event channel, mbf_event (see s1_event.h).
 *
 * @par Kernel side only
 * This file includes the kernel headers and no lwIP or cyw43 header, like
 * mqtt_queue.c (the two disagree about size_t).
 *
 * @par Drop count
 * Tasks on both cores may post at the same moment, so the drop count is
 * raised the same way as mqtt_queue.c's full count: atomic_add() in a
 * dual-core build, dispatch disabled around the increment in a single-core
 * build.
 */

#include "s1_event.h"
#include "subsystem1.h"

#if TM_MQTT

/* NULL comes from the kernel headers: <stddef.h> would clash with their
 * size_t (see mqtt_queue.c).
 */
#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

/* The kernel stores a length header (one INT) in front of every record and
 * rounds the record up to a whole INT.
 */
#define MBF_UNIT_BYTES          4U
#define RECORD_COST_BYTES       (MBF_UNIT_BYTES + (((MSG_EVENT_RECORD_BYTES \
                                 + MBF_UNIT_BYTES) - 1U) / MBF_UNIT_BYTES)  \
                                 * MBF_UNIT_BYTES)
#define EVENT_BUF_BYTES         (S1_EVENT_DEPTH * RECORD_COST_BYTES)

/* Compile-time checks: the record has no padding, the kernel's units are
 * the sizes assumed above, and the buffer is the 240 bytes the layout
 * document states.
 */
typedef uint8_t s1_event_check_record_t[(sizeof(msg_layout_event_t)
                                         == MSG_EVENT_RECORD_BYTES) ? 1 : -1];
typedef uint8_t s1_event_check_unit_t[((sizeof(INT) == MBF_UNIT_BYTES)
                                       && (sizeof(UW) == sizeof(uint32_t)))
                                      ? 1 : -1];
typedef uint8_t s1_event_check_size_t[(240U == EVENT_BUF_BYTES) ? 1 : -1];

/* Storage for mbf_event, as 32-bit words for the header's alignment. */
static uint32_t           g_event_buf[EVENT_BUF_BYTES / MBF_UNIT_BYTES];

/* Read by tasks on both cores. The ID is written once, before any task
 * other than usermain runs.
 */
static volatile ID        gh_event_mbf = 0;
static volatile uint32_t  g_drop_count = 0U;

static void count_drop(void);

/*!
 * @brief Create mbf_event (see s1_event.h).
 *
 * @return 0, or the negative kernel error from tk_cre_mbf().
 */
int32_t
s1_event_init (void)
{
    T_CMBF  cmbf = {0};
    ID      h_mbf = 0;
    int32_t result = 0;

    /* The sizes are compile-time constants (240 and 3 bytes), so the casts
     * to the kernel's signed size types keep their values.
     */
    cmbf.mbfatr = TA_USERBUF;
    cmbf.bufsz  = (SZ)sizeof(g_event_buf);
    cmbf.maxmsz = (INT)MSG_EVENT_RECORD_BYTES;
    cmbf.bufptr = g_event_buf;

    h_mbf = tk_cre_mbf(&cmbf);

    if (h_mbf < E_OK)
    {
        /* ER is the kernel's W, a signed 32-bit integer on this port. */
        result = (int32_t)h_mbf;
    }
    else
    {
        gh_event_mbf = h_mbf;
    }

    return (result);
}

/*!
 * @brief Report one event to the dashboard (public; see subsystem1.h).
 *
 * Never waits. Safe from any task on either core; not from an interrupt
 * handler.
 *
 * @param[in] code  The event code (S5's table: 0x50 to 0x65; 0 is never
 *                  used).
 * @param[in] arg_a First argument byte.
 * @param[in] arg_b Second argument byte.
 *
 * @return True if the record was queued; false if it was dropped (buffer
 *         full, or no buffer because the MQTT link did not start).
 */
bool
subsystem1_event_post (uint8_t code, uint8_t arg_a, uint8_t arg_b)
{
    msg_layout_event_t record = {0U, {0U}};
    bool               b_queued = false;
    const ID           h_mbf = gh_event_mbf;

    record.code    = code;
    record.args[0] = arg_a;
    record.args[1] = arg_b;

    /* The record is 3 bytes, so the cast to INT keeps the value. */
    if ((h_mbf > 0)
        && (E_OK == tk_snd_mbf(h_mbf, &record, (INT)MSG_EVENT_RECORD_BYTES,
                               TMO_POL)))
    {
        b_queued = true;
    }
    else
    {
        count_drop();
    }

    return (b_queued);
}

/*!
 * @brief Take the oldest event record without waiting (see s1_event.h).
 *
 * @param[out] p_event Receives the record.
 *
 * @return True if a record was taken.
 */
bool
s1_event_take (msg_layout_event_t * p_event)
{
    bool     b_taken = false;
    INT      received = 0;
    const ID h_mbf = gh_event_mbf;

    if ((NULL != p_event) && (h_mbf > 0))
    {
        received = tk_rcv_mbf(h_mbf, p_event, TMO_POL);

        /* The cast compares a size the kernel returned with a small
         * constant; a negative error never matches.
         */
        b_taken = ((INT)MSG_EVENT_RECORD_BYTES == received);
    }

    return (b_taken);
}

/*!
 * @brief Count of dropped posts since boot (see s1_event.h).
 *
 * @return The count. One aligned word, so a read is never torn.
 */
uint32_t
s1_event_drop_count (void)
{
    return (g_drop_count);
}

/*!
 * @brief Add one to the drop count.
 *
 * Any task on either core may call this at the same time.
 */
static void
count_drop (void)
{
#if TK_SUPPORT_SMP
    /* The cast only removes volatile; UW and uint32_t are the same type
     * here (checked above), and atomic_add() accesses the word through a
     * volatile pointer itself. The new count it returns is not needed.
     */
    (void)atomic_add((UW *)&g_drop_count, 1U);
#else
    /* One core: keeping other tasks off the CPU between the read and the
     * write is enough. Both calls only fail outside a task, and this runs
     * in one.
     */
    (void)tk_dis_dsp();
    g_drop_count++;
    (void)tk_ena_dsp();
#endif
}

#else /* TM_MQTT */

/*!
 * @brief Report one event: without MQTT there is no channel.
 *
 * @param[in] code  Unused.
 * @param[in] arg_a Unused.
 * @param[in] arg_b Unused.
 *
 * @return False: the event is not sent.
 */
bool
subsystem1_event_post (uint8_t code, uint8_t arg_a, uint8_t arg_b)
{
    /* The casts mark the arguments as unused in this build. */
    (void)code;
    (void)arg_a;
    (void)arg_b;

    return (false);
}

#endif /* TM_MQTT */

/*** end of file ***/
