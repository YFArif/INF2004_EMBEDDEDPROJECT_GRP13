/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file mqtt_queue.c
 *
 * @brief Kernel side of the MQTT link: the two message buffers.
 *
 * @par
 * See mqtt_queue.h for why this module exists apart from mqtt_link.c.
 * This file includes <tk/tkernel.h> and must never include an lwIP header.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0) -- see mqtt_link.h.
 */

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#include "mqtt_queue.h"

/* Size of the kernel's per-message length header, and also the unit that
 * message data is rounded up to. It is sizeof(INT); see HEADERSZ and
 * ROUNDSZ in kernel/tkernel/messagebuf.h. The check below keeps this
 * constant honest.
 */
#define MBF_UNIT_BYTES          4U

/* Largest wait, in milliseconds, that is passed to the kernel. A longer
 * wait is shortened to this, which is about 24 days.
 */
#define WAIT_MAX_MSEC             0x7FFFFFFFU

/* sizeof is a compile-time constant of a few tens of bytes, so converting
 * it to uint32_t cannot lose any bits.
 */
#define PUBLISH_RECORD_BYTES    ((uint32_t)sizeof(mqtt_queue_publish_t))
#define COMMAND_RECORD_BYTES    ((uint32_t)sizeof(mqtt_link_command_t))

/* Space one record takes in a message buffer: the length header plus the
 * record rounded up to a multiple of MBF_UNIT_BYTES.
 */
#define PUBLISH_COST_BYTES      (MBF_UNIT_BYTES + \
    (((PUBLISH_RECORD_BYTES + MBF_UNIT_BYTES - 1U) / MBF_UNIT_BYTES) \
     * MBF_UNIT_BYTES))
#define COMMAND_COST_BYTES      (MBF_UNIT_BYTES + \
    (((COMMAND_RECORD_BYTES + MBF_UNIT_BYTES - 1U) / MBF_UNIT_BYTES) \
     * MBF_UNIT_BYTES))

/* Buffer sizes. Every record is sent at its full size, and the kernel
 * stores each as a length header plus the record rounded up to a multiple
 * of 4 bytes, so a buffer holds exactly MQTT_LINK_QUEUE_DEPTH records when
 * it is MQTT_LINK_QUEUE_DEPTH times the cost of one:
 *
 *   publish:  32-byte record ->  32 + 4 =   36 bytes;  8 x  36 =  288 bytes
 *   command:   4-byte record ->   4 + 4 =    8 bytes;  8 x   8 =   64 bytes
 *   total static RAM for the two buffers                          352 bytes
 *
 * NOTE: The figures above are for a depth of 8, a one-byte topic ID,
 * MQTT_LINK_PAYLOAD_MAX 25 and MQTT_LINK_COMMAND_MAX 2 (both from
 * msg_layout.h). The macros recompute them if those constants change.
 */
#define PUBLISH_BUF_BYTES       (MQTT_LINK_QUEUE_DEPTH * PUBLISH_COST_BYTES)
#define COMMAND_BUF_BYTES       (MQTT_LINK_QUEUE_DEPTH * COMMAND_COST_BYTES)

/* Compile-time checks. Each typedef has a negative array size, and so stops
 * the build, when its condition is false.
 */
typedef uint8_t mqtt_queue_check_unit_t[(sizeof(INT) == MBF_UNIT_BYTES)
                                        ? 1 : -1];
typedef uint8_t mqtt_queue_check_word_t[(sizeof(uint32_t) == sizeof(UW))
                                        ? 1 : -1];
typedef uint8_t mqtt_queue_check_depth_t[(MQTT_LINK_QUEUE_DEPTH > 0U)
                                         ? 1 : -1];

/* Storage for the message buffers. A kernel buffer supplied by the caller
 * must be aligned for the length header, so it is declared as 32-bit words.
 */
static uint32_t g_publish_buf[PUBLISH_BUF_BYTES / MBF_UNIT_BYTES];
static uint32_t g_command_buf[COMMAND_BUF_BYTES / MBF_UNIT_BYTES];

/* Kernel object IDs and the ready flag. They are volatile because tasks on
 * both cores read them, and gb_ready is written last so that a reader that
 * sees it set also sees both IDs.
 */
static volatile ID   gh_publish_mbf = 0;
static volatile ID   gh_command_mbf = 0;
static volatile bool gb_ready = false;

/* Publishes dropped because the channel was full. */
static volatile uint32_t g_full_count = 0U;

static int32_t create_buffers(void);
static int32_t create_buffer(uint32_t * p_buf, uint32_t buf_bytes,
                             uint32_t record_bytes, volatile ID * ph_mbf);
static void memory_barrier(void);
static void count_full(void);
static int32_t put_record(ID h_mbf, const void * p_record,
                          uint32_t record_bytes);
static int32_t get_record(ID h_mbf, void * p_record, uint32_t record_bytes,
                          int32_t wait_msec);

/*!
 * @brief Create the two message buffers.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_queue_init (void)
{
    int32_t result = MQTT_LINK_OK;

    if (!gb_ready)
    {
        result = create_buffers();

        if (MQTT_LINK_OK == result)
        {
            memory_barrier();
            gb_ready = true;
        }
    }

    return (result);
}

/*!
 * @brief Report whether mqtt_queue_init() has succeeded.
 *
 * @return True once the buffers exist.
 */
bool
mqtt_queue_is_ready (void)
{
    const bool b_ready = gb_ready;

    if (b_ready)
    {
        /* Order the read of the IDs that follow after the flag. */
        memory_barrier();
    }

    return (b_ready);
}

/*!
 * @brief Put one message on the publish channel.
 *
 * @param[in] p_msg The message.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_FULL or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_queue_put_publish (const mqtt_queue_publish_t * p_msg)
{
    int32_t result = MQTT_LINK_OK;

    if (NULL == p_msg)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if (p_msg->payload_len > MQTT_LINK_PAYLOAD_MAX)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if (!mqtt_queue_is_ready())
    {
        result = MQTT_LINK_ERR_NOT_READY;
    }
    else
    {
        result = put_record(gh_publish_mbf, p_msg, PUBLISH_RECORD_BYTES);

        if (MQTT_LINK_ERR_FULL == result)
        {
            count_full();
        }
    }

    return (result);
}

/*!
 * @brief Take the oldest message from the publish channel.
 *
 * @param[out] p_msg Receives the message.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_TIMEOUT when the channel is empty,
 *         MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY or
 *         MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_queue_get_publish (mqtt_queue_publish_t * p_msg)
{
    int32_t result = MQTT_LINK_OK;

    if (NULL == p_msg)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if (!mqtt_queue_is_ready())
    {
        result = MQTT_LINK_ERR_NOT_READY;
    }
    else
    {
        result = get_record(gh_publish_mbf, p_msg, PUBLISH_RECORD_BYTES, 0);

        if ((MQTT_LINK_OK == result)
            && (p_msg->payload_len > MQTT_LINK_PAYLOAD_MAX))
        {
            /* A stored record can only be damaged by a bug; refuse it. */
            result = MQTT_LINK_ERR_KERNEL;
        }
    }

    return (result);
}

/*!
 * @brief Put one command on the command channel.
 *
 * @param[in] p_command The command.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_FULL or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_queue_put_command (const mqtt_link_command_t * p_command)
{
    int32_t result = MQTT_LINK_OK;

    if (NULL == p_command)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if (p_command->payload_len > MQTT_LINK_COMMAND_MAX)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if (!mqtt_queue_is_ready())
    {
        result = MQTT_LINK_ERR_NOT_READY;
    }
    else
    {
        result = put_record(gh_command_mbf, p_command, COMMAND_RECORD_BYTES);
    }

    return (result);
}

/*!
 * @brief Take the oldest command from the command channel.
 *
 * @param[out] p_command  Receives the command.
 * @param[in]  timeout_msec Wait limit in milliseconds, or
 *                        MQTT_LINK_WAIT_FOREVER.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_TIMEOUT, MQTT_LINK_ERR_PARAM,
 *         MQTT_LINK_ERR_NOT_READY or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_queue_get_command (mqtt_link_command_t * p_command, uint32_t timeout_msec)
{
    int32_t result = MQTT_LINK_OK;
    int32_t wait_msec = 0;

    if (NULL == p_command)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if (!mqtt_queue_is_ready())
    {
        result = MQTT_LINK_ERR_NOT_READY;
    }
    else
    {
        if (MQTT_LINK_WAIT_FOREVER == timeout_msec)
        {
            wait_msec = -1;
        }
        else if (timeout_msec > WAIT_MAX_MSEC)
        {
            /* WAIT_MAX_MSEC is INT32_MAX, so it fits int32_t. */
            wait_msec = (int32_t)WAIT_MAX_MSEC;
        }
        else
        {
            /* The test above leaves timeout_msec at most INT32_MAX. */
            wait_msec = (int32_t)timeout_msec;
        }

        result = get_record(gh_command_mbf, p_command, COMMAND_RECORD_BYTES,
                            wait_msec);

        if ((MQTT_LINK_OK == result)
            && (p_command->payload_len > MQTT_LINK_COMMAND_MAX))
        {
            /* A stored record can only be damaged by a bug; refuse it. */
            result = MQTT_LINK_ERR_KERNEL;
        }
    }

    return (result);
}

/*!
 * @brief Read the number of publishes dropped because a channel was full.
 *
 * @return Drops since start-up.
 */
uint32_t
mqtt_queue_full_count (void)
{
    return (g_full_count);
}

/*!
 * @brief Read the kernel's own millisecond clock.
 *
 * @return Milliseconds since boot, low 32 bits.
 */
uint32_t
mqtt_queue_now_msec (void)
{
    SYSTIM otm;

    tk_get_otm(&otm);
    return otm.lo;
}

/*!
 * @brief Order memory accesses for the other core (see mqtt_queue.h).
 */
void
mqtt_queue_barrier (void)
{
    memory_barrier();
}

/*!
 * @brief Create both message buffers, or neither.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_KERNEL.
 */
static int32_t
create_buffers (void)
{
    int32_t result = create_buffer(g_publish_buf, PUBLISH_BUF_BYTES,
                                   PUBLISH_RECORD_BYTES, &gh_publish_mbf);

    if (MQTT_LINK_OK == result)
    {
        result = create_buffer(g_command_buf, COMMAND_BUF_BYTES,
                               COMMAND_RECORD_BYTES, &gh_command_mbf);

        if (MQTT_LINK_OK != result)
        {
            /* Do not leave half the queue behind. The return value is
             * ignored because the caller can only report the failure
             * that is already in result.
             */
            (void)tk_del_mbf(gh_publish_mbf);
            gh_publish_mbf = 0;
        }
    }

    return (result);
}

/*!
 * @brief Create one message buffer on caller-supplied storage.
 *
 * @param[in]  p_buf         Storage, aligned for 32-bit words.
 * @param[in]  buf_bytes     Storage size in bytes.
 * @param[in]  record_bytes  Size of every record sent through the buffer.
 * @param[out] ph_mbf      Receives the kernel ID.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_KERNEL.
 */
static int32_t
create_buffer (uint32_t * p_buf, uint32_t buf_bytes, uint32_t record_bytes,
               volatile ID * ph_mbf)
{
    int32_t result = MQTT_LINK_OK;
    T_CMBF  cmbf = { 0 };
    ID      h_mbf = 0;

    /* The sizes are compile-time constants of a few hundred bytes, so the
     * casts to the kernel's signed size types cannot overflow.
     */
    cmbf.mbfatr = TA_USERBUF;
    cmbf.bufsz  = (SZ)buf_bytes;
    cmbf.maxmsz = (INT)record_bytes;
    cmbf.bufptr = p_buf;

    h_mbf = tk_cre_mbf(&cmbf);

    if (h_mbf < E_OK)
    {
        result = MQTT_LINK_ERR_KERNEL;
    }
    else
    {
        *ph_mbf = h_mbf;
    }

    return (result);
}

/*!
 * @brief Make writes before this call visible to the other core first.
 *
 * NOTE: A single-core build has no other core, and the volatile accesses
 * are enough there. The barrier exists only in a dual-core build.
 */
static void
memory_barrier (void)
{
#if TK_SUPPORT_SMP
    mp_memory_barrir();
#endif
}

/*!
 * @brief Add one to the count of publishes dropped because of a full
 *        channel.
 *
 * Any task on either core may call this at the same time.
 */
static void
count_full (void)
{
#if TK_SUPPORT_SMP
    /* The cast only removes volatile. UW and uint32_t are the same 32-bit
     * unsigned type here, checked above, and atomic_add() reads and writes
     * the word through a volatile pointer itself. Its return value, the new
     * count, is not needed.
     */
    (void)atomic_add((UW *)&g_full_count, 1U);
#else
    /* One core: no other CPU can touch the word, so it is enough to keep
     * another task from running between the read and the write. The return
     * values are not needed; both calls only fail if the kernel is not
     * running, and a task is running here.
     */
    (void)tk_dis_dsp();
    g_full_count++;
    (void)tk_ena_dsp();
#endif
}

/*!
 * @brief Send one fixed-size record without waiting.
 *
 * @param[in] h_mbf        Message buffer ID.
 * @param[in] p_record      The record.
 * @param[in] record_bytes  Size of the record.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_FULL or MQTT_LINK_ERR_KERNEL.
 */
static int32_t
put_record (ID h_mbf, const void * p_record, uint32_t record_bytes)
{
    int32_t result = MQTT_LINK_OK;
    ER      kernel_result = E_OK;

    /* The record size is a compile-time constant of a few hundred bytes, so
     * the cast to the kernel's signed INT cannot overflow.
     */
    kernel_result = tk_snd_mbf(h_mbf, p_record, (INT)record_bytes, TMO_POL);

    if (E_OK == kernel_result)
    {
        result = MQTT_LINK_OK;
    }
    else if (E_TMOUT == kernel_result)
    {
        /* With no wait, the kernel reports a full buffer as a time-out. */
        result = MQTT_LINK_ERR_FULL;
    }
    else
    {
        result = MQTT_LINK_ERR_KERNEL;
    }

    return (result);
}

/*!
 * @brief Receive one fixed-size record.
 *
 * @param[in]  h_mbf        Message buffer ID.
 * @param[out] p_record      Receives the record.
 * @param[in]  record_bytes  Size every record must have.
 * @param[in]  wait_msec       0 to poll, -1 to wait forever, or a limit.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_TIMEOUT or MQTT_LINK_ERR_KERNEL.
 */
static int32_t
get_record (ID h_mbf, void * p_record, uint32_t record_bytes, int32_t wait_msec)
{
    int32_t result = MQTT_LINK_OK;
    INT     received = 0;

    /* TMO is the kernel's signed 32-bit wait type, the same width as
     * int32_t, and wait_msec is -1, 0 or at most INT32_MAX.
     */
    received = tk_rcv_mbf(h_mbf, p_record, (TMO)wait_msec);

    /* The cast below widens a size the kernel just returned, which is
     * never negative on this branch.
     */
    if ((received > 0) && ((uint32_t)received == record_bytes))
    {
        result = MQTT_LINK_OK;
    }
    else if (E_TMOUT == received)
    {
        result = MQTT_LINK_ERR_TIMEOUT;
    }
    else
    {
        /* A kernel error, or a record of the wrong size. */
        result = MQTT_LINK_ERR_KERNEL;
    }

    return (result);
}

#endif /* TM_MQTT */

/*** end of file ***/
