/** @file s5_ranger.c
 *
 * @brief One complete, paced HC-SR04 measurement with fault detection.
 *
 * @par Timing (HC-SR04 datasheet, Week 6 report 6.4 and Appendix D)
 * Echo width is 58 us per cm of range, so 4 m gives about 23 ms. A ping
 * is read within ECHO_WINDOW_MS; an echo longer than ECHO_MAX_US means
 * nothing in range, which is a clear path, not a fault.
 *
 * @par Faults (Week 6 report 6.4)
 * - FAULT_PINGS pings in a row with no echo pulse at all: the sensor is
 *   not answering (unplugged or unpowered).
 * - ECHO held high for STUCK_US: the sensor is stuck.
 * A fault is latched until s5_ranger_reset_fault().
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_s5_hw.h"
#include "drv_ultrasonic.h"
#include "s5_ranger.h"
#include "subsystem5.h"

#define PING_PERIOD_US      (60000u)    /* NFR7: at most one per 60 ms.  */
#define ECHO_WINDOW_MS      (40u)       /* Longest wait for the echo.    */
#define POLL_MS             (10u)       /* One kernel tick.              */
#define ECHO_MAX_US         (25000u)    /* About 4.3 m.                  */
#define MIN_RANGE_MM        (20u)       /* Datasheet minimum, 2 cm.      */
#define FAULT_PINGS         (5u)        /* Unanswered pings in a row.    */
#define STUCK_US            (300000u)   /* Echo high this long: stuck.   */
#define US_PER_MS           (1000u)

/* Range in mm = echo_us * speed of sound / 2, with sound at 343 m/s. */
#define SOUND_MM_PER_MS     (343u)
#define ROUND_TRIP_SCALE    (2000u)     /* 2 (out and back) x 1000 us/ms.*/

static uint32_t g_last_ping_us = 0u;
static uint32_t g_busy_since   = 0u;
static bool     g_b_busy       = false;
static uint32_t g_unanswered   = 0u;
static bool     g_b_fault      = false;

static void               wait_for_slot(void);
static s5_ranger_status_t ping_and_read(uint32_t * p_echo_us);
static s5_ranger_status_t note_busy(void);
static uint16_t           echo_to_mm(uint32_t echo_us);

/*!
 * @brief Prepare the ranger; the driver must already be initialised.
 */
void
s5_ranger_init (void)
{
    // Pretend the last ping was a full period ago so the first is at once.
    g_last_ping_us = drv_s5_hw_now_us() - PING_PERIOD_US;
    g_b_busy       = false;
    g_unanswered   = 0u;
    g_b_fault      = false;
}

/*!
 * @brief Take one measurement, waiting first if the last ping was less
 *        than 60 ms ago.
 *
 * @param[out] p_reading Status, range and raw echo width.
 *
 * NOTE: Blocks for up to about 100 ms (60 ms pacing plus the echo
 * window). Call it only from obstacle_task.
 */
void
s5_ranger_measure (s5_ranger_reading_t * p_reading)
{
    uint32_t           echo_us = 0u;
    s5_ranger_status_t status  = S5_RANGER_FAULT;

    wait_for_slot();

    if (g_b_fault)
    {
        status = S5_RANGER_FAULT;
    }
    else if (drv_ultrasonic_echo_is_high())
    {
        status = note_busy();
    }
    else
    {
        g_b_busy = false;
        status   = ping_and_read(&echo_us);
    }

    // Count unanswered pings; any real answer resets the count.
    if (S5_RANGER_INVALID == status)
    {
        g_unanswered++;

        if (g_unanswered >= FAULT_PINGS)
        {
            g_b_fault = true;
            status    = S5_RANGER_FAULT;
        }
    }
    else if ((S5_RANGER_OK == status) || (S5_RANGER_CLEAR == status))
    {
        g_unanswered = 0u;
    }
    else
    {
        // BUSY and FAULT leave the count unchanged.
    }

    p_reading->status  = status;
    p_reading->echo_us = echo_us;
    p_reading->dist_mm = (S5_RANGER_OK == status) ? echo_to_mm(echo_us) : 0u;
}

/*!
 * @brief Tell whether a reading saw something closer than a limit.
 *
 * @param[in] p_reading A measurement.
 * @param[in] limit_mm  The limit.
 *
 * @return true only for an OK reading below limit_mm.
 */
bool
s5_ranger_is_within (s5_ranger_reading_t const * p_reading,
                     uint32_t limit_mm)
{
    return ((S5_RANGER_OK == p_reading->status)
            && (p_reading->dist_mm < limit_mm));
}

/*!
 * @brief Tell whether a reading says anything about the scene.
 *
 * @param[in] p_reading A measurement.
 *
 * @return true for OK (something seen) and CLEAR (nothing in range);
 *         false for BUSY, INVALID and FAULT, which should not move any
 *         near or clear counters.
 */
bool
s5_ranger_is_conclusive (s5_ranger_reading_t const * p_reading)
{
    return ((S5_RANGER_OK == p_reading->status)
            || (S5_RANGER_CLEAR == p_reading->status));
}

/*!
 * @brief Clear a latched sensor fault, e.g. after an MQTT RESET.
 */
void
s5_ranger_reset_fault (void)
{
    g_b_fault    = false;
    g_b_busy     = false;
    g_unanswered = 0u;
}

/*!
 * @brief Sleep until 60 ms have passed since the previous ping.
 */
static void
wait_for_slot (void)
{
    uint32_t elapsed_us = drv_s5_hw_now_us() - g_last_ping_us;

    if (elapsed_us < PING_PERIOD_US)
    {
        uint32_t wait_us = PING_PERIOD_US - elapsed_us;

        subsystem5_delay_ms((wait_us + US_PER_MS - 1u) / US_PER_MS);
    }

    g_last_ping_us = drv_s5_hw_now_us();
}

/*!
 * @brief Send a ping and wait for its echo to end.
 *
 * @param[out] p_echo_us Echo width in microseconds.
 *
 * @return OK, CLEAR or INVALID.
 */
static s5_ranger_status_t
ping_and_read (uint32_t * p_echo_us)
{
    s5_ranger_status_t status    = S5_RANGER_INVALID;
    uint32_t           waited_ms = 0u;
    bool               b_done    = false;

    drv_ultrasonic_trigger();

    // Poll each tick, so a near object is read after 10 ms, not 40 ms.
    while ((!b_done) && (waited_ms < ECHO_WINDOW_MS))
    {
        subsystem5_delay_ms(POLL_MS);
        waited_ms += POLL_MS;
        b_done = (0u != drv_ultrasonic_echo_us())
                 && (!drv_ultrasonic_echo_is_high());
    }

    *p_echo_us = drv_ultrasonic_echo_us();

    if ((0u == *p_echo_us) && (!drv_ultrasonic_echo_is_high()))
    {
        status = S5_RANGER_INVALID;     // ECHO never went high.
    }
    else if ((!b_done) || (*p_echo_us > ECHO_MAX_US))
    {
        status = S5_RANGER_CLEAR;       // Still high, or out of range.
    }
    else
    {
        status = S5_RANGER_OK;
    }

    return (status);
}

/*!
 * @brief Handle ECHO still high from an earlier ping.
 *
 * @return BUSY, or FAULT once ECHO has been high for STUCK_US.
 */
static s5_ranger_status_t
note_busy (void)
{
    s5_ranger_status_t status = S5_RANGER_BUSY;
    uint32_t           now_us = drv_s5_hw_now_us();

    if (!g_b_busy)
    {
        g_b_busy     = true;
        g_busy_since = now_us;
    }
    else if ((now_us - g_busy_since) >= STUCK_US)
    {
        g_b_fault = true;
        status    = S5_RANGER_FAULT;
    }
    else
    {
        // Still within the time a slow HC-SR04 clone may hold ECHO.
    }

    return (status);
}

/*!
 * @brief Convert an echo width to a range.
 *
 * @param[in] echo_us Echo width, at most ECHO_MAX_US.
 *
 * @return Range in mm, at least MIN_RANGE_MM.
 */
static uint16_t
echo_to_mm (uint32_t echo_us)
{
    uint32_t dist_mm = ((echo_us * SOUND_MM_PER_MS) + (ROUND_TRIP_SCALE / 2u))
                       / ROUND_TRIP_SCALE;

    if (dist_mm < MIN_RANGE_MM)
    {
        dist_mm = MIN_RANGE_MM;
    }

    // NOTE: The cast is safe: echo_us <= 25000 gives at most 4288 mm.
    return ((uint16_t) dist_mm);
}

/*** end of file ***/
