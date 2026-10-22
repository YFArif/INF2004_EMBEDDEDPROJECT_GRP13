/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2. Built only with: make TEST=1.
 *        Record every run in testing/subsystem2/TEST_LOG.md.
 *
 * @par Automatic (motors stay braked; safe in every test build)
 * - S2-U08: the 1 MHz timer against the kernel clock.
 * - S2-U05: motion_task period, and MF_ESTOP to brake latency, 20 trials
 *   (NFR6).
 * - S2-U07: E-stop latch and CLEAR protocol (Week 6 report 6.3).
 * - S2-U10: longest motion cycle and stack use (NFR11), run last.
 *
 * @par Bench (set the switch to 1, lift the wheels off the ground first)
 * - S2-U01: encoder ticks per wheel revolution, turned by hand (~90 s).
 * - S2-U06: open-loop drive of each motor (~35 s).
 * - S2-U02: closed-loop step 0 -> 250 mm/s on FOLLOW (~3 s).
 *
 * @par Floor (S2_FLOOR_TEST in subsystem2_floor.c)
 * - S2-U11 distance, S2-U03 turns, S2-U12 braking / slow speed / stall.
 *
 * @par
 * S2-U01 and S2-U06 drive the hardware directly. That is allowed only
 * because motion_task leaves the PWM alone while IDLE.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "subsystem2.h"

#if APP_TEST

#define S2_U01_BENCH        (0)     /* Manual, blocks ~90 s at power-up. */
#define S2_U06_BENCH        (0)     /* Spins each motor, ~35 s.          */
#define S2_U02_BENCH        (0)     /* Spins both motors, ~3 s.          */

#define POLL_MS             (10u)   /* One kernel tick.                  */
#define ALIVE_WINDOW_MS     (1000u)
#define ALIVE_MIN           (48u)   /* 50 cycles of 20 ms, +/- jitter.   */
#define ALIVE_MAX           (52u)
#define STATE_WAIT_MS       (200u)
#define SETTLE_MS           (60u)   /* Three motion cycles.              */

#define U05_TRIALS          (20u)
#define U05_LIMIT_US        (20000u)    /* One motion cycle (NFR6).      */
#define U07_STEPS           (4u)

/* S2-U08: 1 MHz timer against the kernel clock. tk_dly_tsk() runs whole
 * 10 ms ticks, so 1000 ms lasts 1000 ... 1010 ms; allow 3 % either way. */
#define U08_WINDOW_MS       (1000u)
#define U08_MIN_US          (970000u)
#define U08_MAX_US          (1040000u)

/* S2-U10 (NFR11): 20 % of the 20 ms period, 75 % of the stack. */
#define U10_CYCLE_LIMIT_US  (4000u)
#define U10_STACK_PCT       (75u)

#define U01_REVS            (10u)
#define U01_WINDOW_S        (40u)

#define U06_DUTY            (400)
#define U06_SPIN_UP_MS      (2000u)
#define U06_COUNT_MS        (1000u)

#define U02_SPEED_MMS       (250)
#define U02_RUN_MS          (1500u)
#define U02_SETTLE_MS       (500u)      /* NFR4: within 0.5 s ...        */
#define U02_BAND_MMS        (25)        /* ... and within +/- 10 %.      */
#define U02_SAMPLE_MS       (20u)

#if TM_CONSOLE_USB_CDC
/* The USB console sends nothing a closed port can receive, so the tests
 * wait for the PC to open it. tm_usb_state() is the kernel console's own
 * link state (lib/libtm/.../usb_console.c); it has no header, hence the
 * declaration here (BARR-C 4.2c deviation, test builds only). */
#define HOST_WAIT_MS        (30000u)
#define HOST_POLL_MS        (100u)
#define HOST_SETTLE_MS      (1000u)     /* Let the terminal settle.      */
#define USB_HOST_OPEN       (3u)        /* Enumerated and DTR asserted.  */

extern UW tm_usb_state(void);
#endif

static void     host_wait(void);
static bool     motion_alive(void);
static bool     state_wait(subsystem2_state_t state, uint32_t timeout_ms);
static void     cmd_send(subsystem2_cmd_type_t type, int16_t value,
                         uint16_t speed_mms);
static void     delay_ms(uint32_t ms);
static bool     s2_u05_estop_latency(void);
static bool     s2_u07_estop_latch(void);
static bool     s2_u08_timer_rate(void);
static bool     s2_u10_resources(void);

#if S2_U01_BENCH
static bool     s2_u01_wheel(drv_encoder_t wheel, char const * p_name);
#endif
#if S2_U06_BENCH
static bool     s2_u06_motor(drv_motor_t motor, char const * p_name);
static uint32_t s2_u06_run(drv_motor_t motor, int32_t duty);
#endif
#if S2_U02_BENCH
static bool     s2_u02_step(void);
#endif

/*!
 * @brief Run subsystem 2's tests (called by usermain() in test builds).
 */
void
subsystem2_test (void)
{
    TEST_INIT();

    host_wait();

    // NOTE: The cast is safe: the core is 0, 1 or UINT32_MAX, printed as
    // 0, 1 or -1.
    tm_printf((UB *) "[TEST] S2 INFO encoder interrupt on core %d"
              " (A4 wants core 1)\n", (int) drv_encoder_core());

    // Every microsecond figure below depends on this one.
    TEST_CHECK("S2-U08", s2_u08_timer_rate(),
               "1 MHz timer runs at 1 MHz against the kernel clock");

    TEST_CHECK("S2-U05", motion_alive(),
               "motion_task runs every 20 ms (48-52 cycles per second)");
    TEST_CHECK("S2-U05", s2_u05_estop_latency(),
               "MF_ESTOP brakes within one cycle, 20 of 20 trials");

    TEST_CHECK("S2-U07", s2_u07_estop_latch(),
               "E-stop latch and CLEAR protocol");

#if S2_U01_BENCH
    TEST_CHECK("S2-U01", s2_u01_wheel(DRV_ENCODER_LEFT, "LEFT"),
               "left wheel counted on GP16, GP17 silent");
    TEST_CHECK("S2-U01", s2_u01_wheel(DRV_ENCODER_RIGHT, "RIGHT"),
               "right wheel counted on GP17, GP16 silent");
#endif

#if S2_U06_BENCH
    TEST_CHECK("S2-U06", s2_u06_motor(DRV_MOTOR_LEFT, "LEFT"),
               "left motor turned the left encoder");
    TEST_CHECK("S2-U06", s2_u06_motor(DRV_MOTOR_RIGHT, "RIGHT"),
               "right motor turned the right encoder");
#endif

#if S2_U02_BENCH
    TEST_CHECK("S2-U02", s2_u02_step(),
               "both wheels within +/-10 % of 250 mm/s after 0.5 s");
#endif

    // Floor tests: S2_FLOOR_TEST in subsystem2_floor.c, 0 = none.
    subsystem2_floor_test();

    // Last, so it covers everything that ran above (driving included).
    TEST_CHECK("S2-U10", s2_u10_resources(),
               "NFR11: longest motion cycle <= 4000 us, stack <= 75 %");

    TEST_SUMMARY("subsystem2");
}

/*!
 * @brief S2-U10: motion_task's longest cycle and deepest stack use so far.
 *
 * @return true if both are inside NFR11 (20 % of 20 ms; 75 % of stack).
 */
static bool
s2_u10_resources (void)
{
    subsystem2_status_t status = {0};
    uint32_t            used   = subsystem2_stack_used();

    subsystem2_status_get(&status);

    // NOTE: The casts are safe: the format string is ASCII and all values
    // are below 20000.
    tm_printf((UB *) "[TEST] S2-U10 INFO longest cycle %d us (limit %d),"
              " stack %d of %d bytes = %d %% (limit 75 %%), %d cycles\n",
              (int) status.cycle_max_us, (int) U10_CYCLE_LIMIT_US,
              (int) used, (int) S2_MOTION_STACK_BYTES,
              (int) ((used * 100u) / S2_MOTION_STACK_BYTES),
              (int) status.cycle_count);

    return ((status.cycle_max_us <= U10_CYCLE_LIMIT_US) &&
            ((used * 100u) <= (S2_MOTION_STACK_BYTES * U10_STACK_PCT)));
}

/*!
 * @brief Wait up to 30 s for the PC to open the serial port, so the S2
 *        results are not sent to a closed port and lost.
 */
static void
host_wait (void)
{
#if TM_CONSOLE_USB_CDC
    uint32_t waited = 0u;

    while ((USB_HOST_OPEN != tm_usb_state()) && (waited < HOST_WAIT_MS))
    {
        delay_ms(HOST_POLL_MS);
        waited += HOST_POLL_MS;
    }

    delay_ms(HOST_SETTLE_MS);
#endif

    // NOTE: The cast is safe: the format string is ASCII.
    tm_printf((UB *) "\n[TEST] subsystem2 tests start\n");
}

/*!
 * @brief S2-U08: count the 1 MHz timer over a 1000 ms kernel delay.
 *
 * @return true if it counted 0.97 ... 1.04 million.
 */
static bool
s2_u08_timer_rate (void)
{
    uint32_t start_us = drv_encoder_now_us();
    uint32_t count    = 0u;

    delay_ms(U08_WINDOW_MS);
    count = drv_encoder_now_us() - start_us;

    // NOTE: The casts are safe: the format string is ASCII, the count is
    // printed as unsigned (%u) and the register is printed in hex.
    tm_printf((UB *) "[TEST] S2-U08 INFO timer counted %u in %d ms;"
              " WATCHDOG TICK at boot 0x%x (12 cycles = 0x..0c)\n",
              (unsigned int) count, (int) U08_WINDOW_MS,
              (unsigned int) drv_encoder_tick_was());

    return ((count >= U08_MIN_US) && (count <= U08_MAX_US));
}

/*!
 * @brief Check that motion_task runs once every 20 ms: about 50 cycles in
 *        1000 ms (not fewer: it is stuck; not more: it is not sleeping).
 *        Also reports the longest cycle so far (NFR11: <= 4000 us).
 *
 * @return true if it ran ALIVE_MIN ... ALIVE_MAX cycles.
 */
static bool
motion_alive (void)
{
    subsystem2_status_t status = {0};
    uint32_t            first  = 0u;
    uint32_t            cycles = 0u;

    subsystem2_status_get(&status);
    first = status.cycle_count;
    delay_ms(ALIVE_WINDOW_MS);
    subsystem2_status_get(&status);
    cycles = status.cycle_count - first;

    // NOTE: The casts are safe: the format string is ASCII; the cycle
    // count is below 1000 and the time below 20000 us.
    tm_printf((UB *) "[TEST] S2-U05 INFO %d motion cycles in %d ms,"
              " longest cycle %d us\n", (int) cycles,
              (int) ALIVE_WINDOW_MS, (int) status.cycle_max_us);

    return ((cycles >= ALIVE_MIN) && (cycles <= ALIVE_MAX));
}

/*!
 * @brief S2-U05: time from setting MF_ESTOP to the brake being applied.
 *
 * @return true if every trial braked within U05_LIMIT_US.
 *
 * NOTE: This is the software part of S2-U05. The hardware part (GPIO
 * toggle and a scope on GP8 - GP11) is still to be done.
 */
static bool
s2_u05_estop_latency (void)
{
    subsystem2_status_t status  = {0};
    uint32_t            trial   = 0u;
    uint32_t            set_us  = 0u;
    uint32_t            delay   = 0u;
    uint32_t            worst   = 0u;
    uint32_t            total   = 0u;
    uint32_t            before  = 0u;
    bool                b_ok    = state_wait(S2_STATE_IDLE, STATE_WAIT_MS);

    for (trial = 0u; (trial < U05_TRIALS) && b_ok; trial++)
    {
        subsystem2_status_get(&status);
        before = status.estop_count;

        set_us = drv_encoder_now_us();
        subsystem2_estop_set();
        b_ok = state_wait(S2_STATE_ESTOP, STATE_WAIT_MS);

        subsystem2_status_get(&status);
        b_ok  = b_ok && ((before + 1u) == status.estop_count);
        delay = status.estop_brake_us - set_us;
        total += delay;
        if (delay > worst)
        {
            worst = delay;
        }

        subsystem2_estop_clear();
        cmd_send(S2_CMD_CLEAR, 0, 0u);
        b_ok = b_ok && state_wait(S2_STATE_IDLE, STATE_WAIT_MS);
    }

    // NOTE: The casts are safe: the format string is ASCII, and every
    // value is below 200000 us, which an int always holds.
    tm_printf((UB *) "[TEST] S2-U05 INFO %d trials: worst %d us, mean %d us"
              " (limit %d us)\n", (int) trial, (int) worst,
              (int) (total / U05_TRIALS), (int) U05_LIMIT_US);

    return (b_ok && (U05_TRIALS == trial) && (worst <= U05_LIMIT_US));
}

/*!
 * @brief S2-U07: commands before CLEAR are discarded, CLEAR is ignored
 *        while MF_ESTOP is still set, and a command sent after an
 *        accepted CLEAR runs. MOVE 0 is used, so no motor turns.
 *
 * @return true if all U07_STEPS steps passed.
 */
static bool
s2_u07_estop_latch (void)
{
    subsystem2_status_t status    = {0};
    uint32_t            discarded = 0u;
    uint32_t            done      = 0u;
    uint32_t            passed    = 0u;

    subsystem2_estop_set();
    if (state_wait(S2_STATE_ESTOP, STATE_WAIT_MS))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO did not enter ESTOP\n");
    }

    subsystem2_status_get(&status);
    discarded = status.discarded_count;
    done      = status.done_count;

    // A command sent while latched must be dropped.
    cmd_send(S2_CMD_MOVE, 100, 0u);
    delay_ms(SETTLE_MS);
    subsystem2_status_get(&status);
    if ((S2_STATE_ESTOP == status.state) &&
        ((discarded + 1u) == status.discarded_count))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO MOVE was not discarded\n");
    }

    // CLEAR while MF_ESTOP is still set must be ignored.
    cmd_send(S2_CMD_CLEAR, 0, 0u);
    delay_ms(SETTLE_MS);
    subsystem2_status_get(&status);
    if ((S2_STATE_ESTOP == status.state) &&
        ((discarded + 2u) == status.discarded_count))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO CLEAR was not ignored\n");
    }

    // Clear the latch, then CLEAR followed by MOVE 0: both must run.
    subsystem2_estop_clear();
    cmd_send(S2_CMD_CLEAR, 0, 0u);
    cmd_send(S2_CMD_MOVE, 0, 0u);
    delay_ms(SETTLE_MS);
    subsystem2_status_get(&status);
    if ((S2_STATE_IDLE == status.state) &&
        ((discarded + 2u) == status.discarded_count) &&
        ((done + 1u) == status.done_count))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO CLEAR + MOVE 0 did not run\n");
    }

    // NOTE: The casts are safe: the format string is ASCII and the
    // counts are 0 ... 4.
    tm_printf((UB *) "[TEST] S2-U07 INFO %d of %d steps passed\n",
              (int) passed, (int) U07_STEPS);

    return (U07_STEPS == passed);
}

#if S2_U01_BENCH
/*!
 * @brief S2-U01: turn one wheel by hand U01_REVS times.
 *
 * @param[in] wheel  The wheel to turn.
 * @param[in] p_name "LEFT" or "RIGHT".
 *
 * @return true if it counted and the other wheel stayed under 1 %.
 */
static bool
s2_u01_wheel (drv_encoder_t wheel, char const * p_name)
{
    drv_encoder_t       other  = DRV_ENCODER_LEFT;
    drv_encoder_count_t start  = {0};
    drv_encoder_count_t now    = {0};
    drv_encoder_count_t o_from = {0};
    drv_encoder_count_t o_now  = {0};
    uint32_t            sec    = 0u;
    uint32_t            mine   = 0u;
    uint32_t            theirs = 0u;

    if (DRV_ENCODER_LEFT == wheel)
    {
        other = DRV_ENCODER_RIGHT;
    }

    subsystem2_step_ready((DRV_ENCODER_LEFT == wheel) ?
                          "S2-U01 turn the LEFT wheel 10 times by hand,"
                          " right wheel still (40 s)" :
                          "S2-U01 turn the RIGHT wheel 10 times by hand,"
                          " left wheel still (40 s)");

    drv_encoder_get(wheel, &start);
    drv_encoder_get(other, &o_from);
    tm_printf((UB *) "[S2-U01] GO: turn the %s wheel %d full revolutions\n",
              p_name, (int) U01_REVS);

    for (sec = 1u; sec <= U01_WINDOW_S; sec++)
    {
        delay_ms(1000u);
        drv_encoder_get(wheel, &now);
        tm_printf((UB *) "[S2-U01] %d s  ticks=%d  raw rise=%d fall=%d\n",
                  (int) sec, (int) (now.ticks - start.ticks),
                  (int) (now.raw_rise - start.raw_rise),
                  (int) (now.raw_fall - start.raw_fall));
    }

    drv_encoder_get(other, &o_now);
    mine   = now.ticks - start.ticks;
    theirs = o_now.ticks - o_from.ticks;

    // NOTE: The casts are safe: counts over 40 s stay below 100000.
    tm_printf((UB *) "[TEST] S2-U01 INFO %s: ticks=%d -> %d.%d per rev,"
              " rejected=%d, shortest gap=%d us\n", p_name, (int) mine,
              (int) (mine / U01_REVS), (int) ((mine * 10u / U01_REVS) % 10u),
              (int) (now.rejected - start.rejected), (int) now.min_gap_us);

    return ((mine > 0u) && ((theirs * 100u) <= mine));
}
#endif /* S2_U01_BENCH */

#if S2_U06_BENCH
/*!
 * @brief S2-U06: forward, reverse, then forward at double duty.
 *        Direction is judged by eye.
 *
 * @param[in] motor  The motor.
 * @param[in] p_name "LEFT" or "RIGHT".
 *
 * @return true if every run counted and double duty was faster.
 */
static bool
s2_u06_motor (drv_motor_t motor, char const * p_name)
{
    uint32_t slow = 0u;
    uint32_t back = 0u;
    uint32_t fast = 0u;
    bool     b_l  = (DRV_MOTOR_LEFT == motor);

    (void) p_name;

    // Each phase waits for Button 1, so the tester answers the PC's
    // questions while nothing moves, and watches the wheel when it does.
    subsystem2_step_ready(b_l ? "S2-U06 LEFT FORWARD (3 s): WATCH the left"
                                " wheel; does it drive the robot forward?"
                              : "S2-U06 RIGHT FORWARD (3 s): WATCH the right"
                                " wheel; does it drive the robot forward?");
    slow = s2_u06_run(motor, U06_DUTY);

    subsystem2_step_ready(b_l ? "S2-U06 LEFT REVERSE (3 s): WATCH the left"
                                " wheel; the opposite way?"
                              : "S2-U06 RIGHT REVERSE (3 s): WATCH the right"
                                " wheel; the opposite way?");
    back = s2_u06_run(motor, -U06_DUTY);

    subsystem2_step_ready(b_l ? "S2-U06 LEFT FORWARD FAST (3 s): WATCH the"
                                " left wheel; forward and faster?"
                              : "S2-U06 RIGHT FORWARD FAST (3 s): WATCH the"
                                " right wheel; forward and faster?");
    fast = s2_u06_run(motor, 2 * U06_DUTY);

    drv_motor_brake();

    return ((slow > 0u) && (back > 0u) && (fast > slow));
}

/*!
 * @brief Drive one motor, let it settle, then count its ticks for 1 s.
 *
 * @param[in] motor The motor (its encoder has the same index).
 * @param[in] duty  Permille, signed.
 *
 * @return Ticks per second; 0 if the other wheel also moved.
 */
static uint32_t
s2_u06_run (drv_motor_t motor, int32_t duty)
{
    drv_encoder_t       mine   = DRV_ENCODER_LEFT;
    drv_encoder_t       other  = DRV_ENCODER_RIGHT;
    drv_encoder_count_t m_from = {0};
    drv_encoder_count_t m_to   = {0};
    drv_encoder_count_t o_from = {0};
    drv_encoder_count_t o_to   = {0};
    uint32_t            ticks  = 0u;
    uint32_t            tpr    = (uint32_t) S2_TICKS_PER_REV_L;

    if (DRV_MOTOR_RIGHT == motor)
    {
        mine  = DRV_ENCODER_RIGHT;
        other = DRV_ENCODER_LEFT;
        tpr   = (uint32_t) S2_TICKS_PER_REV_R;
    }

    drv_motor_set(motor, duty);
    delay_ms(U06_SPIN_UP_MS);
    drv_encoder_get(mine, &m_from);
    drv_encoder_get(other, &o_from);
    delay_ms(U06_COUNT_MS);
    drv_encoder_get(mine, &m_to);
    drv_encoder_get(other, &o_to);
    drv_motor_set(motor, 0);
    delay_ms(1000u);

    ticks = m_to.ticks - m_from.ticks;

    // NOTE: The casts are safe: the format string is ASCII; ticks per
    // second stay below 5000 and the speed below 2000 mm/s. 1979 is the
    // wheel circumference in tenths of a millimetre.
    tm_printf((UB *) "[S2-U06]   duty %d: %d ticks/s = %d mm/s,"
              " other wheel %d ticks\n", (int) duty, (int) ticks,
              (int) ((ticks * 1979u) / (tpr * 10u)),
              (int) (o_to.ticks - o_from.ticks));

    if (((o_to.ticks - o_from.ticks) * 100u) > ticks)
    {
        ticks = 0u;
    }

    return (ticks);
}
#endif /* S2_U06_BENCH */

#if S2_U02_BENCH
/*!
 * @brief S2-U02: FOLLOW at 250 mm/s with no steering; log both wheel
 *        speeds every 20 ms and check them after 0.5 s.
 *
 * @return true if both stayed within +/- U02_BAND_MMS after settling.
 */
static bool
s2_u02_step (void)
{
    subsystem2_odom_t   odom    = {0};
    subsystem2_status_t status  = {0};
    uint32_t            elapsed = 0u;
    bool                b_ok    = true;

    subsystem2_step_ready("S2-U02 both wheels run 1.5 s at 250 mm/s: WATCH"
                          " that BOTH turn the way that drives the robot"
                          " forward");
    tm_printf((UB *) "[S2-U02] t_ms,speed_l,speed_r,duty_l,duty_r\n");

    subsystem2_steer_set(0);
    cmd_send(S2_CMD_FOLLOW, 0, (uint16_t) U02_SPEED_MMS);

    for (elapsed = U02_SAMPLE_MS; elapsed <= U02_RUN_MS;
         elapsed += U02_SAMPLE_MS)
    {
        delay_ms(U02_SAMPLE_MS);
        subsystem2_odom_get(&odom);
        subsystem2_status_get(&status);

        // NOTE: The casts are safe: the format string is ASCII and all
        // values are below 2000.
        tm_printf((UB *) "[S2-U02] %d,%d,%d,%d,%d\n", (int) elapsed,
                  (int) odom.speed_l_mms, (int) odom.speed_r_mms,
                  (int) status.duty_l, (int) status.duty_r);

        if ((elapsed >= U02_SETTLE_MS) &&
            (((odom.speed_l_mms - U02_SPEED_MMS) > U02_BAND_MMS) ||
             ((U02_SPEED_MMS - odom.speed_l_mms) > U02_BAND_MMS) ||
             ((odom.speed_r_mms - U02_SPEED_MMS) > U02_BAND_MMS) ||
             ((U02_SPEED_MMS - odom.speed_r_mms) > U02_BAND_MMS)))
        {
            b_ok = false;
        }
    }

    cmd_send(S2_CMD_STOP, 0, 0u);

    return (b_ok && (S2_STATE_FOLLOWING == status.state));
}
#endif /* S2_U02_BENCH */

/*!
 * @brief Wait until motion_task reports a state.
 *
 * @param[in] state      The state.
 * @param[in] timeout_ms Longest wait.
 *
 * @return true if the state was reached.
 */
static bool
state_wait (subsystem2_state_t state, uint32_t timeout_ms)
{
    subsystem2_status_t status = {0};
    uint32_t            waited = 0u;

    subsystem2_status_get(&status);

    while ((state != status.state) && (waited < timeout_ms))
    {
        delay_ms(POLL_MS);
        waited += POLL_MS;
        subsystem2_status_get(&status);
    }

    return (state == status.state);
}

/*!
 * @brief Send one command to motion_task.
 *
 * @param[in] type      Command type.
 * @param[in] value     mm or degrees.
 * @param[in] speed_mms 0 = default.
 */
static void
cmd_send (subsystem2_cmd_type_t type, int16_t value, uint16_t speed_mms)
{
    // NOTE: The cast is safe: command types are 0 ... 4.
    subsystem2_cmd_t cmd =
    {
        .type      = (uint8_t) type,
        .value     = value,
        .speed_mms = speed_mms
    };

    CHECK(subsystem2_motion_send(&cmd), "subsystem2_motion_send");
}

/*!
 * @brief Sleep, letting the other tasks run.
 *
 * @param[in] ms Milliseconds.
 */
static void
delay_ms (uint32_t ms)
{
    // NOTE: The cast is safe: test delays are at most a few seconds.
    CHECK(tk_dly_tsk((RELTIM) ms), "tk_dly_tsk");
}

#endif /* APP_TEST */

/*** end of file ***/
