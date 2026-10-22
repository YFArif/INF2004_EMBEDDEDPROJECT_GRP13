/** @file motion_task.c
 *
 * @brief motion_task: per-wheel PID speed control, encoder-based MOVE and
 *        TURN, line FOLLOW, the emergency-stop latch, stall detection and
 *        odometry (Week 6 report 6.3; FR6, FR7, FR15, NFR4 - NFR6).
 *
 * @par Every 20 ms cycle
 * 1. E-stop first: if MF_ESTOP is set, brake and enter ESTOP_BRAKED.
 *    MF_ESTOP also wakes the task at once, so braking does not wait for
 *    the next period (NFR6).
 * 2. Measure both wheels: edge-timed speed and signed distance.
 * 3. Take every waiting command. In ESTOP_BRAKED all of them are
 *    discarded, except a CLEAR sent after MF_ESTOP was cleared; the
 *    commands behind that CLEAR then run in order.
 * 4. Work out each wheel's speed set-point for the state, ramp it, run the
 *    PID and set the PWM duty. Check for a stall.
 * 5. Publish the odometry and status.
 *
 * @par States (subsystem2_state_t)
 * - IDLE: both motors braked once on entry. The PWM is not rewritten
 *   while idle, so the S2 bench tests may drive the motors directly.
 * - FOLLOWING: wheel speeds = speed -/+ steer_sp from line_task.
 * - MOVING: both wheels travel value mm; the speed drops towards the end
 *   and the two wheels are kept level. Then EV_MOTION_DONE and IDLE.
 * - TURNING: on the spot; each wheel travels theta * pi * track / 360,
 *   left wheel backward for a left (positive) turn. Then EV_MOTION_DONE.
 * - ESTOP: braked every cycle until MF_ESTOP is cleared and CLEAR comes.
 *   A stall (|duty| >= 40 % and no tick for 300 ms) also ends here, after
 *   setting MF_ESTOP itself and reporting S5_FAULT_STALL with EV_FAULT.
 *
 * @par Direction
 * The encoders have one phase and cannot tell direction, so each new tick
 * is counted in the direction of the duty last applied to that wheel.
 *
 * @par
 * Gains and limits marked "preliminary" are tuned in S2-U02 and S2-U03.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem5/subsystem5.h"   /* EV_MOTION_DONE, S5 faults.  */
#include "drv_encoder.h"
#include "drv_motor.h"
#include "motion_task.h"
#include "subsystem2.h"

/* Wheels, indexed the same way as the motor and encoder channels. */
#define WHEEL_LEFT              (0u)
#define WHEEL_RIGHT             (1u)
#define WHEEL_COUNT             (2u)

/* Set-point shaping (preliminary until S2-U02 / S2-U03). */
#define ACCEL_MMS_PER_CYCLE     (30)    /* 1.5 m/s^2 ramp.                */
#define SPEED_FINAL_MMS         (60)    /* Speed at the end of MOVE/TURN. */
#define DECEL_PER_S             (4)     /* Limit = final + 4/s x left mm. */
#define SYNC_PER_S              (3)     /* Left/right distance correction.*/
#define TURN_MAX_DEG            (720)   /* Larger TURN angles are clipped.*/

/* MOVE / TURN brake this long before the target, at the current speed,
 * because the wheels roll on while braking and the decision is made only
 * every 20 ms. Preliminary from S2-U09; tuned on the floor in S2-U03. */
#ifndef STOP_LEAD_MS
#define STOP_LEAD_MS            (40)
#endif

/* PID gains as fractions (preliminary until S2-U02). Feed-forward: S2-U06
 * gave about 0.8 mm/s per permille with the wheels off the ground, so
 * 1.25 permille per mm/s, and the integral term covers the load. The
 * #ifndef lets the host test (S2-U09) try other gains. */
#define FF_NUM                  (5)
#define FF_DEN                  (4)
#ifndef KP_NUM
#define KP_NUM                  (1)     /* 1.0 permille per mm/s.         */
#define KP_DEN                  (1)
#endif
#ifndef KI_NUM
#define KI_NUM                  (1)     /* 10 permille per mm/s per       */
#define KI_DEN                  (5)     /* second = 0.2 per 20 ms cycle.  */
#endif
#define KD_NUM                  (0)     /* D on measurement, off for now. */
#define KD_DEN                  (1)
#define I_TERM_MAX              (400)   /* Permille the I term may add.   */
#define INTEGRAL_MAX            ((I_TERM_MAX * KI_DEN) / KI_NUM)
#define DUTY_LIMIT              (900)

/* Stall (Week 6 report 6.3). */
#define STALL_DUTY              (400)
#define STALL_CYCLES            (15u)   /* 300 ms.                        */

/* Speed measurement. */
#define US_PER_MS               (1000u)
#define CYCLE_US                (S2_MOTION_PERIOD_MS * US_PER_MS)
#define STALE_TICK_US           (2u * CYCLE_US)
#define SPEED_STOPPED_US        (100000u)   /* No tick for 100 ms = 0.    */

/* Geometry, in micrometres. */
#define UM_PER_MM               (1000)
#define PI_X1E6                 (3141593)
#define PI_X1E4                 (31416u)
#define WHEEL_CIRC_UM           ((S2_WHEEL_DIA_MM * PI_X1E6) / UM_PER_MM)

/* Heading: 2 pi x 112 mm = 703717 um of wheel difference per full turn;
 * 1 rad = 5729.58 cdeg, kept as x10 to work in 0.1 mm. */
#define FULL_TURN_UM            ((2 * S2_TRACK_MM * PI_X1E6) / UM_PER_MM)
#define UM_PER_DMM              (100)
#define CDEG_PER_RAD_X10        (57296)
#define CDEG_HALF_TURN          (18000)
#define CDEG_FULL_TURN          (36000)

/** State of one wheel. */
typedef struct
{
    drv_encoder_t encoder;
    drv_motor_t   motor;
    int32_t       ticks_per_rev;
    uint32_t      ticks_seen;       /* Encoder ticks at the last cycle.   */
    uint32_t      tick_time_us;     /* Time of the latest tick seen.      */
    uint32_t      new_ticks;        /* Ticks during this cycle.           */
    int32_t       ticks_signed;     /* Signed ticks since power-up.       */
    int32_t       direction;        /* +1 / -1, applied to new ticks.     */
    int32_t       speed_mms;        /* Measured, signed.                  */
    int32_t       speed_prev_mms;   /* Previous cycle, for the D term.    */
    int32_t       target_mms;       /* Ramped set-point, signed.          */
    int32_t       integral;         /* Sum of errors, (mm/s) x cycles.    */
    int32_t       duty;             /* Output, permille, signed.          */
    uint32_t      still_cycles;     /* Cycles at stall duty with no tick. */
    int32_t       start_um;         /* Distance when MOVE / TURN started. */
    int32_t       travel_sign;      /* +1 / -1: MOVE / TURN direction.    */
} wheel_t;

/** Everything motion_task keeps between cycles. */
typedef struct
{
    wheel_t             wheel[WHEEL_COUNT];
    subsystem2_state_t  state;
    int32_t             speed_mms;  /* Speed of the active command.       */
    int32_t             goal_um;    /* MOVE / TURN travel per wheel.      */
    subsystem2_status_t status;
} motion_t;

static motion_t g_motion =
{
    .wheel =
    {
        { .encoder = DRV_ENCODER_LEFT,  .motor = DRV_MOTOR_LEFT,
          .ticks_per_rev = S2_TICKS_PER_REV_L, .direction = 1 },
        { .encoder = DRV_ENCODER_RIGHT, .motor = DRV_MOTOR_RIGHT,
          .ticks_per_rev = S2_TICKS_PER_REV_R, .direction = 1 }
    },
    .state = S2_STATE_IDLE
};

static void    motion_setup(void);
static void    motion_cycle(void);
static void    commands_process(void);
static void    command_start(subsystem2_cmd_t const * p_cmd);
static void    travel_begin(subsystem2_state_t state, int32_t goal_um,
                            int32_t sign_left, int32_t sign_right,
                            int32_t speed_mms);
static void    control_run(void);
static bool    travel_targets(int32_t * p_want_left,
                              int32_t * p_want_right);
static void    wheel_measure(wheel_t * p_wheel, uint32_t now_us);
static void    wheel_drive(wheel_t * p_wheel, int32_t want_mms);
static bool    wheel_stalled(wheel_t * p_wheel);
static void    wheel_reset(wheel_t * p_wheel);
static int32_t wheel_dist_um(wheel_t const * p_wheel);
static void    enter_idle(void);
static void    enter_estop(void);
static void    stall_handle(void);
static void    publish(uint32_t start_us);
static int32_t heading_cdeg(int32_t diff_um);
static int32_t turn_arc_um(int32_t angle_deg);
static int32_t speed_pick(uint16_t requested_mms, int32_t default_mms);
static int32_t clamp(int32_t value, int32_t low, int32_t high);
static int32_t magnitude(int32_t value);

/*!
 * @brief motion_task entry (priority 9, core 1, every 20 ms).
 *
 * @param[in] stacd   Start code (unused).
 * @param[in] p_exinf Extended information (unused).
 */
void
motion_task (INT stacd, void * p_exinf)
{
    (void) stacd;
    (void) p_exinf;

    motion_setup();

    for (;;)
    {
        // In ESTOP the latch stays set, so only the tick may wake us.
        subsystem2_cycle_wait(S2_STATE_ESTOP != g_motion.state);
        motion_cycle();
    }
}

/*!
 * @brief Set up the encoders on this core (core 1) and the wheel state.
 */
static void
motion_setup (void)
{
    drv_encoder_count_t count  = {0};
    ER                  result = drv_encoder_init();
    uint32_t            idx    = 0u;
    int                 core   = 0;

    if (E_OK != result)
    {
        // Without encoders nothing can be controlled: stay braked.
        CHECK(result, "drv_encoder_init");
        subsystem2_estop_set();
    }

    for (idx = 0u; idx < WHEEL_COUNT; idx++)
    {
        drv_encoder_get(g_motion.wheel[idx].encoder, &count);
        g_motion.wheel[idx].ticks_seen   = count.ticks;
        g_motion.wheel[idx].tick_time_us = count.tick_time_us;
    }

#if TK_SUPPORT_SMP
    // NOTE: The cast is safe: tk_get_prc() returns 1 or 2 here.
    core = (int) tk_get_prc() - 1;
#endif

    // NOTE: The casts are safe: the format string is ASCII, and the
    // period is 20, which an int always holds.
    tm_printf((UB *) "motion_task: core %d, period %d ms\n", core,
              (int) S2_MOTION_PERIOD_MS);
}

/*!
 * @brief One control cycle.
 */
static void
motion_cycle (void)
{
    uint32_t start_us = drv_encoder_now_us();

    // 1. Emergency stop before anything else (NFR6).
    if ((S2_STATE_ESTOP != g_motion.state) && subsystem2_estop_is_set())
    {
        enter_estop();
    }

    // 2. Wheel speeds and distances.
    wheel_measure(&g_motion.wheel[WHEEL_LEFT], start_us);
    wheel_measure(&g_motion.wheel[WHEEL_RIGHT], start_us);

    // 3. Commands, 4. control, 5. publish.
    commands_process();
    control_run();
    publish(start_us);
}

/*!
 * @brief Take every waiting command, in order.
 */
static void
commands_process (void)
{
    subsystem2_cmd_t cmd = {0};

    while (subsystem2_cmd_receive(&cmd))
    {
        if (S2_STATE_ESTOP == g_motion.state)
        {
            if ((S2_CMD_CLEAR == cmd.type) && !subsystem2_estop_is_set())
            {
                enter_idle();
            }
            else
            {
                // Sent before the CLEAR, or CLEAR while still latched.
                g_motion.status.discarded_count++;
            }
        }
        else
        {
            command_start(&cmd);
        }
    }
}

/*!
 * @brief Start one command (not in ESTOP).
 *
 * @param[in] p_cmd The command.
 */
static void
command_start (subsystem2_cmd_t const * p_cmd)
{
    int32_t sign  = 1;
    int32_t speed = 0;

    switch (p_cmd->type)
    {
        case S2_CMD_STOP:
            enter_idle();
        break;

        case S2_CMD_FOLLOW:
            // A new FOLLOW while following only changes the speed, e.g.
            // half speed near an obstacle; the ramp keeps it smooth.
            g_motion.speed_mms = speed_pick(p_cmd->speed_mms,
                                            S2_SPEED_CRUISE_MMS);
            g_motion.state     = S2_STATE_FOLLOWING;
        break;

        case S2_CMD_MOVE:
            if (p_cmd->value < 0)
            {
                sign = -1;
            }
            speed = speed_pick(p_cmd->speed_mms, S2_SPEED_CRUISE_MMS);
            travel_begin(S2_STATE_MOVING,
                         magnitude(p_cmd->value) * UM_PER_MM,
                         sign, sign, speed);
        break;

        case S2_CMD_TURN:
            if (p_cmd->value < 0)
            {
                sign = -1;
            }
            speed = speed_pick(p_cmd->speed_mms, S2_SPEED_TURN_MMS);
            travel_begin(S2_STATE_TURNING, turn_arc_um(p_cmd->value),
                         -sign, sign, speed);
        break;

        case S2_CMD_CLEAR:
            // Not braked for an E-stop: nothing to clear.
        break;

        default:
            g_motion.status.discarded_count++;
        break;
    }
}

/*!
 * @brief Start a MOVE or TURN from where the wheels are now.
 *
 * @param[in] state      S2_STATE_MOVING or S2_STATE_TURNING.
 * @param[in] goal_um    Travel of each wheel, micrometres, >= 0.
 * @param[in] sign_left  +1 forward / -1 backward, left wheel.
 * @param[in] sign_right +1 forward / -1 backward, right wheel.
 * @param[in] speed_mms  Cruise speed.
 */
static void
travel_begin (subsystem2_state_t state, int32_t goal_um, int32_t sign_left,
              int32_t sign_right, int32_t speed_mms)
{
    wheel_t * p_left  = &g_motion.wheel[WHEEL_LEFT];
    wheel_t * p_right = &g_motion.wheel[WHEEL_RIGHT];

    p_left->start_um     = wheel_dist_um(p_left);
    p_left->travel_sign  = sign_left;
    p_right->start_um    = wheel_dist_um(p_right);
    p_right->travel_sign = sign_right;

    g_motion.goal_um   = goal_um;
    g_motion.speed_mms = speed_mms;
    g_motion.state     = state;
}

/*!
 * @brief Set-points, PID and PWM for the current state.
 */
static void
control_run (void)
{
    int32_t want_left  = 0;
    int32_t want_right = 0;
    int32_t steer_mms  = 0;
    bool    b_drive    = false;
    bool    b_stall_l  = false;
    bool    b_stall_r  = false;

    switch (g_motion.state)
    {
        case S2_STATE_FOLLOWING:
            steer_mms  = subsystem2_steer_get();
            want_left  = clamp(g_motion.speed_mms - steer_mms,
                               -S2_SPEED_MAX_MMS, S2_SPEED_MAX_MMS);
            want_right = clamp(g_motion.speed_mms + steer_mms,
                               -S2_SPEED_MAX_MMS, S2_SPEED_MAX_MMS);
            b_drive    = true;
        break;

        case S2_STATE_MOVING:
        case S2_STATE_TURNING:
            b_drive = travel_targets(&want_left, &want_right);
        break;

        case S2_STATE_ESTOP:
            // Re-assert the brake every cycle while latched.
            drv_motor_brake();
        break;

        case S2_STATE_IDLE:
        default:
            // Braked on entry; the PWM is left alone while idle.
        break;
    }

    if (b_drive)
    {
        wheel_drive(&g_motion.wheel[WHEEL_LEFT], want_left);
        wheel_drive(&g_motion.wheel[WHEEL_RIGHT], want_right);

        // Both checks run every cycle, so both counters stay current.
        b_stall_l = wheel_stalled(&g_motion.wheel[WHEEL_LEFT]);
        b_stall_r = wheel_stalled(&g_motion.wheel[WHEEL_RIGHT]);

        if (b_stall_l || b_stall_r)
        {
            stall_handle();
        }
    }
}

/*!
 * @brief Wheel set-points for MOVE / TURN, or finish it.
 *
 * @param[out] p_want_left  Left wheel speed, signed.
 * @param[out] p_want_right Right wheel speed, signed.
 *
 * @return true while still travelling; false once finished (now IDLE).
 */
static bool
travel_targets (int32_t * p_want_left, int32_t * p_want_right)
{
    wheel_t * p_left    = &g_motion.wheel[WHEEL_LEFT];
    wheel_t * p_right   = &g_motion.wheel[WHEEL_RIGHT];
    int32_t   prog_l    = (wheel_dist_um(p_left) - p_left->start_um) *
                          p_left->travel_sign;
    int32_t   prog_r    = (wheel_dist_um(p_right) - p_right->start_um) *
                          p_right->travel_sign;
    int32_t   remaining = g_motion.goal_um - ((prog_l / 2) + (prog_r / 2));
    int32_t   along_mms = ((p_left->speed_mms * p_left->travel_sign) +
                           (p_right->speed_mms * p_right->travel_sign)) / 2;
    int32_t   speed     = 0;
    int32_t   sync      = 0;
    bool      b_running = true;

    // mm/s x ms = um: the travel while the brake takes hold.
    if (remaining <= (magnitude(along_mms) * STOP_LEAD_MS))
    {
        enter_idle();
        g_motion.status.done_count++;

#if defined(EV_MOTION_DONE)
        subsystem5_mission_signal(EV_MOTION_DONE);
#endif
        b_running = false;
    }
    else
    {
        // Slow down near the end, so the stop is short and repeatable.
        speed = SPEED_FINAL_MMS + ((DECEL_PER_S * remaining) / UM_PER_MM);
        if (speed > g_motion.speed_mms)
        {
            speed = g_motion.speed_mms;
        }

        // Keep both wheels level: a wheel that is ahead slows down.
        sync = (SYNC_PER_S * (prog_l - prog_r)) / UM_PER_MM;
        sync = clamp(sync, -(speed / 2), speed / 2);

        *p_want_left  = p_left->travel_sign * (speed - sync);
        *p_want_right = p_right->travel_sign * (speed + sync);
    }

    return (b_running);
}

/*!
 * @brief Update one wheel's tick count, distance and speed.
 *
 * @param[in,out] p_wheel The wheel.
 * @param[in]     now_us  Timer value at the start of this cycle.
 */
static void
wheel_measure (wheel_t * p_wheel, uint32_t now_us)
{
    drv_encoder_count_t count       = {0};
    uint32_t            tpr         = 0u;
    uint32_t            um_per_tick = 0u;
    uint32_t            dt_us       = 0u;
    uint32_t            speed       = 0u;
    uint32_t            bound       = 0u;

    // NOTE: The casts are safe: ticks per revolution are 629 / 639.
    tpr         = (uint32_t) p_wheel->ticks_per_rev;
    um_per_tick = (uint32_t) WHEEL_CIRC_UM / tpr;

    drv_encoder_get(p_wheel->encoder, &count);
    p_wheel->new_ticks = count.ticks - p_wheel->ticks_seen;

    if (0u != p_wheel->new_ticks)
    {
        // Edge-timed: the new ticks span the time from the previous last
        // tick to this last tick. After a stop that span is meaningless,
        // so use one cycle instead.
        dt_us = count.tick_time_us - p_wheel->tick_time_us;
        if ((0u == dt_us) || (dt_us > STALE_TICK_US))
        {
            dt_us = CYCLE_US;
        }

        // At most ~50 ticks per cycle: 50 x 197920 and 15500 x 1000 both
        // fit in 32 bits.
        speed = (((p_wheel->new_ticks * (uint32_t) WHEEL_CIRC_UM) / tpr) *
                 US_PER_MS) / dt_us;
        p_wheel->tick_time_us = count.tick_time_us;
    }
    else
    {
        // No tick yet: the speed is at most one tick over the time since
        // the last one, and zero after SPEED_STOPPED_US.
        dt_us = now_us - p_wheel->tick_time_us;
        if ((0u != dt_us) && (dt_us < SPEED_STOPPED_US))
        {
            bound = (um_per_tick * US_PER_MS) / dt_us;

            // NOTE: The cast is safe: speeds are below 2000 mm/s.
            speed = (uint32_t) magnitude(p_wheel->speed_mms);
            if (bound < speed)
            {
                speed = bound;
            }
        }
    }

    p_wheel->ticks_seen = count.ticks;

    // NOTE: The casts are safe: new_ticks is at most ~50 per cycle and
    // speed below 2000 mm/s, both far below INT32_MAX.
    p_wheel->ticks_signed += p_wheel->direction * (int32_t) p_wheel->new_ticks;
    p_wheel->speed_mms     = p_wheel->direction * (int32_t) speed;
}

/*!
 * @brief Ramp the set-point, run the PID and drive the motor.
 *
 * @param[in,out] p_wheel  The wheel.
 * @param[in]     want_mms Wanted speed, signed.
 */
static void
wheel_drive (wheel_t * p_wheel, int32_t want_mms)
{
    int32_t error    = 0;
    int32_t integral = p_wheel->integral;
    int32_t raw      = 0;
    int32_t duty     = 0;
    int32_t step     = clamp(want_mms - p_wheel->target_mms,
                             -ACCEL_MMS_PER_CYCLE, ACCEL_MMS_PER_CYCLE);
    bool    b_ramp   = (step != (want_mms - p_wheel->target_mms));

    p_wheel->target_mms += step;

    if (0 == p_wheel->target_mms)
    {
        p_wheel->integral = 0;
    }
    else
    {
        error = p_wheel->target_mms - p_wheel->speed_mms;

        // While the set-point is still ramping the wheel always lags it;
        // integrating that lag only winds up an overshoot.
        if (!b_ramp)
        {
            integral = clamp(integral + error, -INTEGRAL_MAX, INTEGRAL_MAX);
        }

        raw = ((p_wheel->target_mms * FF_NUM) / FF_DEN) +
              ((error * KP_NUM) / KP_DEN) +
              ((integral * KI_NUM) / KI_DEN) -
              (((p_wheel->speed_mms - p_wheel->speed_prev_mms) * KD_NUM) /
               KD_DEN);
        duty = clamp(raw, -DUTY_LIMIT, DUTY_LIMIT);

        // Anti-windup: keep the new integral unless the output is
        // saturated and the error would push it further.
        if ((duty == raw) || ((raw > 0) != (error > 0)))
        {
            p_wheel->integral = integral;
        }
    }

    if (duty > 0)
    {
        p_wheel->direction = 1;
    }
    else if (duty < 0)
    {
        p_wheel->direction = -1;
    }
    else
    {
        // Braked: ticks while stopping still go the last way.
    }

    p_wheel->speed_prev_mms = p_wheel->speed_mms;
    p_wheel->duty           = duty;
    drv_motor_set(p_wheel->motor, duty);
}

/*!
 * @brief Count cycles with a high duty and no tick.
 *
 * @param[in,out] p_wheel The wheel.
 *
 * @return true once that has lasted STALL_CYCLES (300 ms).
 */
static bool
wheel_stalled (wheel_t * p_wheel)
{
    if ((magnitude(p_wheel->duty) >= STALL_DUTY) &&
        (0u == p_wheel->new_ticks))
    {
        p_wheel->still_cycles++;
    }
    else
    {
        p_wheel->still_cycles = 0u;
    }

    return (p_wheel->still_cycles >= STALL_CYCLES);
}

/*!
 * @brief Forget the controller state of one wheel (after braking).
 *
 * @param[in,out] p_wheel The wheel.
 */
static void
wheel_reset (wheel_t * p_wheel)
{
    p_wheel->target_mms   = 0;
    p_wheel->integral     = 0;
    p_wheel->duty         = 0;
    p_wheel->still_cycles = 0u;
}

/*!
 * @brief Signed distance one wheel has travelled since power-up.
 *
 * @param[in] p_wheel The wheel.
 *
 * @return Micrometres (overflows only after ~2 km).
 */
static int32_t
wheel_dist_um (wheel_t const * p_wheel)
{
    // Whole revolutions and the rest separately, so 32 bits are enough.
    int32_t revs = p_wheel->ticks_signed / p_wheel->ticks_per_rev;
    int32_t part = p_wheel->ticks_signed % p_wheel->ticks_per_rev;

    return ((revs * WHEEL_CIRC_UM) +
            ((part * WHEEL_CIRC_UM) / p_wheel->ticks_per_rev));
}

/*!
 * @brief Brake both motors and enter IDLE.
 */
static void
enter_idle (void)
{
    drv_motor_brake();
    wheel_reset(&g_motion.wheel[WHEEL_LEFT]);
    wheel_reset(&g_motion.wheel[WHEEL_RIGHT]);
    g_motion.state = S2_STATE_IDLE;
}

/*!
 * @brief Brake both motors and latch ESTOP.
 */
static void
enter_estop (void)
{
    drv_motor_brake();
    g_motion.status.estop_brake_us = drv_encoder_now_us();
    wheel_reset(&g_motion.wheel[WHEEL_LEFT]);
    wheel_reset(&g_motion.wheel[WHEEL_RIGHT]);
    g_motion.state = S2_STATE_ESTOP;
    g_motion.status.estop_count++;
}

/*!
 * @brief A wheel is stalled: brake, latch MF_ESTOP and tell S5.
 */
static void
stall_handle (void)
{
    enter_estop();
    subsystem2_estop_set();
    g_motion.status.stall_count++;

#if defined(S5_FAULT_STALL) && defined(EV_FAULT)
    subsystem5_fault_set(S5_FAULT_STALL);
    subsystem5_mission_signal(EV_FAULT);
#endif

    // NOTE: The cast is safe: the format string is ASCII.
    tm_printf((UB *) "motion_task: STALL, braked, MF_ESTOP set\n");
}

/*!
 * @brief Publish odometry and status for the other subsystems.
 *
 * @param[in] start_us Timer value at the start of this cycle.
 */
static void
publish (uint32_t start_us)
{
    wheel_t const *   p_left  = &g_motion.wheel[WHEEL_LEFT];
    wheel_t const *   p_right = &g_motion.wheel[WHEEL_RIGHT];
    subsystem2_odom_t odom    = {0};
    uint32_t          used_us = 0u;

    odom.count_l      = p_left->ticks_seen;
    odom.count_r      = p_right->ticks_seen;
    odom.dist_l_mm    = wheel_dist_um(p_left) / UM_PER_MM;
    odom.dist_r_mm    = wheel_dist_um(p_right) / UM_PER_MM;
    odom.dist_mm      = ((wheel_dist_um(p_left) / 2) +
                         (wheel_dist_um(p_right) / 2)) / UM_PER_MM;
    odom.speed_l_mms  = p_left->speed_mms;
    odom.speed_r_mms  = p_right->speed_mms;
    odom.speed_mms    = (p_left->speed_mms + p_right->speed_mms) / 2;
    odom.heading_cdeg = heading_cdeg(wheel_dist_um(p_right) -
                                     wheel_dist_um(p_left));

    g_motion.status.state        = g_motion.state;
    g_motion.status.duty_l       = p_left->duty;
    g_motion.status.duty_r       = p_right->duty;
    g_motion.status.target_l_mms = p_left->target_mms;
    g_motion.status.target_r_mms = p_right->target_mms;
    g_motion.status.cycle_count++;

    used_us = drv_encoder_now_us() - start_us;
    if (used_us > g_motion.status.cycle_max_us)
    {
        g_motion.status.cycle_max_us = used_us;
    }

    subsystem2_odom_put(&odom, &g_motion.status);
}

/*!
 * @brief Heading from the difference of the wheel distances.
 *
 * @param[in] diff_um Right wheel minus left wheel, micrometres.
 *
 * @return Centidegrees, + = left, -17999 ... 18000.
 */
static int32_t
heading_cdeg (int32_t diff_um)
{
    // One full turn on the spot is 2 pi x track of difference. Reduce to
    // one turn first, so the multiplication below cannot overflow:
    // 7037 x 57296 < 2^31.
    int32_t turn_dmm = (diff_um % FULL_TURN_UM) / UM_PER_DMM;
    int32_t cdeg     = (turn_dmm * CDEG_PER_RAD_X10) / (S2_TRACK_MM * 100);

    if (cdeg > CDEG_HALF_TURN)
    {
        cdeg -= CDEG_FULL_TURN;
    }
    else if (cdeg <= -CDEG_HALF_TURN)
    {
        cdeg += CDEG_FULL_TURN;
    }
    else
    {
        // Already in range.
    }

    return (cdeg);
}

/*!
 * @brief Arc each wheel travels in an on-the-spot turn.
 *
 * @param[in] angle_deg Signed angle; clipped to +/- TURN_MAX_DEG.
 *
 * @return |angle| x pi x track / 360, in micrometres.
 */
static int32_t
turn_arc_um (int32_t angle_deg)
{
    // NOTE: The casts are safe: the angle is clipped to 0 ... 720, so
    // 720 x 112 x 31416 = 2.53e9 fits in uint32_t and the result
    // (703 mm = 703000 um) fits in int32_t.
    uint32_t angle = (uint32_t) clamp(magnitude(angle_deg), 0, TURN_MAX_DEG);
    uint32_t arc   = (angle * (uint32_t) S2_TRACK_MM * PI_X1E4) / 3600u;

    return ((int32_t) arc);
}

/*!
 * @brief Speed for a command: the request, or the default if 0.
 *
 * @param[in] requested_mms From the command.
 * @param[in] default_mms   Used when the request is 0.
 *
 * @return Speed, at most S2_SPEED_MAX_MMS.
 */
static int32_t
speed_pick (uint16_t requested_mms, int32_t default_mms)
{
    int32_t speed = default_mms;

    if (0u != requested_mms)
    {
        speed = clamp((int32_t) requested_mms, 1, S2_SPEED_MAX_MMS);
    }

    return (speed);
}

/*!
 * @brief Limit a value to a range.
 *
 * @param[in] value The value.
 * @param[in] low   Lowest result.
 * @param[in] high  Highest result (>= low).
 *
 * @return value, clipped to low ... high.
 */
static int32_t
clamp (int32_t value, int32_t low, int32_t high)
{
    int32_t result = value;

    if (value < low)
    {
        result = low;
    }
    else if (value > high)
    {
        result = high;
    }
    else
    {
        // In range.
    }

    return (result);
}

/*!
 * @brief Absolute value.
 *
 * @param[in] value Any value above INT32_MIN.
 *
 * @return |value|.
 */
static int32_t
magnitude (int32_t value)
{
    return ((value < 0) ? -value : value);
}

/*** end of file ***/
