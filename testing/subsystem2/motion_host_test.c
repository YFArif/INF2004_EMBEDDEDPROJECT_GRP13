/** @file motion_host_test.c
 *
 * @brief S2-U09: host (PC) test of motion_task.c against a simulated
 *        robot. No hardware and no other subsystem is needed.
 *
 * @par What it does
 * app_program/subsystem2/motion_task.c is #included unchanged. Its
 * drivers and kernel objects are replaced by the stubs below, and a
 * physics model plays the two wheels: first-order motor lag, a dead band
 * and floor load, a stronger right motor (S2-U06 saw +2 %), electrical
 * braking, and encoder ticks with 1 us timestamps. The test then drives
 * motion_task through every command and fault, cycle by cycle, and checks
 * the result against the design (NFR4, NFR5, NFR6, FR6, FR7, FR15 and
 * the IF-04 / IF-05 protocol). Built with the address and undefined-
 * behaviour sanitizers, so any overflow or out-of-range access fails it.
 *
 * @par
 * The model is not the real car: passing here proves the logic, the
 * arithmetic and the state machine; the floor tests (S2-U02, S2-U03,
 * S2-U11, S2-U12) prove the tuning.
 *
 * @par Run (from the repository root, in WSL)
 *     sh testing/subsystem2/run_motion_test.sh
 */

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/subsystem2/motion_task.c"

/* Simulation. */
#define SIM_STEP_US         (20u)
#define SIM_STEPS           (CYCLE_US / SIM_STEP_US)
#define SIM_TAU_S           (0.080)     /* Motor speed lag.              */
#define SIM_TAU_BRAKE_S     (0.030)     /* Electrical brake.             */
#define SIM_GAIN_L          (0.80)      /* mm/s per permille, no load.   */
#define SIM_GAIN_R          (0.82)      /* Right motor 2 % stronger.     */
#define SIM_DEAD_BAND       (50.0)      /* Permille that do not move it. */
#define SIM_LOAD_FLOOR      (0.85)
#define SIM_PI              (3.14159265358979)
#define SETTLE_CYCLES       (25)        /* 0.5 s to come to rest.        */
#define QUEUE_LEN           (16u)

/** One simulated wheel. */
typedef struct
{
    double   speed_mms;     /* True signed speed.                        */
    double   pos_um;        /* True signed distance.                     */
    double   travel_um;     /* Total distance either way (for ticks).    */
    double   gain;          /* mm/s per permille after the load.         */
    double   tick_um;       /* Distance per encoder tick.                */
    int32_t  duty;          /* Last duty set by motion_task.             */
    bool     b_blocked;     /* Held still (stall test).                  */
    uint32_t ticks;
    uint32_t tick_time_us;
} sim_wheel_t;

static sim_wheel_t         g_sim[WHEEL_COUNT];
static uint32_t            g_now_us        = 0u;
static subsystem2_cmd_t    g_queue[QUEUE_LEN];
static uint32_t            g_queue_head    = 0u;
static uint32_t            g_queue_count   = 0u;
static bool                g_b_estop       = false;
static int16_t             g_steer_mms     = 0;
static subsystem2_odom_t   g_odom          = {0};
static subsystem2_status_t g_status        = {0};
static uint32_t            g_done_events   = 0u;
static uint32_t            g_fault_events  = 0u;
static uint32_t            g_stall_faults  = 0u;
static bool                g_b_print       = false;
static motion_t            g_motion_boot;
static double              g_tau_s         = SIM_TAU_S;
static uint32_t            g_pass          = 0u;
static uint32_t            g_fail          = 0u;

/* ---------------------------------------------------------------------
 *  Stubs for the drivers, the S2 kernel objects and S5's hooks.
 * --------------------------------------------------------------------- */

ER
drv_encoder_init (void)
{
    return (E_OK);
}

void
drv_encoder_get (drv_encoder_t wheel, drv_encoder_count_t * p_count)
{
    (void) memset(p_count, 0, sizeof(*p_count));
    p_count->ticks        = g_sim[wheel].ticks;
    p_count->tick_time_us = g_sim[wheel].tick_time_us;
}

uint32_t
drv_encoder_now_us (void)
{
    return (g_now_us);
}

void
drv_motor_set (drv_motor_t motor, int32_t duty_permille)
{
    int32_t duty = duty_permille;

    if (duty > DRV_MOTOR_DUTY_MAX)
    {
        duty = DRV_MOTOR_DUTY_MAX;
    }
    else if (duty < -DRV_MOTOR_DUTY_MAX)
    {
        duty = -DRV_MOTOR_DUTY_MAX;
    }
    else
    {
        // In range.
    }

    g_sim[motor].duty = duty;
}

void
drv_motor_brake (void)
{
    g_sim[WHEEL_LEFT].duty  = 0;
    g_sim[WHEEL_RIGHT].duty = 0;
}

void
subsystem2_cycle_wait (bool b_estop_wakes)
{
    (void) b_estop_wakes;
}

bool
subsystem2_cmd_receive (subsystem2_cmd_t * p_cmd)
{
    bool b_got = false;

    if (g_queue_count > 0u)
    {
        *p_cmd       = g_queue[g_queue_head];
        g_queue_head = (g_queue_head + 1u) % QUEUE_LEN;
        g_queue_count--;
        b_got = true;
    }

    return (b_got);
}

void
subsystem2_odom_put (subsystem2_odom_t const * p_odom,
                     subsystem2_status_t const * p_status)
{
    g_odom   = *p_odom;
    g_status = *p_status;
}

int16_t
subsystem2_steer_get (void)
{
    return (g_steer_mms);
}

void
subsystem2_estop_set (void)
{
    g_b_estop = true;
}

bool
subsystem2_estop_is_set (void)
{
    return (g_b_estop);
}

void
subsystem5_mission_signal (uint32_t events)
{
    if (0u != (events & EV_MOTION_DONE))
    {
        g_done_events++;
    }

    if (0u != (events & EV_FAULT))
    {
        g_fault_events++;
    }
}

void
subsystem5_fault_set (uint32_t faults)
{
    if (0u != (faults & S5_FAULT_STALL))
    {
        g_stall_faults++;
    }
}

INT
tm_printf (UB const * p_format, ...)
{
    va_list args;

    if (g_b_print)
    {
        va_start(args, p_format);
        (void) printf("    firmware: ");
        (void) vprintf((char const *) p_format, args);
        va_end(args);
    }

    return (0);
}

/* ---------------------------------------------------------------------
 *  Simulation.
 * --------------------------------------------------------------------- */

static void
sim_reset (double load, uint32_t start_us)
{
    uint32_t idx = 0u;

    (void) memset(g_sim, 0, sizeof(g_sim));
    g_sim[WHEEL_LEFT].gain  = SIM_GAIN_L * load;
    g_sim[WHEEL_RIGHT].gain = SIM_GAIN_R * load;
    g_sim[WHEEL_LEFT].tick_um  = (S2_WHEEL_DIA_MM * SIM_PI * 1000.0) /
                                 S2_TICKS_PER_REV_L;
    g_sim[WHEEL_RIGHT].tick_um = (S2_WHEEL_DIA_MM * SIM_PI * 1000.0) /
                                 S2_TICKS_PER_REV_R;

    for (idx = 0u; idx < WHEEL_COUNT; idx++)
    {
        g_sim[idx].tick_time_us = start_us;
    }

    g_now_us       = start_us;
    g_queue_head   = 0u;
    g_queue_count  = 0u;
    g_b_estop      = false;
    g_steer_mms    = 0;
    g_done_events  = 0u;
    g_fault_events = 0u;
    g_stall_faults = 0u;
    (void) memset(&g_odom, 0, sizeof(g_odom));
    (void) memset(&g_status, 0, sizeof(g_status));

    g_motion = g_motion_boot;
    motion_setup();
}

static void
sim_wheel_step (sim_wheel_t * p_wheel, double dt_s)
{
    double magnitude_permille = 0.0;
    double target_mms         = 0.0;
    double step_um            = 0.0;

    if (p_wheel->b_blocked)
    {
        p_wheel->speed_mms = 0.0;
    }
    else if (0 == p_wheel->duty)
    {
        p_wheel->speed_mms -= (p_wheel->speed_mms * dt_s) / SIM_TAU_BRAKE_S;
    }
    else
    {
        magnitude_permille = fabs((double) p_wheel->duty) - SIM_DEAD_BAND;
        if (magnitude_permille < 0.0)
        {
            magnitude_permille = 0.0;
        }

        target_mms = copysign(p_wheel->gain * magnitude_permille,
                              (double) p_wheel->duty);
        p_wheel->speed_mms += ((target_mms - p_wheel->speed_mms) * dt_s) /
                              g_tau_s;
    }

    step_um             = p_wheel->speed_mms * dt_s * 1000.0;
    p_wheel->pos_um    += step_um;
    p_wheel->travel_um += fabs(step_um);

    while (p_wheel->travel_um >= ((double) (p_wheel->ticks + 1u) *
                                  p_wheel->tick_um))
    {
        p_wheel->ticks++;
        p_wheel->tick_time_us = g_now_us;
    }
}

/* One 20 ms period of the robot, then one motion_task cycle. */
static void
sim_cycle (void)
{
    uint32_t step = 0u;

    for (step = 0u; step < SIM_STEPS; step++)
    {
        g_now_us += SIM_STEP_US;
        sim_wheel_step(&g_sim[WHEEL_LEFT], SIM_STEP_US / 1e6);
        sim_wheel_step(&g_sim[WHEEL_RIGHT], SIM_STEP_US / 1e6);
    }

    motion_cycle();
}

static void
sim_run (uint32_t cycles)
{
    uint32_t idx = 0u;

    for (idx = 0u; idx < cycles; idx++)
    {
        sim_cycle();
    }
}

/* Run until motion_task is IDLE; returns the cycles used, or 0 if not. */
static uint32_t
sim_run_until_idle (uint32_t max_cycles)
{
    uint32_t used = 0u;

    do
    {
        sim_cycle();
        used++;
    } while ((S2_STATE_IDLE != g_status.state) && (used < max_cycles));

    return ((S2_STATE_IDLE == g_status.state) ? used : 0u);
}

static void
send (subsystem2_cmd_type_t type, int16_t value, uint16_t speed_mms)
{
    subsystem2_cmd_t cmd =
    {
        .type = (uint8_t) type, .value = value, .speed_mms = speed_mms
    };

    if (g_queue_count < QUEUE_LEN)
    {
        g_queue[(g_queue_head + g_queue_count) % QUEUE_LEN] = cmd;
        g_queue_count++;
    }
}

static double
true_dist_mm (void)
{
    return ((g_sim[WHEEL_LEFT].pos_um + g_sim[WHEEL_RIGHT].pos_um) / 2000.0);
}

static double
true_heading_deg (void)
{
    return (((g_sim[WHEEL_RIGHT].pos_um - g_sim[WHEEL_LEFT].pos_um) /
             (1000.0 * S2_TRACK_MM)) * (180.0 / SIM_PI));
}

static void
check (char const * p_id, bool b_ok, char const * p_format, ...)
{
    va_list args;

    va_start(args, p_format);
    (void) printf("[HOST] %s %s  ", p_id, b_ok ? "PASS" : "FAIL");
    (void) vprintf(p_format, args);
    (void) printf("\n");
    va_end(args);

    if (b_ok)
    {
        g_pass++;
    }
    else
    {
        g_fail++;
    }
}

/* ---------------------------------------------------------------------
 *  Tests.
 * --------------------------------------------------------------------- */

/* NFR4: FOLLOW 0 -> 250 mm/s; both wheels within +/-10 % from 0.5 s. */
static void
test_step (char const * p_id, double load, uint32_t start_us)
{
    double   worst   = 0.0;
    double   err     = 0.0;
    double   peak    = 0.0;
    uint32_t cycle   = 0u;
    uint32_t idx     = 0u;
    uint32_t settled = 0u;

    sim_reset(load, start_us);
    send(S2_CMD_FOLLOW, 0, 250u);

    for (cycle = 1u; cycle <= 75u; cycle++)
    {
        sim_cycle();

        for (idx = 0u; idx < WHEEL_COUNT; idx++)
        {
            err = fabs(g_sim[idx].speed_mms - 250.0);

            if (g_sim[idx].speed_mms > peak)
            {
                peak = g_sim[idx].speed_mms;
            }

            if ((cycle * S2_MOTION_PERIOD_MS) > 500u)
            {
                if (err > worst)
                {
                    worst = err;
                }
            }
        }

        if ((0u == settled) &&
            (fabs(g_sim[WHEEL_LEFT].speed_mms - 250.0) <= 25.0) &&
            (fabs(g_sim[WHEEL_RIGHT].speed_mms - 250.0) <= 25.0))
        {
            settled = cycle * S2_MOTION_PERIOD_MS;
        }
    }

    check(p_id, (worst <= 25.0) && (peak <= 300.0) &&
                (S2_STATE_FOLLOWING == g_status.state),
          "step 0->250 mm/s, load %.2f: in band at %u ms, worst error after"
          " 500 ms %.1f (limit 25), overshoot %.0f%% (limit 20%%)", load,
          (unsigned) settled, worst, ((peak - 250.0) * 100.0) / 250.0);
}

/* FR6: MOVE by encoder distance, both wheels level, odometry agrees. */
static void
test_move (char const * p_id, int16_t mm, uint16_t speed, double tol_mm)
{
    uint32_t used = 0u;
    double   dist = 0.0;
    double   head = 0.0;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_MOVE, mm, speed);
    used = sim_run_until_idle(1000u);
    sim_run(SETTLE_CYCLES);
    dist = true_dist_mm();
    head = true_heading_deg();

    check(p_id, (0u != used) && (fabs(dist - mm) <= tol_mm) &&
                (fabs(head) <= 2.0) && (1u == g_done_events) &&
                (fabs((double) g_odom.dist_mm - dist) <= 3.0),
          "MOVE %d mm @ %u: stopped at %.1f mm (tol %.0f), heading %.2f deg,"
          " odom %d mm, %u ms, EV_MOTION_DONE x%u", (int) mm,
          (unsigned) speed, dist, tol_mm, head, (int) g_odom.dist_mm,
          (unsigned) (used * S2_MOTION_PERIOD_MS), (unsigned) g_done_events);
}

/* FR6, NFR5: TURN on the spot by encoder arc. */
static void
test_turn (char const * p_id, int16_t deg)
{
    uint32_t used = 0u;
    double   head = 0.0;
    double   dist = 0.0;
    double   odom = 0.0;
    double   diff = 0.0;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_TURN, deg, 0u);
    used = sim_run_until_idle(1000u);
    sim_run(SETTLE_CYCLES);
    head = true_heading_deg();
    dist = true_dist_mm();
    odom = g_odom.heading_cdeg / 100.0;

    // Compare on the circle: 179.9 and -179.9 are 0.2 degrees apart.
    diff = fmod((odom - head) + 540.0, 360.0) - 180.0;

    check(p_id, (0u != used) && (fabs(head - deg) <= 3.0) &&
                (fabs(dist) <= 3.0) && (1u == g_done_events) &&
                (fabs(diff) <= 0.5),
          "TURN %d deg: turned %.2f deg (tol 3), odom heading %.2f deg,"
          " centre moved %.1f mm, %u ms, EV_MOTION_DONE x%u", (int) deg,
          head, odom, dist, (unsigned) (used * S2_MOTION_PERIOD_MS),
          (unsigned) g_done_events);
}

static void
test_stop_interrupts_move (void)
{
    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_MOVE, 1000, 0u);
    sim_run(50u);
    send(S2_CMD_STOP, 0, 0u);
    sim_cycle();

    check("H07", (S2_STATE_IDLE == g_status.state) &&
                 (0 == g_sim[WHEEL_LEFT].duty) &&
                 (0 == g_sim[WHEEL_RIGHT].duty),
          "STOP during MOVE: IDLE and braked in the same cycle");

    sim_run(SETTLE_CYCLES);
    check("H07", (0u == g_done_events) && (true_dist_mm() < 500.0),
          "STOP cancels the MOVE: no EV_MOTION_DONE, stopped at %.0f mm",
          true_dist_mm());
}

/* NFR6, IF-04, IF-05: E-stop latch and the CLEAR protocol. */
static void
test_estop (void)
{
    uint32_t discarded = 0u;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_FOLLOW, 0, 300u);
    sim_run(40u);
    g_b_estop = true;
    sim_cycle();

    check("H08", (S2_STATE_ESTOP == g_status.state) &&
                 (0 == g_sim[WHEEL_LEFT].duty) &&
                 (0 == g_sim[WHEEL_RIGHT].duty),
          "MF_ESTOP brakes in the first cycle that sees it");

    discarded = g_status.discarded_count;
    send(S2_CMD_MOVE, 100, 0u);
    send(S2_CMD_CLEAR, 0, 0u);
    sim_run(5u);
    check("H08", (S2_STATE_ESTOP == g_status.state) &&
                 ((discarded + 2u) == g_status.discarded_count) &&
                 (0 == g_sim[WHEEL_LEFT].duty),
          "while latched: MOVE discarded, CLEAR ignored, still braked");

    g_b_estop = false;
    sim_run(3u);
    check("H08", S2_STATE_ESTOP == g_status.state,
          "flag cleared but no CLEAR yet: stays braked");

    send(S2_CMD_CLEAR, 0, 0u);
    send(S2_CMD_MOVE, 50, 0u);
    sim_cycle();
    check("H08", S2_STATE_MOVING == g_status.state,
          "CLEAR then MOVE sent together: the MOVE runs");

    (void) sim_run_until_idle(500u);
    check("H08", 1u == g_done_events, "and finishes with EV_MOTION_DONE");
}

/* FR15: stall -> brake, MF_ESTOP, S5_FAULT_STALL + EV_FAULT. */
static void
test_stall (void)
{
    uint32_t used = 0u;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_FOLLOW, 0, 300u);
    sim_run(40u);
    g_sim[WHEEL_LEFT].b_blocked = true;

    do
    {
        sim_cycle();
        used++;
    } while ((S2_STATE_ESTOP != g_status.state) && (used < 100u));

    check("H09", (S2_STATE_ESTOP == g_status.state) && g_b_estop &&
                 (1u == g_stall_faults) && (1u == g_fault_events) &&
                 (0 == g_sim[WHEEL_RIGHT].duty) && (used >= STALL_CYCLES) &&
                 (used <= 30u),
          "left wheel held: stall after %u ms (300 ms of high duty),"
          " MF_ESTOP set, S5_FAULT_STALL x%u, EV_FAULT x%u",
          (unsigned) (used * S2_MOTION_PERIOD_MS), (unsigned) g_stall_faults,
          (unsigned) g_fault_events);
}

/* IF-02: positive steering turns left (right wheel faster). */
static void
test_steer (void)
{
    double   sum_l = 0.0;
    double   sum_r = 0.0;
    uint32_t idx   = 0u;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    g_steer_mms = 50;
    send(S2_CMD_FOLLOW, 0, 250u);
    sim_run(50u);

    for (idx = 0u; idx < 25u; idx++)
    {
        sim_cycle();
        sum_l += g_sim[WHEEL_LEFT].speed_mms;
        sum_r += g_sim[WHEEL_RIGHT].speed_mms;
    }

    check("H10", (fabs((sum_l / 25.0) - 200.0) <= 15.0) &&
                 (fabs((sum_r / 25.0) - 300.0) <= 15.0) &&
                 (true_heading_deg() > 0.0),
          "FOLLOW 250, steer +50: left %.0f mm/s, right %.0f mm/s, heading"
          " %+.0f deg (left)", sum_l / 25.0, sum_r / 25.0,
          true_heading_deg());
}

/* Edge-timed speed against the true speed. */
static void
test_speed_accuracy (void)
{
    double   err_l = 0.0;
    double   err_r = 0.0;
    uint32_t idx   = 0u;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_FOLLOW, 0, 300u);
    sim_run(50u);

    for (idx = 0u; idx < 50u; idx++)
    {
        sim_cycle();
        err_l += fabs(g_odom.speed_l_mms - g_sim[WHEEL_LEFT].speed_mms);
        err_r += fabs(g_odom.speed_r_mms - g_sim[WHEEL_RIGHT].speed_mms);
    }

    check("H12", ((err_l / 50.0) <= 9.0) && ((err_r / 50.0) <= 9.0),
          "measured wheel speed vs true at 300 mm/s: mean error L %.1f,"
          " R %.1f mm/s (limit 9 = 3 %%)", err_l / 50.0, err_r / 50.0);
}

/* A new FOLLOW only changes the speed (e.g. half speed near obstacles). */
static void
test_speed_change (void)
{
    bool     b_stayed = true;
    uint32_t idx      = 0u;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_FOLLOW, 0, 300u);
    sim_run(50u);
    send(S2_CMD_FOLLOW, 0, 150u);

    for (idx = 0u; idx < 40u; idx++)
    {
        sim_cycle();
        b_stayed = b_stayed && (S2_STATE_FOLLOWING == g_status.state);
    }

    check("H13", b_stayed &&
                 (fabs(g_sim[WHEEL_LEFT].speed_mms - 150.0) <= 15.0) &&
                 (fabs(g_sim[WHEEL_RIGHT].speed_mms - 150.0) <= 15.0),
          "FOLLOW 300 -> 150 while following: L %.0f R %.0f mm/s after"
          " 0.8 s, never left FOLLOWING", g_sim[WHEEL_LEFT].speed_mms,
          g_sim[WHEEL_RIGHT].speed_mms);
}

/* Documented semantics: a new command replaces the running one. */
static void
test_latest_wins (void)
{
    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_MOVE, 1000, 0u);
    send(S2_CMD_TURN, 90, 0u);
    (void) sim_run_until_idle(1000u);
    sim_run(SETTLE_CYCLES);

    check("H14", (fabs(true_heading_deg() - 90.0) <= 3.0) &&
                 (fabs(true_dist_mm()) <= 3.0) && (1u == g_done_events),
          "MOVE then TURN in one cycle: only the TURN runs (send the next"
          " command after EV_MOTION_DONE)");
}

/* Long run: odometry keeps up and nothing overflows (sanitizers). */
static void
test_long_run (void)
{
    double dist = 0.0;

    sim_reset(SIM_LOAD_FLOOR, 0xFFFFFFFFu - 5000000u);
    send(S2_CMD_FOLLOW, 0, 400u);
    sim_run(3000u);
    dist = true_dist_mm();

    check("H15", fabs((double) g_odom.dist_mm - dist) <= (0.005 * dist),
          "60 s at 400 mm/s (timer wraps after 5 s): odom %d mm, true %.0f"
          " mm", (int) g_odom.dist_mm, dist);
}

/* Speed requests are clipped to S2_SPEED_MAX_MMS. */
static void
test_speed_clip (void)
{
    double   fastest = 0.0;
    uint32_t idx     = 0u;

    sim_reset(1.0, 0u);
    send(S2_CMD_FOLLOW, 0, 900u);

    for (idx = 0u; idx < 100u; idx++)
    {
        sim_cycle();
        if (g_sim[WHEEL_LEFT].speed_mms > fastest)
        {
            fastest = g_sim[WHEEL_LEFT].speed_mms;
        }
    }

    check("H16", fastest <= (S2_SPEED_MAX_MMS * 1.1),
          "FOLLOW 900 is clipped to %d mm/s: fastest %.0f mm/s (limit +10%%)",
          (int) S2_SPEED_MAX_MMS, fastest);
}

/* Bad commands are counted and change nothing. */
static void
test_bad_command (void)
{
    uint32_t discarded = 0u;

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_FOLLOW, 0, 200u);
    sim_run(10u);
    discarded = g_status.discarded_count;
    send((subsystem2_cmd_type_t) 9, 0, 0u);
    sim_run(2u);

    check("H17", (S2_STATE_FOLLOWING == g_status.state) &&
                 ((discarded + 1u) == g_status.discarded_count),
          "unknown command type: discarded and counted, still FOLLOWING");

    sim_reset(SIM_LOAD_FLOOR, 0u);
    send(S2_CMD_TURN, 32767, 0u);
    (void) sim_run_until_idle(2000u);
    sim_run(SETTLE_CYCLES);
    check("H17", fabs(true_heading_deg() - TURN_MAX_DEG) <= 5.0,
          "TURN 32767 deg is clipped to %d: turned %.0f deg",
          (int) TURN_MAX_DEG, true_heading_deg());
}

int
main (void)
{
    g_motion_boot = g_motion;

    (void) printf("S2-U09 host test of motion_task.c (simulated robot)\n");
    (void) printf("-- speed control (NFR4)\n");
    test_step("H01", 1.0, 0u);
    test_step("H02", SIM_LOAD_FLOOR, 0u);
    test_step("H03", 0.6, 0u);
    test_step("H11", SIM_LOAD_FLOOR, 0xFFFFFFFFu - 300000u);

    // The real motors' time constant is not measured yet: the gains must
    // also work if it is half or about twice the model's 80 ms.
    g_tau_s = 0.040;
    test_step("H18", SIM_LOAD_FLOOR, 0u);
    g_tau_s = 0.150;
    test_step("H18", SIM_LOAD_FLOOR, 0u);
    test_step("H18", 0.6, 0u);
    g_tau_s = SIM_TAU_S;

    (void) printf("-- MOVE and TURN (FR6, NFR5)\n");
    test_move("H04", 1000, 0u, 10.0);
    test_move("H04", 300, 150u, 5.0);
    test_move("H05", -100, 0u, 5.0);
    test_turn("H06", 90);
    test_turn("H06", -90);
    test_turn("H06", 180);
    test_turn("H06", 45);
    test_turn("H06", 360);
    test_turn("H06", -540);

    (void) printf("-- commands, E-stop and faults (IF-04, IF-05, FR15)\n");
    test_stop_interrupts_move();
    test_estop();
    test_stall();
    test_steer();
    test_speed_change();
    test_latest_wins();
    test_bad_command();

    (void) printf("-- measurement and robustness\n");
    test_speed_accuracy();
    test_long_run();
    test_speed_clip();

    (void) printf("S2-U09: %s (%u passed, %u failed)\n",
                  (0u == g_fail) ? "PASS" : "FAIL", (unsigned) g_pass,
                  (unsigned) g_fail);

    return ((0u == g_fail) ? 0 : 1);
}

/*** end of file ***/
