#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>

ER tk_dly_tsk(RELTIM ms);

#include "../../app_program/subsystem2/subsystem2_test.c"

static uint32_t g_now;
static uint32_t g_button_origin;
static uint32_t g_run_ms;
static bool g_active;
static bool g_estop;
static bool g_abort;
static bool g_late;
static bool g_fail_follow;
static bool g_fail_stop;
static bool g_fail_idle;
static uint32_t g_follow_calls;
static uint32_t g_motor_calls;
static uint32_t g_rows;
static uint32_t g_extra_ms;
static uint32_t g_ticks[2];
static int32_t g_duty[2];
static subsystem2_status_t g_status;
#if (S2_MOTOR_DIAG == 3)
static int16_t g_move_distance;
static uint16_t g_move_speed;
static bool g_never_done;
#endif

INT
tm_printf (UB const * p_format, ...)
{
    if ((0 == strncmp((char const *) p_format, "D,", 2u)) ||
        (0 == strncmp((char const *) p_format, "M,", 2u)))
    {
        assert(!g_active);
        g_rows++;
    }
    return 0;
}

ER
tk_dly_tsk (RELTIM ms)
{
    if (g_active)
    {
        ms += g_extra_ms;
    }
    g_now += ms * 1000u;
    if (g_active)
    {
        g_run_ms += ms;
        if (g_late)
        {
            g_now += 200000u;
        }
        if (g_duty[0] != 0)
        {
            g_ticks[0]++;
        }
        if (g_duty[1] != 0)
        {
            g_ticks[1]++;
        }
#if (S2_MOTOR_DIAG == 3)
        if ((!g_never_done) && (!g_late) && (g_run_ms >= 300u))
        {
            g_status.done_count++;
            g_status.state = S2_STATE_IDLE;
            g_active = false;
        }
#endif
    }
    return E_OK;
}

bool
drv_button_is_down (drv_button_id_t button)
{
    (void) button;
    if (g_active)
    {
        return g_abort && (g_run_ms >= 60u);
    }
    return ((g_now - g_button_origin) >= 10000u) &&
           ((g_now - g_button_origin) < 80000u);
}

uint32_t
drv_encoder_now_us (void)
{
    return g_now;
}

void
drv_encoder_get (drv_encoder_t wheel, drv_encoder_count_t * p_count)
{
    (void) memset(p_count, 0, sizeof(*p_count));
    p_count->ticks = g_ticks[wheel];
    p_count->tick_time_us = g_now;
}

bool
subsystem2_estop_is_set (void)
{
    return g_estop;
}

void
subsystem2_estop_set (void)
{
    g_estop = true;
}

void
drv_motor_set (drv_motor_t motor, int32_t duty)
{
    assert(!g_estop);
    assert(S2_STATE_IDLE == g_status.state);
    g_duty[motor] = duty;
    g_active = true;
    g_motor_calls++;
}

void
drv_motor_brake (void)
{
    g_duty[0] = 0;
    g_duty[1] = 0;
    g_active = false;
}

ER
subsystem2_follow (uint16_t speed)
{
    assert(!g_estop);
    assert(0u == g_motor_calls);
    g_follow_calls++;
    if (g_fail_follow)
    {
        return -1;
    }
    g_status.state = S2_STATE_FOLLOWING;
    g_status.target_l_mms = speed;
    g_status.target_r_mms = speed;
    g_active = true;
    return E_OK;
}

#if (S2_MOTOR_DIAG == 3)
ER
subsystem2_move (int16_t distance, uint16_t speed)
{
    assert(!g_estop);
    g_move_distance = distance;
    g_move_speed = speed;
    if (g_fail_follow)
    {
        return -1;
    }
    g_status.state = S2_STATE_MOVING;
    g_status.target_l_mms = (distance < 0) ? -(int32_t) speed : speed;
    g_status.target_r_mms = g_status.target_l_mms;
    g_active = true;
    return E_OK;
}
#endif

ER
subsystem2_stop (void)
{
    if (g_fail_stop)
    {
        return -1;
    }
    if (!g_fail_idle)
    {
        g_status.state = S2_STATE_IDLE;
        drv_motor_brake();
    }
    return E_OK;
}

void
subsystem2_steer_set (int16_t steer)
{
    assert(0 == steer);
}

void
subsystem2_status_get (subsystem2_status_t * p_status)
{
    *p_status = g_status;
}

void
subsystem2_odom_get (subsystem2_odom_t * p_odom)
{
    (void) memset(p_odom, 0, sizeof(*p_odom));
    p_odom->count_l = g_ticks[0];
    p_odom->count_r = g_ticks[1];
}

int
main (void)
{
    uint32_t scenario;

#if (S2_MOTOR_DIAG == 3)
    for (scenario = 0u; scenario < 11u; scenario++)
    {
        uint16_t const speeds[] = {40u, 60u, 100u};
        bool expected = (scenario < 6u) || (10u == scenario);
        bool result;
        int16_t distance = (0u == (scenario % 2u)) ? 400 : -400;
        uint16_t speed = speeds[(scenario / 2u) % 3u];

        g_now = (10u == scenario) ? UINT32_MAX - 100000u : 0u;
        g_button_origin = g_now;
        g_run_ms = 0u;
        g_active = false;
        g_estop = false;
        g_abort = (6u == scenario);
        g_late = (7u == scenario);
        g_fail_follow = (8u == scenario);
        g_never_done = (9u == scenario);
        g_extra_ms = 10u;
        g_rows = 0u;
        g_motor_calls = 0u;
        g_follow_calls = 0u;
        (void) memset(&g_status, 0, sizeof(g_status));
        result = diag_move(scenario + 1u, distance, speed);
        assert(result == expected);
        assert(g_move_distance == distance);
        assert(g_move_speed == speed);
        assert(!g_active);
        assert(g_estop == !expected);
        assert(0u == g_motor_calls);
        assert(0u == g_follow_calls);
        if (expected)
        {
            assert(1u == g_status.done_count);
            assert(S2_STATE_IDLE == g_status.state);
            assert(3u == g_rows);
        }
        (void) printf("MOVE HOST PASS scenario %u\n", (unsigned) scenario);
    }
#else
    for (scenario = 0u; scenario < 14u; scenario++)
    {
        bool b_ok;
        bool b_closed = (1u == scenario) || (4u == scenario) || (6u == scenario);
        bool b_expected = (scenario < 2u) || (scenario >= 8u);
        uint16_t speed = (12u == scenario) ? 60u :
                         ((13u == scenario) ? 100u : 40u);
#if (S2_MOTOR_DIAG == 2)
        b_closed = true;
#endif

        g_extra_ms = (scenario >= 8u) ? 10u : 0u;
        if (9u == scenario)
        {
            g_extra_ms = 80u;
        }
        if (10u == scenario)
        {
            b_closed = true;
        }
        g_now = (11u == scenario) ? UINT32_MAX - 1000000u : 0u;
        g_button_origin = g_now;
        g_run_ms = 0u;
        g_active = false;
        g_estop = (7u == scenario);
        g_abort = (2u == scenario);
        g_late = (3u == scenario);
        g_fail_follow = (4u == scenario);
        g_fail_stop = (5u == scenario);
        g_fail_idle = (6u == scenario);
        g_follow_calls = 0u;
        g_motor_calls = 0u;
        g_rows = 0u;
        (void) memset(g_ticks, 0, sizeof(g_ticks));
        (void) memset(g_duty, 0, sizeof(g_duty));
        (void) memset(&g_status, 0, sizeof(g_status));
        b_ok = diag_run(1u, b_closed, DRV_MOTOR_RIGHT, b_closed ? speed : 150u);
        if (b_ok != b_expected)
        {
            (void) fprintf(stderr, "scenario %u: result=%d expected=%d rows=%u run_ms=%u\n",
                           (unsigned) scenario, b_ok, b_expected,
                           (unsigned) g_rows, (unsigned) g_run_ms);
        }
        assert(b_ok == b_expected);
        assert(!g_active && (0 == g_duty[0]) && (0 == g_duty[1]));
        assert(g_estop == !b_expected);
        if (b_expected)
        {
            assert((g_rows > 0u) && (g_rows <= DIAG_SAMPLES));
            assert((g_run_ms >= (DIAG_DURATION_US / 1000u)) &&
                   (g_run_ms <= (DIAG_DURATION_US / 1000u + 100u)));
            if ((8u == scenario) || (10u == scenario) || (11u == scenario))
            {
                assert(((DIAG_DURATION_US + 29999u) / 30000u) == g_rows);
                assert((g_rows * 30u) == g_run_ms);
            }
            assert(g_follow_calls == (b_closed ? 1u : 0u));
            assert(g_motor_calls == (b_closed ? 0u : 1u));
            if (b_closed)
            {
                assert(g_status.target_l_mms == speed);
                assert(g_status.target_r_mms == speed);
            }
        }
#if (S2_MOTOR_DIAG == 2)
        assert(0u == g_motor_calls);
#endif
        (void) printf("DIAG HOST PASS mode %u scenario %u\n",
                      (unsigned) S2_MOTOR_DIAG, (unsigned) scenario);
    }
#endif
    return 0;
}
