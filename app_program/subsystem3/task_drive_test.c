/** @file task_drive_test.c
 *
 * @brief Subsystem 3 -- TEST ONLY (S3_DRIVE_TEST 1): line following on
 *        Buddy 2's motors before the Vehicle Controller exists.
 *
 * @par How to use (Button 1 only, GP20)
 * Sweep the car sideways across the line until it is calibrated, put it
 * on the line, then:
 * - SHORT press (let go within 1.5 s): start after a 3 s countdown.
 * - HOLD 1.5 s, let go before 4 s: next preset (printed).
 * - HOLD 4 s: recalibrate (forget both sensors' white / black levels,
 *   then sweep again). No power cycle needed.
 * - Any press while driving: stop.
 * While waiting, "S3 DRIVE CAL" prints both sensors' white / black levels
 * whenever they change. A start is refused until both black levels are
 * at least S3_DRIVE_CAL_BLACK_MIN (a half-done sweep looked "valid" at
 * 2099 on 28 Oct, then kept learning while driving).
 * Button 2 (GP21) also steps the preset, if it can be pressed. Both
 * buttons are only read, with the pad pull-up, as in subsystem2_step.c.
 *
 * @par Stops by itself
 * Button 1, LINE LOST for S3_DRIVE_LOST_MM or S3_DRIVE_LOST_MS, no
 * odometry change for S3_DRIVE_STILL_MS (LOST is measured in distance, so
 * a car that stopped moving stayed in FOLLOW for ever: log 28 Oct), an S2
 * emergency stop (for example a wheel stall: duty >= 40 % and no encoder
 * tick for 300 ms latches MF_ESTOP and S2 then ignores every command), or
 * the distance / time limit.
 *
 * @par S2 emergency stop: reported, NOT cleared
 * Buddy 2's reference (MOTOR_S3_REFERENCE.md section 10): "S3 must not
 * clear an E-stop just to make line-following tests run." If MF_ESTOP is
 * latched the run is refused with the S2 counters; power-cycle the car.
 * Steering is now limited to the base speed (preset steer_max), so the
 * inner wheel keeps rolling forward and should not stall.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../subsystem2/subsystem2.h"   /* follow / stop / E-stop.       */

#include "s3_api.h"
#include "s3_config.h"
#include "s3_private.h"
#include "task_drive_test.h"

#if (1 == S3_DRIVE_TEST)

#define POLL_MS             (20u)
#define PRESS_MS            (60u)   /* Held this long = a real press.   */
#define HOLD_MS             (1500u) /* Held this long = next preset.    */
#define CAL_PRINT_MS        (1000u) /* Check the levels this often.     */
#define CAL_PRINT_STEP      (50)    /* Print when one moved this much.  */
#define SETTLE_MS           (100)
#define COUNTDOWN_S         (3u)
#define COUNTDOWN_STEP_MS   (1000)
#define AFTER_STOP_MS       (500)
#define CLEAR_WAIT_MS       (100)   /* Let motion_task take the CLEAR.  */
#define U8_WHITE_BELOW      (64u)   /* s3_line_status_t n_xxx: 0..255.  */
#define U8_BLACK_ABOVE      (191u)
#define PERMILLE            (1000u)
#define U8_FULL             (255u)

/** Preset: Kp x100, Kd x100, speed mm/s, steer_sp at full lock mm/s. */
typedef struct
{
    int16_t  kp_x100;
    int16_t  kd_x100;
    uint16_t speed_mms;
    uint16_t steer_max_mms;
} drive_preset_t;

/* v20 (28 Oct): gains for the 20 mm straddle layout. Car log v19 (Kp 150
 * / Kd 400): Centre in the edge band 52 %, dv sign flips 27 % of steps,
 * mean |dv| in the band only 20 mm/s. Host sim with sensor noise: Kp 250 /
 * Kd 50 halves the flips and keeps the Centre in the band ~95 % (UNVERIFIED
 * on the car). Speed 80 mm/s and steer_max = speed throughout, so presets
 * differ ONLY in gains and can be compared directly. */
static drive_preset_t const g_presets[] =
{
    {250,  50,  80,  80},   /* 0: default.                              */
    {200,  50,  80,  80},   /* 1: softer P (if it weaves on straights). */
    {300,  50,  80,  80},   /* 2: stronger P (if it is late on curves). */
    {250,   0,  80,  80},   /* 3: no D (if still shaky).                */
    {250, 100,  80,  80}    /* 4: more D (if it overshoots).            */
};

#define N_PRESETS   (sizeof(g_presets) / sizeof(g_presets[0]))

static uint32_t g_preset = 0u;

static void drive_task(INT stacd, void * p_exinf);
static bool button1_down(void);
static bool button2_down(void);
static void button_input(uint32_t pin);
static void preset_next(void);
static void preset_apply(void);
static void wait_start(void);
static bool start_allowed(void);
static bool cal_ok(void);
static void cal_print(s3_cal_levels_t const * p_cal);
static void cal_watch(s3_cal_levels_t * p_last);
static bool cal_moved(uint16_t now, uint16_t was);
static bool estop_clear_ok(void);
static void drive_run(void);

/*!
 * @brief Create the drive test task on core 0.
 */
ER
task_drive_test_start (void)
{
    ID tid = app_start_task((FP) drive_task, S3_PRI_DRIVE_TEST, TP_PRC1,
                            "s3_drive_task");

    return (tid < E_OK) ? tid : E_OK;
}

static void
drive_task (INT stacd, void * p_exinf)
{
    uint32_t i;

    (void) stacd;
    (void) p_exinf;
    button_input(S3_DRIVE_BTN1_PIN);
    button_input(S3_DRIVE_BTN2_PIN);
    tk_dly_tsk(SETTLE_MS);
    preset_apply();

    for (;;)
    {
        tm_printf((UB *) "S3 DRIVE: calibrate (sweep sideways across the "
                         "line), put the Centre on the line or its %s edge. "
                         "BUTTON 1: short press = START, hold 1.5 s = next "
                         "preset, hold 4 s = recalibrate.\n", S3_EDGE_TXT);
        wait_start();
        if (!cal_ok())
        {
            continue;
        }
        for (i = COUNTDOWN_S; i > 0u; i--)
        {
            tm_printf((UB *) "S3 DRIVE %d\n", (INT) i);
            tk_dly_tsk(COUNTDOWN_STEP_MS);
        }
        if (start_allowed() && estop_clear_ok())
        {
            drive_run();
        }
    }
}

/*!
 * @brief Only start from a calibrated sensor that can tell which side the
 *        line is on. UNCALIBRATED gets no steering (drives blind); LOST
 *        keeps its last correction (spins). "Both white" is refused: the
 *        line could be on either side.
 */
static bool
start_allowed (void)
{
    s3_line_status_t st = {0};

    if ((E_OK != s3_get_line_status(&st)) || (S3_LINE_FOLLOW != st.state))
    {
        tm_printf((UB *) "S3 DRIVE NOT STARTED: line state is %s. Sweep the "
                         "car sideways across the line until it is "
                         "calibrated, then put the Centre on the %s edge.\n",
                  s3_state_txt((uint32_t) st.state, 0u), S3_EDGE_TXT);
        return false;
    }
    if ((st.n_centre < U8_WHITE_BELOW) && (st.n_outer <= U8_BLACK_ABOVE))
    {
        tm_printf((UB *) "S3 DRIVE NOT STARTED: neither sensor is on the line "
                         "(C=%d, outer=%d of 1000). Put the car so the Centre "
                         "or the outer sensor is over the black line.\n",
                  (INT) ((st.n_centre * PERMILLE) / U8_FULL),
                  (INT) ((st.n_outer * PERMILLE) / U8_FULL));
        return false;
    }
    return true;
}

/*!
 * @brief Both sensors swept far enough: black levels at least
 *        S3_DRIVE_CAL_BLACK_MIN. Prints the levels either way.
 */
static bool
cal_ok (void)
{
    s3_cal_levels_t cal = {0};

    s3_cal_levels_get(&cal);
    cal_print(&cal);
    if ((cal.black_o >= S3_DRIVE_CAL_BLACK_MIN) &&
        (cal.black_c >= S3_DRIVE_CAL_BLACK_MIN))
    {
        return true;
    }
    tm_printf((UB *) "S3 DRIVE NOT STARTED: black level below %d. Sweep BOTH "
                     "sensors fully over the black line (or hold BUTTON 1 "
                     "4 s to recalibrate).\n", (INT) S3_DRIVE_CAL_BLACK_MIN);
    return false;
}

static void
cal_print (s3_cal_levels_t const * p_cal)
{
    bool b_ready = (p_cal->black_o >= S3_DRIVE_CAL_BLACK_MIN) &&
                   (p_cal->black_c >= S3_DRIVE_CAL_BLACK_MIN);

    tm_printf((UB *) "S3 DRIVE CAL: outer white %d black %d, Centre white %d "
                     "black %d (raw) -> %s\n",
              (INT) p_cal->white_o, (INT) p_cal->black_o,
              (INT) p_cal->white_c, (INT) p_cal->black_c,
              b_ready ? "READY" : "keep sweeping");
}

/*!
 * @brief While waiting: print the levels when one moved CAL_PRINT_STEP.
 */
static void
cal_watch (s3_cal_levels_t * p_last)
{
    s3_cal_levels_t cal = {0};

    s3_cal_levels_get(&cal);
    if (cal_moved(cal.white_o, p_last->white_o) ||
        cal_moved(cal.black_o, p_last->black_o) ||
        cal_moved(cal.white_c, p_last->white_c) ||
        cal_moved(cal.black_c, p_last->black_c))
    {
        cal_print(&cal);
        *p_last = cal;
    }
}

static bool
cal_moved (uint16_t now, uint16_t was)
{
    int32_t d = (int32_t) now - (int32_t) was;

    return (d >= CAL_PRINT_STEP) || (d <= -CAL_PRINT_STEP);
}

/*!
 * @brief Refuse to start while S2's MF_ESTOP is latched. S3 does not clear
 *        it (MOTOR_S3_REFERENCE.md section 10): power-cycle the car.
 */
static bool
estop_clear_ok (void)
{
    subsystem2_status_t s2 = {0};

    if (!subsystem2_estop_is_set())
    {
        return true;
    }
    subsystem2_status_get(&s2);
    tm_printf((UB *) "S3 DRIVE NOT STARTED: S2 E-STOP is latched (stalls=%d, "
                     "e-stops=%d). S3 may not clear it: power-cycle the car "
                     "(Buddy 2 rule).\n", (INT) s2.stall_count,
              (INT) s2.estop_count);
    return false;
}

/*!
 * @brief One run: FOLLOW until a stop condition, then stop the motors.
 */
static void
drive_run (void)
{
    drive_preset_t const * p_set   = &g_presets[g_preset];
    s3_line_status_t       st      = {0};
    subsystem2_status_t    s2      = {0};
    uint32_t               start_mm  = s3_odom_mm();
    uint32_t               moved_mm  = 0u;
    uint32_t               ms        = 0u;
    uint32_t               lost_from = 0u;  /* moved_mm + 1 at LOST.     */
    uint32_t               lost_ms   = 0u;  /* Time spent LOST.          */
    uint32_t               still_ms  = 0u;  /* Time without odometry.    */
    uint32_t               last_mm   = 0u;
    char const *           p_why     = "?";
    ER                     er;

    er = subsystem2_follow(p_set->speed_mms);
    tm_printf((UB *) "S3 DRIVE GO preset %d: %d mm/s, kp=%d kd=%d, steer max "
                     "%d mm/s, edge %s (er=%d). Press BUTTON 1 to stop.\n",
              (INT) g_preset, (INT) p_set->speed_mms, (INT) p_set->kp_x100,
              (INT) p_set->kd_x100, (INT) p_set->steer_max_mms, S3_EDGE_TXT,
              (INT) er);
    for (;;)
    {
        tk_dly_tsk(POLL_MS);
        ms      += POLL_MS;
        moved_mm = s3_odom_mm() - start_mm;    /* Reversing wraps huge.  */
        if (button1_down())
        {
            p_why = "button";
            break;
        }
        still_ms = (moved_mm == last_mm) ? (still_ms + POLL_MS) : 0u;
        last_mm  = moved_mm;
        if (still_ms >= S3_DRIVE_STILL_MS)
        {
            p_why = "car not moving (no odometry change)";
            break;
        }
        if (subsystem2_estop_is_set())
        {
            p_why = "S2 E-STOP (wheel stall?), see S2 counters below";
            break;
        }
        /* LOST keeps the last (full) correction, so the car usually turns
         * back onto the line by itself; stop only if it stays lost, by
         * distance or, if it is not moving, by time. */
        if ((E_OK == s3_get_line_status(&st)) && (S3_LINE_LOST == st.state))
        {
            lost_ms += POLL_MS;
            if (0u == lost_from)
            {
                lost_from = moved_mm + 1u;
            }
            if (((moved_mm + 1u - lost_from) >= S3_DRIVE_LOST_MM) ||
                (lost_ms >= S3_DRIVE_LOST_MS))
            {
                p_why = "LINE LOST, not found again";
                break;
            }
        }
        else
        {
            lost_from = 0u;
            lost_ms   = 0u;
        }
        if (moved_mm >= S3_DRIVE_MAX_MM)
        {
            p_why = "distance limit";
            break;
        }
        if (ms >= S3_DRIVE_MAX_MS)
        {
            p_why = "time limit";
            break;
        }
    }
    (void) subsystem2_stop();
    subsystem2_status_get(&s2);
    tm_printf((UB *) "S3 DRIVE STOP (%s) after %d mm, %d ms. S2: stalls=%d "
                     "e-stops=%d discarded=%d\n",
              p_why, (INT) moved_mm, (INT) ms, (INT) s2.stall_count,
              (INT) s2.estop_count, (INT) s2.discarded_count);
    while (button1_down())          /* The stop press must not restart.  */
    {
        tk_dly_tsk(POLL_MS);
    }
    tk_dly_tsk(AFTER_STOP_MS);
}

/*!
 * @brief Wait for a SHORT Button 1 press (start). A hold of 1.5 ... 4 s
 *        selects the next preset when let go; 4 s recalibrates at once.
 *        Button 2 also selects the next preset.
 */
static void
wait_start (void)
{
    s3_cal_levels_t last      = {0};
    uint32_t        held      = 0u;
    uint32_t        cal_ms    = 0u;
    bool            b_recal   = false;  /* This hold recalibrated.       */
    bool            b_b2_was  = false;
    bool            b_b2;

    while (button1_down())          /* Let go of the last press first.  */
    {
        tk_dly_tsk(POLL_MS);
    }
    for (;;)
    {
        tk_dly_tsk(POLL_MS);
        cal_ms += POLL_MS;
        if (cal_ms >= CAL_PRINT_MS)
        {
            cal_ms = 0u;
            cal_watch(&last);
        }
        b_b2 = button2_down();
        if (b_b2 && (!b_b2_was))
        {
            preset_next();
        }
        b_b2_was = b_b2;

        if (button1_down())
        {
            held += POLL_MS;
            if (HOLD_MS == held)
            {
                tm_printf((UB *) "S3 DRIVE: let go now = next preset, keep "
                                 "holding = recalibrate\n");
            }
            if ((held >= S3_DRIVE_RECAL_MS) && (!b_recal))
            {
                b_recal = true;
                s3_request_recalibrate();
                tm_printf((UB *) "S3 DRIVE: RECALIBRATING. Let go, then sweep "
                                 "the car sideways across the line until "
                                 "\"S3 DRIVE CAL ... READY\".\n");
            }
        }
        else
        {
            if ((!b_recal) && (held >= HOLD_MS))
            {
                preset_next();
            }
            else if ((!b_recal) && (held >= PRESS_MS))
            {
                return;             /* Short press: start.              */
            }
            else
            {
                /* No press, or the end of a recalibrate hold. */
            }
            held    = 0u;
            b_recal = false;
        }
    }
}

static void
preset_next (void)
{
    g_preset = (g_preset + 1u) % N_PRESETS;
    preset_apply();
}

static void
preset_apply (void)
{
    drive_preset_t const * p_set = &g_presets[g_preset];

    s3_set_line_gains(p_set->kp_x100, p_set->kd_x100);
    s3_set_line_shape(0);
    s3_set_steer_max(p_set->steer_max_mms);
    tm_printf((UB *) "S3 DRIVE PRESET %d of %d: %d mm/s, kp=%d kd=%d, steer "
                     "max %d mm/s\n",
              (INT) g_preset, (INT) (N_PRESETS - 1u), (INT) p_set->speed_mms,
              (INT) p_set->kp_x100, (INT) p_set->kd_x100,
              (INT) p_set->steer_max_mms);
}

static void
button_input (uint32_t pin)
{
    (void) gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

static bool
button1_down (void)
{
    return (0u == gpio_get_val(S3_DRIVE_BTN1_PIN));
}

static bool
button2_down (void)
{
    return (0u == gpio_get_val(S3_DRIVE_BTN2_PIN));
}

#else /* S3_DRIVE_TEST */

/*!
 * @brief Drive test not built: nothing to start.
 */
ER
task_drive_test_start (void)
{
    return E_OK;
}

#endif /* S3_DRIVE_TEST */

/*** end of file ***/
