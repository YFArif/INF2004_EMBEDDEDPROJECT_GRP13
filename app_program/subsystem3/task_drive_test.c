/** @file task_drive_test.c
 *
 * @brief Subsystem 3 -- TEST ONLY (S3_DRIVE_TEST 1): line following on
 *        Buddy 2's motors before the Vehicle Controller exists.
 *
 * @par How to use
 * Sweep the car sideways across the line until it is calibrated, put it
 * on the line, press Button 1 (GP20). The car follows at the preset's
 * speed and stops on Button 1 again, after S3_DRIVE_LOST_MM of LINE
 * LOST, or at the distance / time limit. While waiting, Button 2 (GP21)
 * selects the next gain preset. Both buttons are only read, with the pad
 * pull-up, as in subsystem2_step.c.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../subsystem2/subsystem2.h"   /* subsystem2_follow / _stop.   */

#include "s3_api.h"
#include "s3_config.h"
#include "s3_private.h"
#include "task_drive_test.h"

#if (1 == S3_DRIVE_TEST)

#define POLL_MS             (20u)
#define PRESS_MS            (60u)   /* Held this long = a real press.   */
#define SETTLE_MS           (100)
#define COUNTDOWN_S         (3u)
#define COUNTDOWN_STEP_MS   (1000)
#define AFTER_STOP_MS       (500)
#define U8_WHITE_BELOW      (64u)   /* s3_line_status_t n_xxx: 0..255.  */
#define U8_BLACK_ABOVE      (191u)
#define PERMILLE            (1000u)
#define U8_FULL             (255u)

/** Gain preset: Kp x100, Kd x100, speed mm/s. */
typedef struct
{
    int16_t  kp_x100;
    int16_t  kd_x100;
    uint16_t speed_mms;
} drive_preset_t;

static drive_preset_t const g_presets[] =
{
    {100, 300, 150}, {100, 300, 120}, {100, 300, 100},
    {150, 400, 100}, {150, 400,  80}
};

#define N_PRESETS   (sizeof(g_presets) / sizeof(g_presets[0]))

/* 100 mm/s: at 150 every run weaved off within ~0.5 m (26 Oct). */
static uint32_t g_preset = 2u;

static void drive_task(INT stacd, void * p_exinf);
static bool button1_down(void);
static bool button2_down(void);
static void button_input(uint32_t pin);
static void preset_apply(void);
static void wait_press(void);
static bool start_allowed(void);
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
        tm_printf((UB *) "S3 DRIVE: sweep the car sideways across the line "
                         "until calibrated, put the car on the line (Centre "
                         "on the line or its %s edge), then press BUTTON 1 "
                         "(GP20). BUTTON 2 = next gain preset.\n",
                  S3_EDGE_TXT);
        wait_press();
        for (i = COUNTDOWN_S; i > 0u; i--)
        {
            tm_printf((UB *) "S3 DRIVE %d\n", (INT) i);
            tk_dly_tsk(COUNTDOWN_STEP_MS);
        }
        if (start_allowed())
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
 * @brief One run: FOLLOW until a stop condition, then stop the motors.
 */
static void
drive_run (void)
{
    s3_line_status_t st        = {0};
    uint32_t         start_mm  = s3_odom_mm();
    uint32_t         moved_mm  = 0u;
    uint32_t         ms        = 0u;
    uint32_t         lost_from = 0u;   /* moved_mm + 1 when LOST began. */
    char const *     p_why     = "?";
    ER               er;

    er = subsystem2_follow(g_presets[g_preset].speed_mms);
    tm_printf((UB *) "S3 DRIVE GO %d mm/s (er=%d). Press BUTTON 1 to stop.\n",
              (INT) g_presets[g_preset].speed_mms, (INT) er);
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
        /* LOST keeps the last (full) correction, so the car usually turns
         * back onto the line by itself; stop only if it stays lost. */
        if ((E_OK == s3_get_line_status(&st)) && (S3_LINE_LOST == st.state))
        {
            if (0u == lost_from)
            {
                lost_from = moved_mm + 1u;
            }
            else if ((moved_mm + 1u - lost_from) >= S3_DRIVE_LOST_MM)
            {
                p_why = "LINE LOST, not found again";
                break;
            }
            else
            {
                ;   /* Still lost, keep going. */
            }
        }
        else
        {
            lost_from = 0u;
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
    tm_printf((UB *) "S3 DRIVE STOP (%s) after %d mm, %d ms\n",
              p_why, (INT) moved_mm, (INT) ms);
    tk_dly_tsk(AFTER_STOP_MS);
}

/*!
 * @brief Wait for Button 1; every Button 2 press selects the next preset.
 */
static void
wait_press (void)
{
    uint32_t held     = 0u;
    bool     b_b2_was = false;
    bool     b_b2;

    while (button1_down())          /* Let go of the last press first.  */
    {
        tk_dly_tsk(POLL_MS);
    }
    while (held < PRESS_MS)
    {
        tk_dly_tsk(POLL_MS);
        held = button1_down() ? (held + POLL_MS) : 0u;
        b_b2 = button2_down();
        if (b_b2 && (!b_b2_was))
        {
            g_preset = (g_preset + 1u) % N_PRESETS;
            preset_apply();
        }
        b_b2_was = b_b2;
    }
    while (button1_down())
    {
        tk_dly_tsk(POLL_MS);
    }
}

static void
preset_apply (void)
{
    s3_set_line_gains(g_presets[g_preset].kp_x100,
                      g_presets[g_preset].kd_x100);
    tm_printf((UB *) "S3 DRIVE PRESET %d: %d mm/s, kp=%d kd=%d (x100)\n",
              (INT) g_preset, (INT) g_presets[g_preset].speed_mms,
              (INT) g_presets[g_preset].kp_x100,
              (INT) g_presets[g_preset].kd_x100);
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
