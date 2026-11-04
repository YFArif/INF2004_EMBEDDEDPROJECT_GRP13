/** @file task_ir_view.c
 * @brief Stationary IR height/layout check. No movement commands.
 *
 * ADC snapshots are read on core 1. Reference values are captured only
 * on request, not continuously learned, so movement cannot shift them.
 */
#include "s3_experiment.h"

#if (S3_EXPERIMENT_MODE == 3)
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/drv_button.h"
#include "drv_adc.h"
#include "s3_config.h"
#include "s3_private.h"
#include "s3_ir_zone.h"

#define VIEW_POLL_MS (20u)
#define VIEW_SAMPLES (25u)

typedef struct
{
    uint32_t sum;
    uint16_t low;
    uint16_t high;
} view_window_t;

static void view_task(INT stacd, void * p_exinf);
static void window_add(view_window_t * p_window, uint16_t raw);
static int32_t normalise(uint16_t raw, uint16_t white, uint16_t black);
static char const * zone_name(s3_ir_zone_t zone);
static void zone_report(char const * p_sensor, uint16_t raw, int32_t norm,
                         s3_ir_zone_t * p_previous);

ER
task_probe_start (void)
{
    ID tid = app_start_task((FP) view_task, S3_PRI_LINE, TP_PRC2,
                            "s3_ir_view");

    return (tid < E_OK) ? tid : E_OK;
}

static void
view_task (INT stacd, void * p_exinf)
{
    view_window_t left = {0u, 4095u, 0u};
    view_window_t right = {0u, 4095u, 0u};
    uint16_t raw_left;
    uint16_t raw_right;
    uint16_t mean_left;
    uint16_t mean_right;
    uint16_t white_left = 0u;
    uint16_t white_right = 0u;
    uint16_t black_left = 0u;
    uint16_t black_right = 0u;
    uint32_t count = 0u;
    uint32_t sequence = 0u;
    uint32_t adc_previous;
    bool b_white = false;
    bool b_black = false;
    bool b_capture_white = false;
    bool b_capture_black = false;
    int32_t norm_left = -1;
    int32_t norm_right = -1;
    s3_ir_zone_t previous_left = S3_IR_ZONE_INVALID;
    s3_ir_zone_t previous_right = S3_IR_ZONE_INVALID;

    (void) stacd;
    (void) p_exinf;
    CHECK(subsystem2_stop(), "IR view STOP");
    /* A latched E-stop prevents accidental motion throughout this image. */
    subsystem2_estop_set();
    drv_button_init();
    CHECK(tk_dly_tsk(3000), "IR view USB settle");
    adc_previous = drv_adc_sample_index();
    tm_printf((UB *) "\nIR VIEW v31 GREY ALERT - MOTORS LOCKED BY E-STOP\n"
                     "LEFT=GP27 ADC1; RIGHT=GP26 ADC0. Higher raw=darker.\n"
                     "Keep sensors at the NEW operating height throughout.\n"
                     "1 BOTH over WHITE: hold still, tap/release GP20.\n"
                     "2 BOTH fully over BLACK: hold still, tap/release GP21.\n"
                     "   A wide black patch can cover both; do not lift car.\n"
                     "3 GP27 at LEFT edge; GP26 outside RIGHT edge.\n"
                     "   Slide sideways slowly; send the IR log.\n"
                     "Buttons capture the NEXT 0.5 s window; keep still.\n"
                     "n=-1 means missing/invalid references; NOT white.\n"
                     "WHITE n<250; GREY 250..750; BLACK n>750.\n"
                     "Zone changes checked every 20 ms, using averaged ADC.\n"
                     "GREY means intermediate reflectance, not proof of edge.\n");
    for (;;)
    {
        CHECK(tk_dly_tsk(VIEW_POLL_MS), "IR view poll");
        drv_adc_ir_read(&raw_right, &raw_left);
        if (DRV_BUTTON_PRESSED ==
            drv_button_update(DRV_BUTTON_START, VIEW_POLL_MS))
        {
            b_capture_white = true;
            b_white = false;
            b_capture_black = false;
            count = 0u;
            left = (view_window_t) {0u, 4095u, 0u};
            right = left;
            tm_printf((UB *) "CAPTURE WHITE: keep still for 0.5 s.\n");
        }
        if (DRV_BUTTON_PRESSED ==
            drv_button_update(DRV_BUTTON_CAL, VIEW_POLL_MS))
        {
            b_capture_black = true;
            b_black = false;
            b_capture_white = false;
            count = 0u;
            left = (view_window_t) {0u, 4095u, 0u};
            right = left;
            tm_printf((UB *) "CAPTURE BLACK: keep still for 0.5 s.\n");
        }
        /* Auto-reference: lowest/highest reading seen since boot, so no
         * button press is needed. Readings >= lift limit (no floor) are
         * ignored. Valid once both sensors span >= 600 counts. */
        if (!b_manual)
        {
            if (raw_left < S3_IR_CAL_LIFT_RAW)
            {
                white_left = (raw_left < white_left) ? raw_left : white_left;
                black_left = (raw_left > black_left) ? raw_left : black_left;
            }
            if (raw_right < S3_IR_CAL_LIFT_RAW)
            {
                white_right = (raw_right < white_right) ? raw_right : white_right;
                black_right = (raw_right > black_right) ? raw_right : black_right;
            }
            b_white = ((int32_t) black_left - white_left >=
                       (int32_t) S3_IR_CAL_MIN_RANGE) &&
                      ((int32_t) black_right - white_right >=
                       (int32_t) S3_IR_CAL_MIN_RANGE);
            b_black = b_white;
            if (b_white && !b_auto_reported)
            {
                b_auto_reported = true;
                tm_printf((UB *) "AUTO CAL VALID: L white %d black %d | "
                                 "R white %d black %d\n",
                          (INT) white_left, (INT) black_left,
                          (INT) white_right, (INT) black_right);
            }
        }
        window_add(&left, raw_left);
        window_add(&right, raw_right);
        norm_left = (b_white && b_black) ?
                    normalise(raw_left, white_left, black_left) : -1;
        norm_right = (b_white && b_black) ?
                     normalise(raw_right, white_right, black_right) : -1;
        if (drv_adc_sample_index() == adc_previous)
        {
            norm_left = -1;
            norm_right = -1;
        }
        zone_report("LEFT GP27", raw_left, norm_left, &previous_left);
        zone_report("RIGHT GP26", raw_right, norm_right, &previous_right);
        if (++count < VIEW_SAMPLES)
        {
            continue;
        }
        mean_left = (uint16_t) (left.sum / VIEW_SAMPLES);
        mean_right = (uint16_t) (right.sum / VIEW_SAMPLES);
        if (drv_adc_sample_index() == adc_previous)
        {
            tm_printf((UB *) "ERROR: ADC sampler stopped; readings invalid.\n");
            b_white = false;
            b_black = false;
            b_capture_white = false;
            b_capture_black = false;
        }
        else if (b_capture_white)
        {
            white_left = mean_left;
            white_right = mean_right;
            b_white = true;
            b_capture_white = false;
            tm_printf((UB *) "WHITE SAVED L=%d R=%d\n",
                      (INT) white_left, (INT) white_right);
        }
        else if (b_capture_black)
        {
            black_left = mean_left;
            black_right = mean_right;
            b_black = true;
            b_capture_black = false;
            tm_printf((UB *) "BLACK SAVED L=%d R=%d\n",
                      (INT) black_left, (INT) black_right);
        }
        adc_previous = drv_adc_sample_index();
        norm_left = (b_white && b_black) ?
                    normalise(raw_left, white_left, black_left) : -1;
        norm_right = (b_white && b_black) ?
                     normalise(raw_right, white_right, black_right) : -1;
        tm_printf((UB *) "IR #%d L_GP27 %s now=%d n=%d mean=%d min=%d max=%d | "
                         "R_GP26 %s now=%d n=%d mean=%d min=%d max=%d\n",
                  (INT) ++sequence, zone_name(s3_ir_zone(norm_left)),
                  (INT) raw_left, (INT) norm_left, (INT) mean_left,
                  (INT) left.low, (INT) left.high,
                  zone_name(s3_ir_zone(norm_right)),
                  (INT) raw_right, (INT) norm_right, (INT) mean_right,
                  (INT) right.low, (INT) right.high);
        if (b_white && b_black &&
            (((int32_t) black_left - white_left < (int32_t) S3_IR_CAL_MIN_RANGE) ||
             ((int32_t) black_right - white_right < (int32_t) S3_IR_CAL_MIN_RANGE)))
        {
            tm_printf((UB *) "INVALID CONTRAST: L=%d R=%d; need >=600 each. "
                             "Recheck placement/height and recapture.\n",
                      (INT) black_left - (INT) white_left,
                      (INT) black_right - (INT) white_right);
        }
        count = 0u;
        left = (view_window_t) {0u, 4095u, 0u};
        right = left;
    }
}

static void
window_add (view_window_t * p_window, uint16_t raw)
{
    p_window->sum += raw;
    if (raw < p_window->low)
    {
        p_window->low = raw;
    }
    if (raw > p_window->high)
    {
        p_window->high = raw;
    }
}

static char const *
zone_name (s3_ir_zone_t zone)
{
    switch (zone)
    {
        case S3_IR_ZONE_WHITE: return "WHITE";
        case S3_IR_ZONE_GREY: return "GREY";
        case S3_IR_ZONE_BLACK: return "BLACK";
        default: return "UNCAL/INVALID";
    }
}

static void
zone_report (char const * p_sensor, uint16_t raw, int32_t norm,
              s3_ir_zone_t * p_previous)
{
    s3_ir_zone_t zone = s3_ir_zone(norm);

    if (zone != *p_previous)
    {
        tm_printf((UB *) "%s t=%d %s -> %s raw=%d n=%d\n",
                  (S3_IR_ZONE_GREY == zone) ? ">>> GREY DETECTED" : "IR CHANGE",
                  (INT) s3_now_ms(), p_sensor, zone_name(zone),
                  (INT) raw, (INT) norm);
        *p_previous = zone;
    }
}

static int32_t
normalise (uint16_t raw, uint16_t white, uint16_t black)
{
    int32_t range = (int32_t) black - white;
    int32_t value;

    if (range < (int32_t) S3_IR_CAL_MIN_RANGE)
    {
        return -1; /* Console sentinel, accompanied by explicit error. */
    }
    value = (((int32_t) raw - white) * 1000) / range;
    return (value < 0) ? 0 : ((value > 1000) ? 1000 : value);
}
#endif
