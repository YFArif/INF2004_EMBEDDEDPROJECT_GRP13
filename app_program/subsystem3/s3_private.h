/** @file s3_private.h
 *
 * @brief Subsystem 3 -- helpers shared between S3's own files only
 *        (s3_api.c, task_line.c, task_barcode.c, task_drive_test.c).
 *        Other subsystems must use s3_api.h.
 *
 * @par Who writes what
 * s3_api.c owns the kernel objects, the packed status, the steering word
 * and the tuning requests. The tasks change them only through the
 * functions below, which take mtx_s3 where the data is shared.
 */

#ifndef S3_PRIVATE_H
#define S3_PRIVATE_H

#include <stdbool.h>
#include <stdint.h>

#include "s3_api.h"
#include "s3_nav.h"

/** Calibration levels, raw ADC counts (drive test print / start check). */
typedef struct
{
    uint16_t white_o;           /* Outer ("Left IR") sensor.             */
    uint16_t black_o;
    uint16_t white_c;           /* Centre.                               */
    uint16_t black_c;
} s3_cal_levels_t;

/** What task_line reports after each step (into s3_line_status_t). */
typedef struct
{
    int32_t  steer_dv;          /* mm/s, -511 ... +511.                  */
    int32_t  pos;               /* -500 ... +500 permille.               */
    uint32_t state;             /* s3_line_state_t.                      */
    uint32_t seq;               /* 0 ... 255.                            */
    int32_t  n_left;            /* Left IR, permille.                    */
    int32_t  n_centre;          /* Centre, permille.                     */
    s3_cal_levels_t cal;        /* Current calibration levels.           */
    uint8_t  overshoot;
    uint8_t  junction_event;
    uint8_t  lost_event;
} s3_line_report_t;

/* Time and distance. */
uint32_t     s3_now_ms(void);
bool         s3_odom_read(uint32_t * p_mm);
uint32_t     s3_odom_mm(void);

/* task_line. */
void         s3_steer_publish(int32_t dv_mms, uint32_t seq, bool b_valid);
void         s3_status_set_line(s3_line_report_t const * p_rep);
bool         s3_gains_request_take(int32_t * p_kp_x100, int32_t * p_kd_x100);
bool         s3_shape_request_take(int32_t * p_boost_x100);
int32_t      s3_steer_max_get(void);
bool         s3_recal_request_take(void);

/* Test tasks (task_drive_test). */
void         s3_cal_levels_get(s3_cal_levels_t * p_cal);

/* task_barcode. */
s3_nav_cmd_t s3_nav_lookup(char ch);
void         s3_status_set_barcode(uint8_t ch_idx, uint8_t cmd, uint8_t rev);

/* Console text. */
char const * s3_state_txt(uint32_t state, uint8_t overshoot);

#endif /* S3_PRIVATE_H */

/*** end of file ***/
