/** @file s3_api.h
 *
 * @brief Subsystem 3 (Buddy 3: IR line following + barcode) -- PUBLIC
 *        interface. Other subsystems include "../subsystem3/subsystem3.h",
 *        which includes this header.
 *
 * @par Change Requirement 1 (minimum-bit data)
 * Every field below uses only the bits its range needs (bit-fields), and
 * every packed object shared between tasks has exactly one writer or is
 * accessed under a mutex -- see the "Concurrency" line of each item.
 *
 * @par Provided by S3 (all objects are created in subsystem3_init)
 *
 * IF-01  S3 -> Vehicle Controller: message buffer mbf_nav.
 *        One s3_nav_msg_t (4 bytes) per decoded barcode, COPIED, 4 deep.
 *        Receiver: s3_nav_msg_t m; tk_rcv_mbf(mbf_nav, &m, TMO_xxx);
 *                  char letter = s3_nav_msg_char(&m);
 *        Concurrency: kernel copy. Full -> S3 drops the newest (TMO_POL).
 *
 * IF-02  S3 -> Motion (Buddy 2): steer_sp, latest value wins.
 *        H dv; UB seq; if (s3_get_steer_sp(&dv, &seq)) { ... }
 *        dv = wheel speed DIFFERENCE in mm/s (+ = turn right), 10-bit
 *        signed; seq increments every 10 ms line step (stale if it stops).
 *        Concurrency: ONE aligned 32-bit word, single writer (task_line),
 *        read with one load -> atomic on the M0+, no lock, any core.
 *
 * IF-07  Vehicle Controller -> S3: event flag flg_line (owned by S3).
 *        LF_ENABLE    set = S3 steers; clear = turning / bypassing
 *                     (steer 0, no junction / lost events). Set at
 *                     power-up.
 *        LF_SEARCH_L  car is LEFT of the line, coming back to it.
 *        LF_SEARCH_R  car is RIGHT of the line, coming back to it.
 *        S3 sets FLG_S3_LINE_FOUND when it has locked onto the CORRECT
 *        edge (from the wrong side it first crosses the line) and keeps
 *        following even while LF_SEARCH_x is still set. ORDER for the
 *        Vehicle Controller: keep LF_ENABLE set, THEN clear LF_SEARCH_x
 *        (clearing both at once = DISABLED for a step). To search again,
 *        clear LF_SEARCH_x first, then set it.
 *
 * IF-S3c S3 -> Vehicle Controller ONLY: event flag flg_s3.
 *        FLG_S3_JUNCTION / LOST / BARCODE / LINE_FOUND, set by S3.
 *        ONE consumer: tk_wai_flg(..., TWF_ORW | TWF_BITCLR, ...).
 *        Everyone else reads the counters in s3_line_status_t.
 *
 * IF-S3b S3 -> anyone (telemetry): s3_line_status_t (16 bytes), getter.
 *        s3_line_status_t st; if (E_OK == s3_get_line_status(&st)) {...}
 *        Concurrency: written by task_line AND task_barcode, both under
 *        mtx_s3; the getter copies under mtx_s3. Never read the packed
 *        struct directly. Tasks only.
 *
 * IF-S3d Motion (Buddy 2) -> S3: s3_set_odometry(fn), fn() = total mm.
 * IF-S3e any task -> S3: s3_set_line_gains(kp, kd); s3_set_line_shape(boost)
 *        for the curve boost (s3_line.h); s3_set_steer_max(mm/s) for the
 *        steer_sp at full lock (set it near the base speed);
 *        s3_request_recalibrate();
 * IF-S3f Buddy 1 (MQTT) -> S3: s3_set_nav_map("DCBA") on the demo day:
 *        letters for LEFT, RIGHT, STRAIGHT, U-TURN. No recompilation.
 *
 * @par Names kept for subsystem 2
 * s3_get_steer_sp() and s3_set_odometry() are declared weak in
 * subsystem2.c with these exact signatures: do not change them.
 */

#ifndef S3_API_H
#define S3_API_H

#include "../app.h"

/* ---------------------------------------------------------------- IF-01 */
/** Navigation command: 5 values -> 3 bits. */
typedef enum
{
    S3_CMD_NONE     = 0,    /* Letter decoded but not in today's map.    */
    S3_CMD_LEFT     = 1,
    S3_CMD_RIGHT    = 2,
    S3_CMD_STRAIGHT = 3,
    S3_CMD_UTURN    = 4
} s3_cmd_t;

/** One decoded barcode: 32 bits (was 12 bytes). */
typedef struct
{
    UW cmd      : 3;    /* s3_cmd_t.                                     */
    UW ch_idx   : 6;    /* Code 39 index 0..43 (s3_nav_msg_char()).      */
    UW reversed : 1;    /* 1 = barcode crossed backwards.                */
    UW source   : 2;    /* 1 = analog + digital agree, 2 = analog only,  */
                        /* 3 = digital only.                             */
    UW seq      : 4;    /* 1..15, 0..: spots a dropped message.          */
    UW odom_mm  : 16;   /* Odometry at decode, mm, wraps every 65.5 m:   */
                        /* distance to the junction = odom_now - odom_mm. */
} s3_nav_msg_t;

extern ID mbf_nav;
char      s3_nav_msg_char(s3_nav_msg_t const * p_msg);

/* ---------------------------------------------------------------- IF-02 */
BOOL s3_get_steer_sp(H * p_dv_mms, UB * p_seq);  /* FALSE = not steering  */
                                                 /* (uncal / disabled /   */
                                                 /* search).              */

/* ---------------------------------------------------------------- IF-07 */
#define LF_ENABLE           (0x0001u)
#define LF_SEARCH_L         (0x0002u)
#define LF_SEARCH_R         (0x0004u)
extern ID flg_line;

/* --------------------------------------------------------------- IF-S3c */
#define FLG_S3_JUNCTION     (0x0001u)
#define FLG_S3_LOST         (0x0002u)
#define FLG_S3_BARCODE      (0x0004u)
#define FLG_S3_LINE_FOUND   (0x0008u)
extern ID flg_s3;

/* --------------------------------------------------------------- IF-S3b */
/** Line state as reported: 6 values -> 3 bits. */
typedef enum
{
    S3_LINE_FOLLOW   = 0,   /* Steering valid.                           */
    S3_LINE_JUNCTION = 1,   /* Junction under the car: steer held at 0.  */
    S3_LINE_LOST     = 2,   /* No line for 40 mm: steer = last value.    */
    S3_LINE_DISABLED = 3,   /* LF_ENABLE clear.                          */
    S3_LINE_SEARCH   = 4,   /* LF_SEARCH_x: waiting for the right edge.  */
    S3_LINE_UNCAL    = 5    /* Sensors not calibrated yet: do not drive. */
} s3_line_state_t;

/** Telemetry: 4 x 32 bits = 16 bytes (was 36). */
typedef struct
{
    /* Word 0 */
    signed int steer_dv  : 10;  /* mm/s, -511..+511 (same as steer_sp).  */
    signed int pos       : 10;  /* Line position, -500..+500 permille.   */
    UW         state     : 3;   /* s3_line_state_t.                      */
    UW         last_cmd  : 3;   /* s3_cmd_t of the last barcode.         */
    UW         last_rev  : 1;   /* It was read backwards.                */
    UW         overshoot : 1;   /* OUTER rule on (outer not white).      */
    UW         pad0      : 4;
    /* Word 1 */
    UW         n_outer   : 8;   /* Left IR, 0 = white .. 255 = black.    */
    UW         n_centre  : 8;   /* Centre, 0 = white .. 255 = black.     */
    UW         last_ch   : 6;   /* Code 39 index of the last letter,     */
                                /* 63 = none.                            */
    UW         seq       : 8;   /* +1 every line step (10 ms).           */
    UW         pad1      : 2;
    /* Word 2 */
    UW         map_left  : 6;   /* Today's letters (Code 39 indices).    */
    UW         map_right : 6;
    UW         map_str   : 6;
    UW         map_uturn : 6;
    UW         junctions : 8;   /* Counters wrap at 256: use differences. */
    /* Word 3 */
    UW         losts     : 8;
    UW         barcodes  : 8;
    UW         updated_ms: 16;  /* System time, ms, wraps every 65.5 s.  */
} s3_line_status_t;

extern ID mtx_s3;
ER        s3_get_line_status(s3_line_status_t * p_status);  /* Tasks only. */

/* --------------------------------------------------------- IF-S3d/e/f */
typedef UW (* s3_odom_fn_t)(void);
void s3_set_odometry(s3_odom_fn_t fn);
void s3_set_line_gains(INT kp_x100, INT kd_x100);   /* e.g. 150, 400.     */
void s3_set_line_shape(INT boost_x100);             /* 0 = off (default). */
void s3_set_steer_max(INT dv_mms);                  /* 1 ... 511 mm/s.    */
void s3_request_recalibrate(void);
ER   s3_set_nav_map(char const * p_map);            /* Tasks only.        */

/* ------------------------------------------------- called by subsystem3.c */
ER   s3_init(void);
ER   s3_start(void);

#endif /* S3_API_H */

/*** end of file ***/
