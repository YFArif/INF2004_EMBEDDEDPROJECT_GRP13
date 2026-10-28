/** @file s3_api.c
 *
 * @brief Subsystem 3 -- kernel objects, the public functions of s3_api.h
 *        and the shared helpers of s3_private.h.
 *
 * @par
 * This file owns everything that is shared between S3's tasks or with
 * other subsystems: mbf_nav, mtx_s3, flg_s3, flg_line, the packed status
 * (always under mtx_s3), the steering word (one writer, one 32-bit store)
 * and the tuning requests (handed over under mtx_s3).
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#include "s3_api.h"
#include "s3_code39.h"
#include "s3_config.h"
#include "s3_line.h"
#include "s3_nav.h"
#include "s3_private.h"
#include "task_barcode.h"

/* IF-02 steer_sp: one aligned 32-bit word, single writer (task_line).
 * Bits 0-9 dv (10-bit two's complement, mm/s), 10-17 seq, 18 valid. */
#define SP_DV_MASK      (0x3FFu)
#define SP_DV_SIGN      (0x200)
#define SP_DV_WRAP      (0x400)
#define SP_SEQ_POS      (10u)
#define SP_SEQ_MASK     (0xFFu)
#define SP_VALID_BIT    (1u << 18)

#define U8_MASK         (0xFFu)
#define U16_MASK        (0xFFFFu)
#define PERMILLE        (1000)
#define U8_FULL         (255)

/* Change Requirement 1: the packed sizes are part of the interface. */
typedef char s3_chk_nav_t[(sizeof(s3_nav_msg_t) == 4u) ? 1 : -1];
typedef char s3_chk_status_t[(sizeof(s3_line_status_t) == 16u) ? 1 : -1];
typedef char s3_chk_line_t[(sizeof(s3_line_t) <= 48u) ? 1 : -1];

ID mbf_nav;
ID mtx_s3;
ID flg_s3;
ID flg_line;

static s3_line_status_t      g_status;      /* Always under mtx_s3.     */
static s3_cal_levels_t       g_cal;         /* Always under mtx_s3.     */
static volatile s3_odom_fn_t g_odom_fn;     /* Set once by S2.          */
static volatile uint32_t     g_steer_word;  /* IF-02, see above.        */

/* Tuning requests: written by any task, taken by task_line. */
static int32_t       g_req_kp;
static int32_t       g_req_kd;
static volatile bool g_b_req_gains;         /* Set and cleared under    */
                                            /* mtx_s3; peeked without.  */
static volatile bool g_b_req_recal;         /* One flag, one byte.      */
static int32_t       g_req_boost;
static volatile bool g_b_req_shape;         /* Like g_b_req_gains.      */
static volatile int32_t g_steer_max_mms = S3_STEER_MAX_MMS; /* One     */
                                            /* aligned 32-bit store.    */

static void status_map_update(void);

/*!
 * @brief Create mbf_nav, mtx_s3, flg_s3 and flg_line; reset the map.
 *
 * @return E_OK or the first kernel error.
 */
ER
s3_init (void)
{
    T_CMBF cmbf = {0};
    T_CMTX cmtx = {0};
    T_CFLG cflg = {0};

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (INT) (S3_NAV_MBF_DEPTH *
                         (sizeof(s3_nav_msg_t) + S3_NAV_MBF_HEADER));
    cmbf.maxmsz = (INT) sizeof(s3_nav_msg_t);
    mbf_nav = tk_cre_mbf(&cmbf);
    if (mbf_nav < E_OK)
    {
        return mbf_nav;
    }

    cmtx.mtxatr = TA_INHERIT;
    mtx_s3 = tk_cre_mtx(&cmtx);
    if (mtx_s3 < E_OK)
    {
        return mtx_s3;
    }

    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0u;
    flg_s3 = tk_cre_flg(&cflg);
    if (flg_s3 < E_OK)
    {
        return flg_s3;
    }
    cflg.iflgptn = LF_ENABLE;       /* Steer from power-up until the VC  */
    flg_line = tk_cre_flg(&cflg);   /* says otherwise.                   */
    if (flg_line < E_OK)
    {
        return flg_line;
    }

    s3_nav_reset();
    g_status.state   = S3_LINE_UNCAL;
    g_status.last_ch = S3_C39_IDX_NONE;
    status_map_update();
    return E_OK;
}

/*!
 * @brief Start S3's tasks: task_barcode sets up the hardware on core 1,
 *        then starts task_line (and the drive test, if built).
 */
ER
s3_start (void)
{
    return task_barcode_start();
}

/*!
 * @brief Copy of the telemetry (IF-S3b). Tasks only.
 *
 * @return E_OK, or the mutex error (E_CTX from a handler).
 */
ER
s3_get_line_status (s3_line_status_t * p_status)
{
    ER er = tk_loc_mtx(mtx_s3, TMO_FEVR);

    if (E_OK != er)
    {
        return er;
    }
    *p_status = g_status;
    (void) tk_unl_mtx(mtx_s3);
    return E_OK;
}

/*!
 * @brief IF-02: latest steering. Any core, any context: one 32-bit load.
 *
 * @return TRUE while S3 is steering (FOLLOW, JUNCTION or LOST).
 */
BOOL
s3_get_steer_sp (H * p_dv_mms, UB * p_seq)
{
    uint32_t word = g_steer_word;   /* ONE load: dv, seq and valid match. */
    int32_t  dv   = (int32_t) (word & SP_DV_MASK);

    if (0 != (dv & SP_DV_SIGN))
    {
        dv -= SP_DV_WRAP;           /* Sign-extend 10 bits.              */
    }
    *p_dv_mms = (H) dv;
    *p_seq    = (UB) ((word >> SP_SEQ_POS) & SP_SEQ_MASK);
    return (0u != (word & SP_VALID_BIT)) ? TRUE : FALSE;
}

/*!
 * @brief The letter of a navigation message, e.g. 'A'.
 */
char
s3_nav_msg_char (s3_nav_msg_t const * p_msg)
{
    return s3_code39_char((uint8_t) p_msg->ch_idx);
}

/*!
 * @brief IF-S3e: new PD gains (x100). task_line applies them at its next
 *        step, so s3_line_t is only ever touched by task_line. Negative
 *        values are ignored.
 */
void
s3_set_line_gains (INT kp_x100, INT kd_x100)
{
    if ((kp_x100 < 0) || (kd_x100 < 0))
    {
        return;
    }
    if (E_OK == tk_loc_mtx(mtx_s3, TMO_FEVR))
    {
        g_req_kp      = kp_x100;
        g_req_kd      = kd_x100;
        g_b_req_gains = true;
        (void) tk_unl_mtx(mtx_s3);
    }
}

/*!
 * @brief IF-S3e: curve boost (x100, 0 = off). task_line applies it at
 *        its next step. Negative values are ignored.
 */
void
s3_set_line_shape (INT boost_x100)
{
    if (boost_x100 < 0)
    {
        return;
    }
    if (E_OK == tk_loc_mtx(mtx_s3, TMO_FEVR))
    {
        g_req_boost   = boost_x100;
        g_b_req_shape = true;
        (void) tk_unl_mtx(mtx_s3);
    }
}

/*!
 * @brief IF-S3e: steer_sp (wheel-speed difference) at full lock, mm/s,
 *        clamped to 1 ... S3_STEER_LIMIT_MMS. Keep it close to the base
 *        speed: S2 applies 50 %, so full lock gives base -/+ half of it
 *        and the inner wheel never stops or reverses (a stopped inner
 *        wheel at high duty trips S2's stall E-stop).
 */
void
s3_set_steer_max (INT dv_mms)
{
    int32_t v = (int32_t) dv_mms;

    v = (v < 1) ? 1 : ((v > S3_STEER_LIMIT_MMS) ? S3_STEER_LIMIT_MMS : v);
    g_steer_max_mms = v;
}

/*!
 * @brief IF-S3f: today's letter map, e.g. "DCBA" (LEFT, RIGHT, STRAIGHT,
 *        U-TURN). An invalid map is rejected and the old one kept.
 *
 * @return E_OK, E_PAR (rejected) or the mutex error.
 */
ER
s3_set_nav_map (char const * p_map)
{
    int  rc;
    char map[S3_NAV_MAP_LEN + 1u];
    ER   er = tk_loc_mtx(mtx_s3, TMO_FEVR);

    if (E_OK != er)
    {
        return er;
    }
    rc = s3_nav_set_map(p_map);
    status_map_update();
    s3_nav_get_map(map);
    (void) tk_unl_mtx(mtx_s3);
    tm_printf((UB *) "S3 NAV MAP %s: LEFT=%c RIGHT=%c STRAIGHT=%c U-TURN=%c\n",
              (0 == rc) ? "set" : "REJECTED (kept)",
              map[0], map[1], map[2], map[3]);
    return (0 == rc) ? E_OK : E_PAR;
}

/*!
 * @brief IF-S3e: forget both calibrations at the next line step.
 */
void
s3_request_recalibrate (void)
{
    g_b_req_recal = true;
}

/*!
 * @brief IF-S3d: S2 hands over its odometry (total mm since power-up).
 */
void
s3_set_odometry (s3_odom_fn_t fn)
{
    g_odom_fn = fn;
}

/* ------------------------------------------------- s3_private.h helpers */

/*!
 * @brief System time in ms (low 32 bits).
 */
uint32_t
s3_now_ms (void)
{
    SYSTIM now;

    (void) tk_get_tim(&now);
    return (uint32_t) now.lo;
}

/*!
 * @brief Read S2's odometry.
 *
 * @return false if S2 has not called s3_set_odometry() yet.
 */
bool
s3_odom_read (uint32_t * p_mm)
{
    s3_odom_fn_t fn = g_odom_fn;

    if (NULL == fn)
    {
        return false;
    }
    *p_mm = fn();
    return true;
}

/*!
 * @brief S2's odometry in mm, or 0 before S2 has handed it over.
 */
uint32_t
s3_odom_mm (void)
{
    uint32_t mm = 0u;

    (void) s3_odom_read(&mm);
    return mm;
}

/*!
 * @brief task_line only: publish IF-02 in one 32-bit store.
 */
void
s3_steer_publish (int32_t dv_mms, uint32_t seq, bool b_valid)
{
    g_steer_word = ((uint32_t) dv_mms & SP_DV_MASK) |
                   ((seq & SP_SEQ_MASK) << SP_SEQ_POS) |
                   (b_valid ? SP_VALID_BIT : 0u);
}

/*!
 * @brief task_line only: its part of the status, under mtx_s3.
 */
void
s3_status_set_line (s3_line_report_t const * p_rep)
{
    (void) tk_loc_mtx(mtx_s3, TMO_FEVR);
    g_status.steer_dv   = p_rep->steer_dv;
    g_status.pos        = p_rep->pos;
    g_status.state      = p_rep->state;
    g_status.overshoot  = p_rep->overshoot;
    g_status.n_outer    = (uint32_t) ((p_rep->n_left * U8_FULL) / PERMILLE);
    g_status.n_centre   = (uint32_t) ((p_rep->n_centre * U8_FULL) / PERMILLE);
    g_status.seq        = p_rep->seq & U8_MASK;
    g_status.junctions  = (g_status.junctions + p_rep->junction_event) &
                          U8_MASK;
    g_status.losts      = (g_status.losts + p_rep->lost_event) & U8_MASK;
    g_status.updated_ms = s3_now_ms() & U16_MASK;
    g_cal               = p_rep->cal;
    (void) tk_unl_mtx(mtx_s3);
}

/*!
 * @brief Test tasks: the calibration levels of the last line step.
 */
void
s3_cal_levels_get (s3_cal_levels_t * p_cal)
{
    (void) tk_loc_mtx(mtx_s3, TMO_FEVR);
    *p_cal = g_cal;
    (void) tk_unl_mtx(mtx_s3);
}

/*!
 * @brief task_line only: take a pending gain request.
 *
 * @return true if there was one (then *p_kp_x100 and *p_kd_x100 are set).
 */
bool
s3_gains_request_take (int32_t * p_kp_x100, int32_t * p_kd_x100)
{
    bool b_take = false;

    /* Called every 10 ms but a request comes perhaps once a run: peek at
     * the flag first and take the mutex only when it is set. A request
     * set just after the peek is taken at the next step. */
    if (!g_b_req_gains)
    {
        return false;
    }
    (void) tk_loc_mtx(mtx_s3, TMO_FEVR);
    if (g_b_req_gains)
    {
        b_take        = true;
        *p_kp_x100    = g_req_kp;
        *p_kd_x100    = g_req_kd;
        g_b_req_gains = false;
    }
    (void) tk_unl_mtx(mtx_s3);
    return b_take;
}

/*!
 * @brief task_line only: take a pending shape request (peek, then lock).
 *
 * @return true if there was one (then *p_boost_x100 is set).
 */
bool
s3_shape_request_take (int32_t * p_boost_x100)
{
    bool b_take = false;

    if (!g_b_req_shape)
    {
        return false;
    }
    (void) tk_loc_mtx(mtx_s3, TMO_FEVR);
    if (g_b_req_shape)
    {
        b_take        = true;
        *p_boost_x100 = g_req_boost;
        g_b_req_shape = false;
    }
    (void) tk_unl_mtx(mtx_s3);
    return b_take;
}

/*!
 * @brief task_line: the current steer_sp at full lock, mm/s.
 */
int32_t
s3_steer_max_get (void)
{
    return g_steer_max_mms;
}

/*!
 * @brief task_line only: take a pending recalibration request.
 */
bool
s3_recal_request_take (void)
{
    if (!g_b_req_recal)
    {
        return false;
    }
    g_b_req_recal = false;
    return true;
}

/*!
 * @brief task_barcode: today's command for a letter (under mtx_s3, as
 *        s3_set_nav_map() may change the map at any time).
 */
s3_nav_cmd_t
s3_nav_lookup (char ch)
{
    s3_nav_cmd_t cmd;

    (void) tk_loc_mtx(mtx_s3, TMO_FEVR);
    cmd = s3_nav_from_char(ch);
    (void) tk_unl_mtx(mtx_s3);
    return cmd;
}

/*!
 * @brief task_barcode only: its part of the status, under mtx_s3.
 */
void
s3_status_set_barcode (uint8_t ch_idx, uint8_t cmd, uint8_t rev)
{
    (void) tk_loc_mtx(mtx_s3, TMO_FEVR);
    g_status.last_ch  = ch_idx;
    g_status.last_cmd = cmd;
    g_status.last_rev = rev;
    g_status.barcodes = (g_status.barcodes + 1u) & U8_MASK;
    (void) tk_unl_mtx(mtx_s3);
}

/*!
 * @brief Console text for a line state.
 */
char const *
s3_state_txt (uint32_t state, uint8_t overshoot)
{
    char const * p_txt;

    switch (state)
    {
        case S3_LINE_FOLLOW:
            p_txt = (0u != overshoot) ? "FOLLOW (outer near line)" : "FOLLOW";
            break;
        case S3_LINE_JUNCTION:
            p_txt = "JUNCTION";
            break;
        case S3_LINE_LOST:
            p_txt = "LOST";
            break;
        case S3_LINE_DISABLED:
            p_txt = "DISABLED (LF_ENABLE clear)";
            break;
        case S3_LINE_SEARCH:
            p_txt = "SEARCH";
            break;
        default:
            p_txt = "UNCALIBRATED (sweep both sensors over white and black)";
            break;
    }
    return p_txt;
}

/* Copy the current letter map into the status. Caller holds mtx_s3, or
 * runs before any task exists (s3_init). */
static void
status_map_update (void)
{
    char map[S3_NAV_MAP_LEN + 1u];

    s3_nav_get_map(map);
    g_status.map_left  = s3_code39_index(map[0]);
    g_status.map_right = s3_code39_index(map[1]);
    g_status.map_str   = s3_code39_index(map[2]);
    g_status.map_uturn = s3_code39_index(map[3]);
}

/*** end of file ***/
