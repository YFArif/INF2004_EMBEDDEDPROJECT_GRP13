/*
 *	s3_api.h
 *	Subsystem 3 (Buddy 3: IR line following + barcode) -- PUBLIC interface.
 *	Other subsystems include "../subsystem3/subsystem3.h" (which includes this).
 *
 *	Change Requirement 1 (minimum-bit data): every field below uses only the
 *	bits its range needs (bit-fields), and every packed object shared between
 *	tasks has exactly one writer or is accessed under a mutex -- see the
 *	"Concurrency" line of each item.
 *
 *	Provided by S3 (all objects are created in subsystem3_init):
 *
 *	IF-01   S3 -> Vehicle Controller       message buffer  mbf_nav
 *	        One S3_NAV_MSG (4 bytes) per decoded barcode, COPIED, 4 deep.
 *	        Receiver: S3_NAV_MSG m; tk_rcv_mbf(mbf_nav, &m, TMO_xxx);
 *	        char  letter = s3_nav_msg_char(&m);
 *	        (Week 6 named this mbx_nav / mailbox; a message buffer copies the
 *	        4 bytes, so there is no buffer-lifetime rule for the receiver.)
 *	        Concurrency: kernel copy.  Full -> S3 drops the newest (TMO_POL).
 *
 *	IF-02   S3 -> Motion (Buddy 2)          steer_sp, latest value wins
 *	        H dv; UB seq; if (s3_get_steer_sp(&dv, &seq)) { ... }
 *	        dv = wheel speed difference in mm/s (+ = turn right), 10-bit
 *	        signed; seq increments every 10 ms line step (stale if it stops).
 *	        Concurrency: ONE aligned 32-bit word, single writer (line task),
 *	        read with one load -> atomic on the M0+, no lock, any core.
 *
 *	IF-07   Vehicle Controller -> S3        event flag  flg_line (owned by S3)
 *	        LF_ENABLE    set = S3 steers; clear = turning/bypassing (steer 0,
 *	                     no junction/lost events).  Set at power-up (bench).
 *	        LF_SEARCH_L  car is LEFT of the line, coming back to it.
 *	        LF_SEARCH_R  car is RIGHT of the line, coming back to it.
 *	        S3 sets FLG_S3_LINE_FOUND when it has locked onto the CORRECT edge
 *	        (from the wrong side it first crosses the line) and keeps
 *	        following even while LF_SEARCH_x is still set.  ORDER for the
 *	        Vehicle Controller: keep LF_ENABLE set, THEN clear LF_SEARCH_x
 *	        (clearing both at once = DISABLED for a step).  To search
 *	        again, clear LF_SEARCH_x first, then set it.
 *
 *	IF-S3c  S3 -> Vehicle Controller ONLY    event flag  flg_s3
 *	        FLG_S3_JUNCTION / LOST / BARCODE / LINE_FOUND, set by S3.
 *	        ONE consumer: tk_wai_flg(..., TWF_ORW | TWF_BITCLR, ...).
 *	        Everyone else reads the counters in S3_STATUS.
 *
 *	IF-S3b  S3 -> anyone (telemetry)         S3_STATUS (16 bytes) via getter
 *	        S3_STATUS st;  if (E_OK == s3_get_line_status(&st)) { ... }
 *	        Concurrency: written by the line task AND the barcode task --
 *	        both write under mtx_s3, the getter copies under mtx_s3.  Never
 *	        read the packed struct directly.  Tasks only.
 *
 *	IF-S3d  Motion (Buddy 2) -> S3     s3_set_odometry(fn): fn() = total mm.
 *	IF-S3e  any task -> S3             s3_set_line_gains(kp, kd); s3_request_recalibrate();
 *	IF-S3f  Buddy 1 (MQTT) -> S3       s3_set_nav_map("DCBA") on the demo day:
 *	        letters for LEFT, RIGHT, STRAIGHT, U-TURN.  No recompilation.
 */
#ifndef S3_API_H
#define S3_API_H

#include "../app.h"

/* ---------------------------------------------------------------- IF-01 */
typedef enum {				/* 5 values -> 3 bits */
	S3_CMD_NONE     = 0,		/* letter decoded but not in today's map */
	S3_CMD_LEFT     = 1,
	S3_CMD_RIGHT    = 2,
	S3_CMD_STRAIGHT = 3,
	S3_CMD_UTURN    = 4
} S3_CMD;

typedef struct {			/* 32 bits (was 12 bytes) */
	UW	cmd      : 3;		/* S3_CMD                                     */
	UW	ch_idx   : 6;		/* Code 39 character index 0..43 (s3_nav_msg_char) */
	UW	reversed : 1;		/* 1 = barcode crossed backwards              */
	UW	source   : 2;		/* 1 = analog+digital agree, 2 = analog, 3 = digital */
	UW	seq      : 4;		/* 1..15, 0.. : spots a dropped message      */
	UW	odom_mm  : 16;		/* odometry at decode, mm, wraps every 65.5 m:
					   distance to the junction = odom_now - odom_mm */
} S3_NAV_MSG;

IMPORT ID	mbf_nav;
IMPORT char	s3_nav_msg_char(const S3_NAV_MSG *p_msg);

/* ---------------------------------------------------------------- IF-02 */
IMPORT BOOL	s3_get_steer_sp(H *p_dv_mms, UB *p_seq);	/* FALSE = not steering (uncal/disabled/search) */

/* ---------------------------------------------------------------- IF-07 */
#define LF_ENABLE		0x0001U
#define LF_SEARCH_L		0x0002U
#define LF_SEARCH_R		0x0004U
IMPORT ID	flg_line;

/* ---------------------------------------------------------------- IF-S3c */
#define FLG_S3_JUNCTION		0x0001U
#define FLG_S3_LOST		0x0002U
#define FLG_S3_BARCODE		0x0004U
#define FLG_S3_LINE_FOUND	0x0008U
IMPORT ID	flg_s3;

/* ---------------------------------------------------------------- IF-S3b */
typedef enum {				/* 6 values -> 3 bits */
	S3_LINE_FOLLOW   = 0,		/* steering valid                             */
	S3_LINE_JUNCTION = 1,		/* junction under the car: steer held at 0    */
	S3_LINE_LOST     = 2,		/* no line for 40 mm: steer = last correction */
	S3_LINE_DISABLED = 3,		/* LF_ENABLE clear                            */
	S3_LINE_SEARCH   = 4,		/* LF_SEARCH_x: waiting for the correct edge  */
	S3_LINE_UNCAL    = 5		/* sensors not yet calibrated: do not drive   */
} S3_LINE_STATE;

typedef struct {			/* 4 x 32 bits = 16 bytes (was 36) */
	/* word 0 */
	INT	steer_dv  : 10;		/* mm/s, -511..+511 (same value as steer_sp)  */
	INT	pos       : 10;		/* line position estimate, -500..+500 permille */
	UW	state     : 3;		/* S3_LINE_STATE                              */
	UW	last_cmd  : 3;		/* S3_CMD of the last published barcode       */
	UW	last_rev  : 1;		/* it was read backwards                      */
	UW	overshoot : 1;		/* outer sensor black + Centre white          */
	UW	pad0      : 4;
	/* word 1 */
	UW	n_outer   : 8;		/* outer sensor, 0 = white .. 255 = black     */
	UW	n_centre  : 8;		/* Centre sensor, 0 = white .. 255 = black    */
	UW	last_ch   : 6;		/* Code 39 index of the last letter, 63 = none */
	UW	seq       : 8;		/* +1 every line step (10 ms): staleness       */
	UW	pad1      : 2;
	/* word 2 */
	UW	map_left  : 6;		/* today's letters (Code 39 indices)          */
	UW	map_right : 6;
	UW	map_str   : 6;
	UW	map_uturn : 6;
	UW	junctions : 8;		/* counters wrap at 256: consumers use differences */
	/* word 3 */
	UW	losts     : 8;
	UW	barcodes  : 8;
	UW	updated_ms: 16;		/* system time, ms, wraps every 65.5 s        */
} S3_STATUS;

IMPORT ID	mtx_s3;
IMPORT ER	s3_get_line_status(S3_STATUS *p_st);	/* tasks only; E_OK or mutex error */

/* ---------------------------------------------------------------- IF-S3d/e/f */
typedef UW (*S3_ODOM_FN)(void);
IMPORT void	s3_set_odometry(S3_ODOM_FN fn);
IMPORT void	s3_set_line_gains(INT kp_x100, INT kd_x100);	/* e.g. 150, 400 */
IMPORT void	s3_request_recalibrate(void);
IMPORT ER	s3_set_nav_map(const char *p_map);		/* tasks only */

/* ---------------------------------------------------------------- S3 internal */
IMPORT ER	s3_init(void);		/* called from subsystem3_init  */
IMPORT ER	s3_bringup_start(void);	/* called from subsystem3_start */

#endif /* S3_API_H */
