/*
 *	subsystem2.h
 *	Subsystem 2 -- Producer (sample source).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"

#define S2_PRI_PRODUCER 14 /* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS 300   /* one sample every 300 ms */

/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct
{
	T_MSG hdr;	/* used by the kernel, do not touch */
	UW seq;		/* sample sequence number */
	UW time_ms; /* system time when the sample was taken */
	INT value;	/* the "measurement", 0..100 */
	INT prc;	/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT 4 /* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID sem_slots;
IMPORT ID mpf_samples;

/* ---------------------------------------------------------------------
 *  Motion control interfaces (IF-02, IF-04, IF-05, IF-06 in the report).
 *  Units: distance mm, angle degrees, speed mm/s.  Left is positive.
 * --------------------------------------------------------------------- */

/* IF-04: commands to motion_task, sent only by vehicle_ctrl_task (S5).
   A mailbox passes a pointer: take each message from your own static ring
   and never reuse it before motion_task has received it. */
typedef enum
{
	MC_STOP,
	MC_FOLLOW,
	MC_MOVE,
	MC_TURN,
	MC_CLEAR
} MC_TYPE;

typedef struct
{
	T_MSG hdr; /* used by the kernel, must be first */
	UB type;   /* MC_xxx */
	H value;   /* MC_MOVE: mm, negative = reverse; MC_TURN: degrees, positive = left */
	H speed;   /* mm/s, never negative; 0 = default speed */
} MOTION_CMD;

IMPORT ID mbx_motion;

/* IF-05: bits of flg_motion */
#define MF_ESTOP 0x00000001 /* S4/S5 set it to brake; only S5 clears it */
IMPORT ID flg_motion;

/* IF-06: odometry, written by motion_task every 20 ms */
typedef struct
{
	UW count_l; /* encoder rising edges since boot */
	UW count_r;
	W dist_mm;	   /* signed distance of the robot centre since boot */
	W speed_mms;   /* robot speed, positive = forward */
	W speed_l_mms; /* signed wheel speeds, used for the turn rate */
	W speed_r_mms;
} ODOM;

IMPORT void s2_odom_put(const ODOM *in); /* motion_task only */
IMPORT void s2_odom_get(ODOM *out);		 /* any task */

/* IF-02: steering correction from line_task, latest value wins.
   Positive turns the robot left (the right wheel runs faster). */
IMPORT void s2_steer_set(H dv_mms); /* line_task */
IMPORT H s2_steer_get(void);		/* motion_task */

/* Chassis constants.  Ticks per revolution are provisional (S2-U01 runs 1-2). */
#define S2_WHEEL_DIA_MM 63
#define S2_TRACK_MM 112		   /* centre of left tyre to centre of right tyre */
#define S2_TICKS_PER_REV_L 639 /* encoder rising edges per wheel revolution */
#define S2_TICKS_PER_REV_R 629

/* Private to subsystem 2 */
IMPORT void producer_task(INT stacd, void *exinf);
IMPORT INT s2_make_value(UW seq);
IMPORT ER s2_drive_demo_start(void); /* make DRIVE=1 only */

#endif /* SUBSYSTEM2_H */
