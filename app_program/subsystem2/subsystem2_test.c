/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "../test_report.h"

#if APP_TEST

#define S2_U01_BENCH 0	   /* manual test, blocks ~90 s at power-up: set to 0 before merging to main */
#define S2_U01_REVS 10	   /* wheel revolutions per measurement */
#define S2_U01_WINDOW_S 40 /* seconds allowed to turn them */

#if S2_U01_BENCH
/* Turn one wheel by hand.  TRUE if it counted edges and the other wheel stayed under 1% of that. */
LOCAL BOOL s2_u01_wheel(INT mine, const char *name)
{
	S2_ENC l, r, m, o;
	INT s;

	tm_printf((UB *)"\n[S2-U01] %s wheel: keep the other wheel still\n", name);
	for (s = 3; s > 0; s--)
	{
		tm_printf((UB *)"[S2-U01] starting in %d\n", s);
		tk_dly_tsk(1000);
	}
	s2_enc_clear();
	tm_printf((UB *)"[S2-U01] GO: turn the %s wheel %d full revolutions, then stop\n",
			  name, S2_U01_REVS);

	for (s = 1; s <= S2_U01_WINDOW_S; s++)
	{
		tk_dly_tsk(1000);
		s2_enc_get(S2_ENC_LEFT, &l);
		s2_enc_get(S2_ENC_RIGHT, &r);
		tm_printf((UB *)"[S2-U01] %d s   L rise=%u fall=%u   R rise=%u fall=%u\n",
				  s, l.rise, l.fall, r.rise, r.fall);
	}

	s2_enc_get(mine, &m);
	s2_enc_get(1 - mine, &o);
	tm_printf((UB *)"[TEST] S2-U01 INFO %s: rise=%u fall=%u  ticks/rev: rising=%u.%u both=%u.%u"
					"  shortest gap=%u us  glitches(<100us)=%u  last period=%u us\n",
			  name, m.rise, m.fall,
			  m.rise / S2_U01_REVS, (m.rise * 10 / S2_U01_REVS) % 10,
			  (m.rise + m.fall) / S2_U01_REVS,
			  ((m.rise + m.fall) * 10 / S2_U01_REVS) % 10,
			  m.min_dt_us, m.glitches, m.period_us);

	return (m.rise + m.fall) > 0 && (o.rise + o.fall) * 100 <= (m.rise + m.fall);
}
#endif /* S2_U01_BENCH */

#define S2_U06_BENCH 1	/* manual: spins each motor on its own, lift the wheels first; ~35 s */
#define S2_U06_DUTY 400 /* permille; the last run uses double */

#if S2_U06_BENCH
/* Drive one motor, let it settle, then count its encoder for 1 s.  TRUE if only its own wheel counted. */
LOCAL BOOL s2_u06_run(INT motor, INT duty, UW *ticks)
{
	S2_ENC m, o;
	UW cpr = (motor == S2_MOTOR_LEFT) ? S2_TICKS_PER_REV_L : S2_TICKS_PER_REV_R;

	s2_motor_set(motor, duty);
	tk_dly_tsk(2000);
	s2_enc_clear();
	tk_dly_tsk(1000);
	s2_enc_get(motor, &m);
	s2_enc_get(1 - motor, &o);
	s2_motor_set(motor, 0);
	tk_dly_tsk(1000);

	*ticks = m.rise;
	tm_printf((UB *)"[S2-U06]   duty %d: %u ticks/s = %u mm/s, other wheel %u edges\n",
			  duty, m.rise, m.rise * 1979 / (cpr * 10), o.rise + o.fall);
	return m.rise > 0 && (o.rise + o.fall) * 100 <= (m.rise + m.fall);
}

/* Forward, reverse, then forward at double duty.  Direction is judged by eye. */
LOCAL BOOL s2_u06_motor(INT motor, const char *name)
{
	UW slow, back, fast;
	BOOL ok;
	INT s;

	tm_printf((UB *)"\n[S2-U06] %s motor: its wheel must be off the ground\n", name);
	for (s = 3; s > 0; s--)
	{
		tm_printf((UB *)"[S2-U06] starting in %d\n", s);
		tk_dly_tsk(1000);
	}

	tm_printf((UB *)"[S2-U06] %s FORWARD: the robot should be driven forward\n", name);
	ok = s2_u06_run(motor, S2_U06_DUTY, &slow);
	tm_printf((UB *)"[S2-U06] %s REVERSE: the opposite direction\n", name);
	ok = s2_u06_run(motor, -S2_U06_DUTY, &back) && ok;
	tm_printf((UB *)"[S2-U06] %s FORWARD at double duty: faster\n", name);
	ok = s2_u06_run(motor, 2 * S2_U06_DUTY, &fast) && ok;

	s2_motor_brake();
	return ok && fast > slow;
}
#endif /* S2_U06_BENCH */

EXPORT void subsystem2_test(void)
{
	TEST_INIT();

#if S2_U01_BENCH
	/* S2-U01: encoder ticks per wheel revolution (manual, one wheel at a time) */
	TEST_CHECK("S2-U01", s2_u01_wheel(S2_ENC_LEFT, "LEFT"),
			   "left wheel counted on GP16, GP17 silent");
	TEST_CHECK("S2-U01", s2_u01_wheel(S2_ENC_RIGHT, "RIGHT"),
			   "right wheel counted on GP17, GP16 silent");
#endif

#if S2_U06_BENCH
	/* S2-U06: each motor turns its own wheel (manual: also watch the direction) */
	TEST_CHECK("S2-U06", s2_u06_motor(S2_MOTOR_LEFT, "LEFT"),
			   "left motor turned the left encoder");
	TEST_CHECK("S2-U06", s2_u06_motor(S2_MOTOR_RIGHT, "RIGHT"),
			   "right motor turned the right encoder");
#endif

	TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
