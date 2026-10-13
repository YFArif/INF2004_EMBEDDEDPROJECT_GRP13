/*
 *	drive_demo.c
 *	Subsystem 2 -- hands-off drive demo, built only with: make DRIVE=1
 *
 *	After a short delay both motors run forward for 30 s, then brake.  Each
 *	wheel is held at the same encoder rate so the robot goes roughly
 *	straight.  Press RESET to run it again.  The example tasks keep running.
 *
 *	Tuning: if it curves, change DEMO_TRIM_R_PERMILLE (lower slows the right
 *	wheel, so it curves right; higher curves left).  DEMO_KI 0 gives a fixed
 *	duty with no feedback.
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "drv_motor.h"

#if APP_DRIVE

#define DEMO_PRI 9
#define DEMO_START_DELAY_S 8 /* time to put the robot down */
#define DEMO_RUN_S 30
#define DEMO_STEP_MS 100
#define DEMO_TARGET_TICKS 100     /* rising edges per step: 1000 ticks/s, about 0.3 m/s */
#define DEMO_TRIM_R_PERMILLE 1000 /* right target = left target * this / 1000 */
#define DEMO_DUTY_START 400
#define DEMO_DUTY_MIN 150
#define DEMO_DUTY_MAX 700
#define DEMO_KI 1           /* permille of duty per tick of error */
#define DEMO_STALL_STEPS 10 /* steps without a tick before it gives up */

LOCAL INT clamp_duty(INT duty)
{
    if (duty < DEMO_DUTY_MIN)
    {
        return DEMO_DUTY_MIN;
    }
    if (duty > DEMO_DUTY_MAX)
    {
        return DEMO_DUTY_MAX;
    }
    return duty;
}

LOCAL void drive_demo_task(INT stacd, void *exinf)
{
    S2_ENC l, r;
    UW prev_l = 0, prev_r = 0;
    INT duty_l = DEMO_DUTY_START, duty_r = DEMO_DUTY_START;
    INT target_r = DEMO_TARGET_TICKS * DEMO_TRIM_R_PERMILLE / 1000;
    INT step, dl, dr, stalled = 0;

    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"drive demo: starting in %d s, runs for %d s\n",
              DEMO_START_DELAY_S, DEMO_RUN_S);
    tk_dly_tsk(DEMO_START_DELAY_S * 1000);

    s2_enc_clear();
    s2_motor_set(S2_MOTOR_LEFT, duty_l);
    s2_motor_set(S2_MOTOR_RIGHT, duty_r);

    for (step = 1; step <= DEMO_RUN_S * 1000 / DEMO_STEP_MS; step++)
    {
        tk_dly_tsk(DEMO_STEP_MS);

        s2_enc_get(S2_ENC_LEFT, &l);
        s2_enc_get(S2_ENC_RIGHT, &r);
        dl = (INT)(l.rise - prev_l);
        dr = (INT)(r.rise - prev_r);
        prev_l = l.rise;
        prev_r = r.rise;

        stalled = (dl == 0 || dr == 0) ? stalled + 1 : 0;
        if (stalled >= DEMO_STALL_STEPS)
        {
            tm_printf((UB *)"drive demo: no encoder ticks, stopping\n");
            break;
        }

        duty_l = clamp_duty(duty_l + DEMO_KI * (DEMO_TARGET_TICKS - dl));
        duty_r = clamp_duty(duty_r + DEMO_KI * (target_r - dr));
        s2_motor_set(S2_MOTOR_LEFT, duty_l);
        s2_motor_set(S2_MOTOR_RIGHT, duty_r);

        if (step % 10 == 0)
        {
            tm_printf((UB *)"drive demo: %d s  ticks/100ms L %d R %d  duty L %d R %d\n",
                      step / 10, dl, dr, duty_l, duty_r);
        }
    }

    s2_motor_brake();
    tm_printf((UB *)"drive demo: finished, motors braked\n");
    tk_slp_tsk(TMO_FEVR);
}

EXPORT ER s2_drive_demo_start(void)
{
    /* Core 1, where motion_task will run */
    ID tid = app_start_task(drive_demo_task, DEMO_PRI, TP_PRC2, "drive_demo");

    return (tid < E_OK) ? tid : E_OK;
}

#endif /* APP_DRIVE */
