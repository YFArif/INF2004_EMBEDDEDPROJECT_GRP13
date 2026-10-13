/*
 *	drv_motor.c
 *	Subsystem 2 -- DC motor driver: 20 kHz hardware PWM on slices 4 and 5.
 *
 *	APIs shown: pwm_set_pin, pwm_set_wrap, pwm_set_cc, pwm_set_enabled
 */

#include "drv_motor.h"

#define PWM_WRAP 6249    /* 125 MHz system clock / 20 kHz - 1 */
#define PWM_DIV_1_0 0x10 /* clock divider 1.0 */

LOCAL const UINT pin_a[2] = {8, 10};
LOCAL const UINT pin_b[2] = {9, 11};
LOCAL const INT invert[2] = {S2_MOTOR_INVERT_L, S2_MOTOR_INVERT_R};

LOCAL UW duty_to_cc(INT duty)
{
    return ((UW)duty * (PWM_WRAP + 1)) / S2_DUTY_MAX;
}

EXPORT ER s2_motor_init(void)
{
    INT i;
    UW base;

    /* The kernel leaves PWM in reset unless USE_PTMR is set (config.h), so release it here. */
    set_w(RESETS_RESET, RESETS_RESET_PWM);
    clr_w(RESETS_RESET, RESETS_RESET_PWM);
    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0)
    {
    }

    for (i = 0; i < 2; i++)
    {
        base = PWM_BASE + 0x14 * (pin_a[i] >> 1); /* registers of this slice */

        pwm_set_enabled(pin_a[i], FALSE);
        out_w(base + PWM_CHx_DIV, PWM_DIV_1_0);
        pwm_set_wrap(pin_a[i], PWM_WRAP);
        pwm_set_cc(pin_a[i], 0); /* both inputs low = brake from the start */
        pwm_set_cc(pin_b[i], 0);
        out_w(base + PWM_CHx_CTR, 0);
        pwm_set_pin(pin_a[i]);
        pwm_set_pin(pin_b[i]);
        pwm_set_enabled(pin_a[i], TRUE);
    }
    return E_OK;
}

EXPORT void s2_motor_set(INT motor, INT duty)
{
    if (motor < 0 || motor > 1)
    {
        return;
    }
    if (invert[motor])
    {
        duty = -duty;
    }
    if (duty > S2_DUTY_MAX)
    {
        duty = S2_DUTY_MAX;
    }
    if (duty < -S2_DUTY_MAX)
    {
        duty = -S2_DUTY_MAX;
    }

    /* The idle input goes low first, so both are never high at once (that would coast). */
    if (duty >= 0)
    {
        pwm_set_cc(pin_b[motor], 0);
        pwm_set_cc(pin_a[motor], duty_to_cc(duty));
    }
    else
    {
        pwm_set_cc(pin_a[motor], 0);
        pwm_set_cc(pin_b[motor], duty_to_cc(-duty));
    }
}

EXPORT void s2_motor_brake(void)
{
    s2_motor_set(S2_MOTOR_LEFT, 0);
    s2_motor_set(S2_MOTOR_RIGHT, 0);
}

EXPORT void s2_motor_coast(void)
{
    INT i;

    for (i = 0; i < 2; i++)
    {
        pwm_set_cc(pin_a[i], PWM_WRAP + 1); /* above the wrap value = always high */
        pwm_set_cc(pin_b[i], PWM_WRAP + 1);
    }
}
