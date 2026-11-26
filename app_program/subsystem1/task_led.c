/** @file task_led.c
 *
 * @brief Subsystem 1 -- the LED task and its cyclic handler.
 *
 * @par
 * APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *             gpio_set_pin / gpio_set_val
 *
 * @par
 * Instead of tk_dly_tsk(), this task sleeps until a cyclic handler
 * wakes it.  The handler is called by the kernel's timer at a fixed
 * rate, so the blink rate stays exact however long the task's own work
 * takes.
 */

#include "subsystem1.h"

/*!
 * @brief Cyclic handler: wake the LED task once per tick.
 *
 * Runs in interrupt context, NOT as a task, so it must be short and must
 * never wait (no tk_slp_tsk, tk_wai_sem, tm_printf...).  Signalling a
 * task is the normal thing to do here.
 *
 * @param[in] p_exinf Extended information; not used.
 */
EXPORT void
s1_led_cyclic_handler (void * p_exinf)
{
    /* The cast marks the argument as unused. */
    (void)p_exinf;
    tk_wup_tsk(gh_tid_led);     /* wake the LED task (the request queues
                                   up if the task is not asleep yet) */
}

/*!
 * @brief The LED task: toggle the LED on every wake-up from the handler.
 *
 * @param[in] stacd   Start code; not used.
 * @param[in] p_exinf Extended information; not used.
 */
EXPORT void
s1_led_task (INT stacd, void * p_exinf)
{
    UINT b_led_on = 0;

    /* The casts mark the arguments as unused. */
    (void)stacd;
    (void)p_exinf;

    gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);    /* make the pin an output */

    for (;;)
    {
        /* Sleep until s1_led_cyclic_handler calls tk_wup_tsk(). */
        tk_slp_tsk(TMO_FEVR);

        /* tk_can_wup: discard wake-ups that queued up while we were
         * not sleeping (e.g. while subsystem 4 had us suspended), so
         * the LED does not "catch up" with a burst of fast blinks.
         */
        tk_can_wup(TSK_SELF);

        b_led_on = !b_led_on;
        gpio_set_val(S1_LED_PIN, b_led_on);
    }
}

/*** end of file ***/

