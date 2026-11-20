/*
 *	task_led.c
 *	Subsystem 1 -- the LED task and its cyclic handler.
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *	            gpio_set_pin / gpio_set_val
 *
 *	Instead of tk_dly_tsk(), this task sleeps until a cyclic handler
 *	wakes it.  The handler is called by the kernel's timer at a fixed
 *	rate, so the blink rate stays exact however long the task's own work
 *	takes.
 */

#include "subsystem1.h"

/*
 * Cyclic handler.  Runs in interrupt context, NOT as a task, so it must
 * be short and must never wait (no tk_slp_tsk, tk_wai_sem, tm_printf...).
 * Signalling a task is the normal thing to do here.
 */
EXPORT void led_cyclic_handler(void *p_exinf)
{
	(void)p_exinf;
	tk_wup_tsk(tid_led);	/* wake the LED task (the request queues
				   up if the task is not asleep yet) */
}

EXPORT void led_task(INT stacd, void *p_exinf)
{
	UINT	level = 0;

	(void)stacd; (void)p_exinf;

	gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);	/* make the pin an output */

	while (1) {
		/* Sleep until led_cyclic_handler calls tk_wup_tsk() */
		tk_slp_tsk(TMO_FEVR);

		/* tk_can_wup: discard wake-ups that queued up while we were
		   not sleeping (e.g. while subsystem 4 had us suspended), so
		   the LED does not "catch up" with a burst of fast blinks. */
		tk_can_wup(TSK_SELF);

		level = !level;
		gpio_set_val(S1_LED_PIN, level);
	}
}
