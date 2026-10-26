/*
 *	subsystem1.c
 *	Subsystem 1 -- LED heartbeat: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"
#include "mqtt_link.h"	/* empty unless TM_MQTT (MQTT=1 builds) */

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};

#if TM_MQTT
	/* MQTT driver: creates its two message buffers; no lwIP calls. A failure
	   leaves the car running without MQTT; mqtt_link_get_status() shows why. */
	if (MQTT_LINK_OK != mqtt_link_init()) {
		tm_printf((UB*)"S1: mqtt_link_init failed\n");
	}
#endif

	/* Cyclic handler: once started, the kernel calls led_cyclic_handler
	   every S1_TICK_MS.  Without TA_STA it is created stopped; it is
	   started in subsystem1_start(), once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;	/* period, ms */
	ccyc.cycphs = S1_TICK_MS;	/* delay before the first call, ms */
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	return tk_sta_cyc(cyc_led);	/* start the cyclic handler now */
}
