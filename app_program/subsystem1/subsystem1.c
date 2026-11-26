/** @file subsystem1.c
 *
 * @brief Subsystem 1 -- LED heartbeat: creates its objects and starts its
 *        task.
 *
 * @par
 * APIs shown: tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"
#include "mqtt_link.h"       /* empty unless TM_MQTT (MQTT=1 builds) */
#include "telemetry_task.h"  /* empty unless TM_MQTT */
#include "s1_event.h"        /* s1_event_init() only if TM_MQTT */

EXPORT ID   gh_tid_led;     /* IF-05, see subsystem1.h */
EXPORT ID   gh_cyc_led;

#if TM_MQTT
LOCAL bool gb_mqtt_ok = false;  /* mqtt_link_init() succeeded */
#endif

/*!
 * @brief Create subsystem 1's kernel objects (called by usermain()).
 *
 * In MQTT=1 builds it also creates the MQTT link's and mbf_event's
 * message buffers; a failure there leaves the car running without them.
 *
 * @return E_OK, or the negative kernel error from tk_cre_cyc().
 */
EXPORT ER
subsystem1_init (void)
{
    T_CCYC ccyc = {0};

#if TM_MQTT
    /* MQTT driver: creates its two message buffers; no lwIP calls. A failure
       leaves the car running without MQTT; mqtt_link_get_status() shows why. */
    if (MQTT_LINK_OK != mqtt_link_init())
    {
        /* The cast is safe: the format string is ASCII. */
        tm_printf((UB *)"S1: mqtt_link_init failed\n");
    }
    else
    {
        gb_mqtt_ok = true;
    }

    /* mbf_event (FR16). Without it posts are dropped and counted; the car
       runs on. */
    if (0 != s1_event_init())
    {
        /* The cast is safe: the format string is ASCII. */
        tm_printf((UB *)"S1: mbf_event not created\n");
    }
#endif

    /* Cyclic handler: once started, the kernel calls s1_led_cyclic_handler
     * every S1_TICK_MS.  Without TA_STA it is created stopped; it is
     * started in subsystem1_start(), once the task it wakes exists.  The
     * cast is safe: FP is the kernel's generic handler address type, and
     * the kernel calls the handler with the one argument it declares.
     */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)s1_led_cyclic_handler;
    ccyc.cyctim = S1_TICK_MS;   /* period, ms */
    ccyc.cycphs = S1_TICK_MS;   /* delay before the first call, ms */
    gh_cyc_led  = tk_cre_cyc(&ccyc);
    return ((gh_cyc_led < E_OK) ? gh_cyc_led : E_OK);
}

/*!
 * @brief Start subsystem 1's task and cyclic handler (called by
 *        usermain() after every subsystem's init).
 *
 * In MQTT=1 builds it also starts telemetry_task if the MQTT link was
 * created; if that fails the car runs on without it.
 *
 * @return E_OK, or the negative kernel error from app_start_task() or
 *         tk_sta_cyc().
 */
EXPORT ER
subsystem1_start (void)
{
    gh_tid_led = app_start_task(s1_led_task, S1_PRI_LED, 0, "led_task");
    if (gh_tid_led < E_OK)
    {
        return gh_tid_led;
    }

#if TM_MQTT
    /* telemetry_task (Stage 4a): dashboard commands and the heartbeat.
       Only with a working MQTT link; without one, or if it cannot start,
       the car runs on without it. */
    if (gb_mqtt_ok && (telemetry_task_start() < 0))
    {
        /* The cast is safe: the format string is ASCII. */
        tm_printf((UB *)"S1: telemetry_task not started\n");
    }
#endif
    return tk_sta_cyc(gh_cyc_led);  /* start the cyclic handler now */
}

/*** end of file ***/

