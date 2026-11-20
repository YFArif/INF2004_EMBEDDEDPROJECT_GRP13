/*
 *	subsystem1.h
 *	Subsystem 1 -- LED heartbeat.  Student ID: <student ID>.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define S1_PRI_LED	12		/* priority band 12-13 (TEAM.md) */
#define S1_LED_PIN	19		/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS	250		/* LED toggles every tick */

/* IF-05: the LED task's ID, so subsystem 4 may pause/resume it */
IMPORT ID	tid_led;

/* IF-10 events (FR16): report one event to the dashboard. Never waits;
   safe from any task on either core, not from an interrupt handler.
   code is from the agreed table (S5: 0x50 to 0x65; 0 is never used);
   arg_a and arg_b are the two argument bytes (s5_event.h lists them per
   code). Returns true if queued, false if dropped (mbf_event full, or a
   build without MQTT, where it does nothing). Defined in s1_event.c. */
IMPORT bool	subsystem1_event_post(uint8_t code, uint8_t arg_a,
				      uint8_t arg_b);

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *p_exinf);
IMPORT void	led_cyclic_handler(void *p_exinf);

#endif /* SUBSYSTEM1_H */
