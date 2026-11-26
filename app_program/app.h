/** @file app.h
 *
 * @brief Definitions shared by the whole team.  Changed only by team
 *        agreement.
 *
 * @par
 * Anything that belongs to ONE subsystem goes in that subsystem's own
 * header instead (app_program/subsystemN/subsystemN.h).
 */

#ifndef APP_H
#define APP_H

#include <tk/tkernel.h>     /* micro T-Kernel API (tk_xxx_yyy) */
#include <tm/tmonitor.h>    /* T-Monitor API: tm_printf() console output */
#include <bsp/libbsp.h>     /* Board helpers: gpio_set_pin/val, pwm_... */

/* ---------------------------------------------------------------------
 *  Pins: free for projects are GP0..GP22 and GP26..GP28.  GP0/GP1 are
 *  taken by UART0 when built with "make CRASH_REPORT=1" (see README).
 *  Never use GP23/24/25/29: they are wired to the Pico W radio.
 *  Claim every pin in TEAM.md section 2 before using it.
 * --------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 *  Task priorities: 1 is the highest; the kernel's USB console (3) and
 *  WiFi (4) tasks sit above every application task.  Each subsystem
 *  has its own band (TEAM.md section 4), defined in its own header.
 * --------------------------------------------------------------------- */
#define TASK_STACK_SIZE     2048    /* bytes; tm_printf needs ~1 KB */

/* Create and start a task.  prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or
 * 0 for "any core".  Defined in app_main.c.
 */
IMPORT ID app_start_task(FP p_entry, PRI pri, UW prc, const char * p_name);

/* The program's entry point, called by the kernel's initial task.
 * Defined in app_main.c.
 */
IMPORT INT usermain(void);

/* ---------------------------------------------------------------------
 *  Every subsystem provides these.  usermain() calls ALL the _init()
 *  functions first and only then ALL the _start() functions, so every
 *  kernel object exists before any task can use it.
 *    subsystemN_init()   create this subsystem's kernel objects
 *    subsystemN_start()  create and start this subsystem's tasks
 *    subsystemN_test()   on-target tests, only in test builds (make TEST=1)
 * --------------------------------------------------------------------- */
IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);

IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* Print a readable message if a kernel call failed (negative error code).
 *
 * NOTE: A macro, not a function (rule 6.3.a deviation): it is used by
 * every subsystem, and turning it into a function would change the
 * compiled code.  er is evaluated once and what is used once (6.3.b).
 * The casts are safe: er is a kernel error code, which fits INT, and the
 * format string is ASCII.
 */
#define CHECK(er, what)                                                 \
    do                                                                  \
    {                                                                   \
        INT app_check_err = (INT)(er);                                  \
                                                                        \
        if (app_check_err < E_OK)                                       \
        {                                                               \
            tm_printf((UB *)"ERROR %s: %d\n", (what), app_check_err);   \
        }                                                               \
    } while (0)

#endif /* APP_H */

/*** end of file ***/

