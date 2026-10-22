/** @file subsystem2_step.c
 *
 * @brief Test builds only: hold each bench or floor test step until the
 *        tester presses Button 1 on the Robo Pico (GP20), then count down
 *        STEP_COUNTDOWN_S seconds before the robot moves.
 *
 * @par Why
 * The PC cannot send keys to the Pico (the USB console drops input), so
 * without this the robot would move on a timer, even while the tester is
 * still measuring or typing. With it, nothing moves until the tester is
 * ready and watching the robot.
 *
 * @par GP20
 * Read-only, through SIO, with the pad pull-up on; a press pulls it low.
 * The Week 6 design gives GP20 to S5 as the start button, which is not
 * written yet; TEAM.md records this test-only use. Nothing here is
 * compiled into a normal (make without TEST=1) image.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>     /* in_w(), out_w(); must precede libbsp.h.  */
#include <bsp/libbsp.h>     /* gpio_set_pin().                           */

#include "subsystem2.h"

#if APP_TEST

#define BUTTON_PIN          (20u)       /* Robo Pico Button 1.           */
#define STEP_COUNTDOWN_S    (3u)
#define POLL_MS             (10u)
#define PRESS_MS            (50u)       /* Held this long = a press.     */
#define REMIND_MS           (60000u)    /* Repeat the prompt each minute.*/

/* PADS_BANK0 (datasheet 2.19.6.3) and SIO (2.3.1.7). */
#define ALIAS_SET           (0x2000u)
#define ALIAS_CLR           (0x3000u)
#define PADS_BANK0_ADDR     (0x4001C000u)
#define PAD_ADDR(pin)       (PADS_BANK0_ADDR + 4u + (4u * (pin)))
#define PAD_PUE_BIT         (0x08u)
#define PAD_PDE_BIT         (0x04u)
#define SIO_GPIO_IN_REG     (0xD0000004u)

static bool g_b_ready = false;          /* GP20 set up.                  */

static bool button_down(void);
static void sleep_ms(uint32_t ms);

/*!
 * @brief Announce the next step, wait for Button 1, count down, return.
 *
 * @param[in] p_next What the next step does and what to watch.
 *
 * NOTE: Only for the S2 test files, from usermain's test run. Prints
 * "[S2-STEP] ..." lines, which s2_test.ps1 uses to beep and to time its
 * questions.
 */
void
subsystem2_step_ready (char const * p_next)
{
    uint32_t held   = 0u;
    uint32_t waited = 0u;
    uint32_t sec    = 0u;

    if (!g_b_ready)
    {
        (void) gpio_set_pin(BUTTON_PIN, GPIO_MODE_IN);
        out_w(PAD_ADDR(BUTTON_PIN) + ALIAS_CLR, PAD_PDE_BIT);
        out_w(PAD_ADDR(BUTTON_PIN) + ALIAS_SET, PAD_PUE_BIT);
        g_b_ready = true;
        sleep_ms(POLL_MS);
    }

    // NOTE: The casts are safe: the strings are ASCII and the numbers are
    // below 10.
    tm_printf((UB *) "\n[S2-STEP] NEXT: %s\n", p_next);
    tm_printf((UB *) "[S2-STEP] press BUTTON 1 (GP20) on the robot when"
              " ready\n");

    // A button still held from the last step must be let go first.
    while (button_down())
    {
        sleep_ms(POLL_MS);
    }

    while (held < PRESS_MS)
    {
        sleep_ms(POLL_MS);
        waited += POLL_MS;

        if (button_down())
        {
            held += POLL_MS;
        }
        else
        {
            held = 0u;
        }

        if (waited >= REMIND_MS)
        {
            waited = 0u;
            tm_printf((UB *) "[S2-STEP] still waiting for BUTTON 1\n");
        }
    }

    while (button_down())
    {
        sleep_ms(POLL_MS);
    }

    for (sec = STEP_COUNTDOWN_S; sec > 0u; sec--)
    {
        tm_printf((UB *) "[S2-STEP] %d\n", (int) sec);
        sleep_ms(1000u);
    }

    tm_printf((UB *) "[S2-STEP] GO\n");
}

/*!
 * @brief Read Button 1.
 *
 * @return true while it is pressed (GP20 low).
 */
static bool
button_down (void)
{
    return (0u == (in_w(SIO_GPIO_IN_REG) & (1u << BUTTON_PIN)));
}

/*!
 * @brief Sleep, letting the other tasks run.
 *
 * @param[in] ms Milliseconds.
 */
static void
sleep_ms (uint32_t ms)
{
    // NOTE: The cast is safe: delays here are at most 1 s.
    CHECK(tk_dly_tsk((RELTIM) ms), "tk_dly_tsk");
}

#endif /* APP_TEST */

/*** end of file ***/
