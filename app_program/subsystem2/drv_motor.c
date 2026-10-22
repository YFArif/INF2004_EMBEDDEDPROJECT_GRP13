/** @file drv_motor.c
 *
 * @brief Subsystem 2 -- DC motor driver: 20 kHz hardware PWM on slices 4
 *        and 5.
 *
 * @par
 * Register addresses and bits are from the RP2040 datasheet: RESETS
 * (2.14) and PWM (4.5). Both inputs of one motor are set with a single
 * write to the slice's CC register, so the two inputs can never be high
 * together by accident (that would coast instead of drive or brake).
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>     /* in_w() and out_w() register access.      */
#include <bsp/libbsp.h>     /* pwm_set_pin() function select.            */

#include "drv_motor.h"

/* Atomic register alias for APB peripherals (datasheet 2.1.2). */
#define ALIAS_CLR           (0x3000u)

/* RESETS. */
#define RESETS_RESET_REG    (0x4000C000u)
#define RESETS_DONE_REG     (0x4000C008u)
#define RESETS_PWM_BIT      (0x00004000u)

/* PWM: eight slices of five registers. */
#define PWM_ADDR            (0x40050000u)
#define PWM_SLICE_STRIDE    (0x14u)
#define PWM_CSR_OFFSET      (0x00u)
#define PWM_DIV_OFFSET      (0x04u)
#define PWM_CTR_OFFSET      (0x08u)
#define PWM_CC_OFFSET       (0x0Cu)
#define PWM_TOP_OFFSET      (0x10u)
#define PWM_CSR_EN_BIT      (0x01u)
#define PWM_DIV_ONE         (0x10u)     /* Integer 1, fraction 0.        */
#define PWM_CC_B_SHIFT      (16u)

/* 125 MHz system clock / 20 kHz = 6250 counts per PWM period. */
#define PWM_TOP             (6249u)
#define PWM_PERIOD_COUNTS   (PWM_TOP + 1u)
#define PWM_ALWAYS_HIGH     (PWM_PERIOD_COUNTS)
#define DUTY_SCALE          (1000u)     /* DRV_MOTOR_DUTY_MAX, unsigned.  */

/** Pins and PWM slice of one motor channel. */
typedef struct
{
    uint32_t pin_a;         /* Input A (PWM channel A): forward.         */
    uint32_t pin_b;         /* Input B (PWM channel B): backward.        */
    uint32_t slice;
    bool     b_invert;      /* Wheel mounted mirrored: swap directions.  */
} motor_channel_t;

static motor_channel_t const g_channel[DRV_MOTOR_COUNT] =
{
    { 8u,  9u, 4u, (0 != DRV_MOTOR_INVERT_LEFT)  },
    { 10u, 11u, 5u, (0 != DRV_MOTOR_INVERT_RIGHT) }
};

static void     pwm_release(void);
static uint32_t pwm_reg(uint32_t slice, uint32_t offset);
static uint32_t duty_to_counts(uint32_t duty_permille);

/*!
 * @brief Set up both motor channels at 20 kHz with both motors braked.
 *
 * NOTE: Called once from subsystem2_init(), before any task runs.
 */
void
drv_motor_init (void)
{
    uint32_t idx = 0u;

    pwm_release();

    // NOTE: The cast is safe: DRV_MOTOR_COUNT is 2.
    for (idx = 0u; idx < (uint32_t) DRV_MOTOR_COUNT; idx++)
    {
        uint32_t slice = g_channel[idx].slice;

        out_w(pwm_reg(slice, PWM_CSR_OFFSET), 0u);  /* Stopped, normal.  */
        out_w(pwm_reg(slice, PWM_DIV_OFFSET), PWM_DIV_ONE);
        out_w(pwm_reg(slice, PWM_TOP_OFFSET), PWM_TOP);
        out_w(pwm_reg(slice, PWM_CC_OFFSET), 0u);   /* Both low: brake.  */
        out_w(pwm_reg(slice, PWM_CTR_OFFSET), 0u);
        (void) pwm_set_pin(g_channel[idx].pin_a);
        (void) pwm_set_pin(g_channel[idx].pin_b);
        out_w(pwm_reg(slice, PWM_CSR_OFFSET), PWM_CSR_EN_BIT);
    }
}

/*!
 * @brief Drive one motor.
 *
 * @param[in] motor         DRV_MOTOR_LEFT or DRV_MOTOR_RIGHT.
 * @param[in] duty_permille -1000 ... +1000; positive drives the robot
 *                          forward, 0 brakes. Values outside are clipped.
 */
void
drv_motor_set (drv_motor_t motor, int32_t duty_permille)
{
    int32_t  duty = duty_permille;
    uint32_t cc_a = 0u;
    uint32_t cc_b = 0u;

    if (motor < DRV_MOTOR_COUNT)
    {
        if (g_channel[motor].b_invert)
        {
            duty = -duty;
        }

        if (duty > DRV_MOTOR_DUTY_MAX)
        {
            duty = DRV_MOTOR_DUTY_MAX;
        }
        else if (duty < -DRV_MOTOR_DUTY_MAX)
        {
            duty = -DRV_MOTOR_DUTY_MAX;
        }
        else
        {
            // Already in range.
        }

        // NOTE: The casts are safe: duty is clipped to -1000 ... 1000
        // above, so each cast value is 0 ... 1000.
        if (duty > 0)
        {
            cc_a = duty_to_counts((uint32_t) duty);
        }
        else if (duty < 0)
        {
            cc_b = duty_to_counts((uint32_t) (-duty));
        }
        else
        {
            // Both inputs low: brake.
        }

        out_w(pwm_reg(g_channel[motor].slice, PWM_CC_OFFSET),
              (cc_b << PWM_CC_B_SHIFT) | cc_a);
    }
}

/*!
 * @brief Brake both motors (both inputs low).
 */
void
drv_motor_brake (void)
{
    drv_motor_set(DRV_MOTOR_LEFT, 0);
    drv_motor_set(DRV_MOTOR_RIGHT, 0);
}

/*!
 * @brief Let both motors spin freely (both inputs high).
 */
void
drv_motor_coast (void)
{
    uint32_t idx = 0u;

    // NOTE: The cast is safe: DRV_MOTOR_COUNT is 2.
    for (idx = 0u; idx < (uint32_t) DRV_MOTOR_COUNT; idx++)
    {
        // A compare value above TOP keeps the output high all the time.
        out_w(pwm_reg(g_channel[idx].slice, PWM_CC_OFFSET),
              (PWM_ALWAYS_HIGH << PWM_CC_B_SHIFT) | PWM_ALWAYS_HIGH);
    }
}

/*!
 * @brief Take the PWM block out of reset if it is still held there.
 *
 * NOTE: The kernel leaves PWM in reset because USE_PTMR is 0 in
 * config.h. This only ever releases the reset and never asserts it, so
 * PWM slices already set up by subsystem 5 (servo, echo timer) survive,
 * whichever subsystem starts first.
 */
static void
pwm_release (void)
{
    if (0u == (in_w(RESETS_DONE_REG) & RESETS_PWM_BIT))
    {
        out_w(RESETS_RESET_REG + ALIAS_CLR, RESETS_PWM_BIT);

        while (0u == (in_w(RESETS_DONE_REG) & RESETS_PWM_BIT))
        {
            // Wait: the reset completes within a few clock cycles.
        }
    }
}

/*!
 * @brief Address of one register of a PWM slice.
 *
 * @param[in] slice  PWM slice, 0 ... 7.
 * @param[in] offset Register offset within the slice.
 *
 * @return The register address.
 */
static uint32_t
pwm_reg (uint32_t slice, uint32_t offset)
{
    return (PWM_ADDR + (PWM_SLICE_STRIDE * slice) + offset);
}

/*!
 * @brief Convert a duty in permille to PWM compare counts.
 *
 * @param[in] duty_permille 0 ... 1000.
 *
 * @return 0 ... PWM_PERIOD_COUNTS.
 */
static uint32_t
duty_to_counts (uint32_t duty_permille)
{
    // 1000 x 6250 fits easily in 32 bits.
    return ((duty_permille * PWM_PERIOD_COUNTS) / DUTY_SCALE);
}

/*** end of file ***/
