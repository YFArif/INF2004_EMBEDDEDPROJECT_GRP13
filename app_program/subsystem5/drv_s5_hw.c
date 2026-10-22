/** @file drv_s5_hw.c
 *
 * @brief RP2040 register access for the subsystem 5 drivers.
 *
 * @par
 * Register addresses and bits are from the RP2040 datasheet: RESETS
 * (2.14), IO_BANK0 and PADS_BANK0 (2.19), SIO (2.3.1), TIMER (4.6) and
 * PWM (4.5). They are written here as unsigned constants rather than
 * taken from the kernel's sysdef.h, whose bit macros are signed ints
 * (BARR-C 5.3).
 *
 * @par
 * Peripheral registers are written through the RP2040's atomic set and
 * clear aliases, so other cores and tasks touching other bits of the
 * same register (for example RESETS) are never disturbed. SIO has no
 * aliases; it has its own set and clear registers instead.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>     /* in_w() and out_w() register access.      */

#include "drv_s5_hw.h"

/* Atomic register aliases for APB peripherals (datasheet 2.1.2). */
#define ALIAS_SET           (0x2000u)
#define ALIAS_CLR           (0x3000u)

/* RESETS. */
#define RESETS_BASE_ADDR    (0x4000C000u)
#define RESETS_RESET_REG    (RESETS_BASE_ADDR + 0x00u)
#define RESETS_DONE_REG     (RESETS_BASE_ADDR + 0x08u)
#define RESETS_PWM_BIT      (0x00004000u)

/* IO_BANK0: one 8-byte block per pin; CTRL is the second word. */
#define IO_BANK0_ADDR       (0x40014000u)
#define IO_CTRL_STRIDE      (8u)
#define IO_CTRL_OFFSET      (4u)
#define IO_FUNCSEL_PWM      (4u)
#define IO_FUNCSEL_SIO      (5u)

/* PADS_BANK0: one word per pin after the voltage-select word. */
#define PADS_BANK0_ADDR     (0x4001C000u)
#define PADS_STRIDE         (4u)
#define PADS_OFFSET         (4u)
#define PAD_OD_BIT          (0x80u)     /* Output disable.               */
#define PAD_IE_BIT          (0x40u)     /* Input enable.                 */
#define PAD_PUE_BIT         (0x08u)     /* Pull-up enable.               */
#define PAD_PDE_BIT         (0x04u)     /* Pull-down enable.             */

/* SIO (single-cycle I/O). */
#define SIO_ADDR            (0xD0000000u)
#define SIO_GPIO_IN         (SIO_ADDR + 0x004u)
#define SIO_GPIO_OUT_SET    (SIO_ADDR + 0x014u)
#define SIO_GPIO_OUT_CLR    (SIO_ADDR + 0x018u)
#define SIO_GPIO_OE_SET     (SIO_ADDR + 0x024u)

/* TIMER: raw low word of the 1 MHz counter (reading it latches nothing). */
#define TIMER_RAWL_REG      (0x40054028u)

/* PWM: eight slices of five registers. */
#define PWM_ADDR            (0x40050000u)
#define PWM_SLICE_STRIDE    (0x14u)
#define PWM_CSR_OFFSET      (0x00u)
#define PWM_DIV_OFFSET      (0x04u)
#define PWM_CTR_OFFSET      (0x08u)
#define PWM_CC_OFFSET       (0x0Cu)
#define PWM_TOP_OFFSET      (0x10u)
#define PWM_CSR_EN_BIT      (0x01u)
#define PWM_DIV_INT_SHIFT   (4u)
#define PWM_CC_A_MASK       (0x0000FFFFu)
#define PWM_SLICE_MASK      (0x07u)

static uint32_t pwm_reg(uint32_t slice, uint32_t offset);
static uint32_t pad_reg(uint32_t pin);

/*!
 * @brief Take the PWM block out of reset if it is still held there.
 *
 * NOTE: This only ever releases the reset. It never asserts it, because
 * that would also stop the motor PWM slices owned by subsystem 2.
 */
void
drv_s5_hw_pwm_release (void)
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
 * @brief PWM slice that drives (or reads) a pin.
 *
 * @param[in] pin GPIO number, 0 to 29.
 *
 * @return Slice number, 0 to 7.
 */
uint32_t
drv_s5_hw_pwm_slice (uint32_t pin)
{
    return ((pin >> 1u) & PWM_SLICE_MASK);
}

/*!
 * @brief Configure and start a PWM slice.
 *
 * @param[in] slice   Slice number, 0 to 7.
 * @param[in] div_int Integer clock divider, 1 to 255.
 * @param[in] top     Counter wrap value.
 * @param[in] mode    DRV_S5_HW_PWM_FREE_RUN or DRV_S5_HW_PWM_GATED_B.
 */
void
drv_s5_hw_pwm_setup (uint32_t slice, uint32_t div_int, uint32_t top,
                     uint32_t mode)
{
    out_w(pwm_reg(slice, PWM_CSR_OFFSET), 0u);
    out_w(pwm_reg(slice, PWM_DIV_OFFSET), div_int << PWM_DIV_INT_SHIFT);
    out_w(pwm_reg(slice, PWM_TOP_OFFSET), top);
    out_w(pwm_reg(slice, PWM_CC_OFFSET), 0u);
    out_w(pwm_reg(slice, PWM_CTR_OFFSET), 0u);
    out_w(pwm_reg(slice, PWM_CSR_OFFSET), mode | PWM_CSR_EN_BIT);
}

/*!
 * @brief Set the compare level of a slice's channel A.
 *
 * @param[in] slice Slice number, 0 to 7.
 * @param[in] level Counts the output stays high in each period.
 */
void
drv_s5_hw_pwm_set_a (uint32_t slice, uint32_t level)
{
    uint32_t cc_reg  = pwm_reg(slice, PWM_CC_OFFSET);
    uint32_t level_b = in_w(cc_reg) & ~PWM_CC_A_MASK;

    out_w(cc_reg, level_b | (level & PWM_CC_A_MASK));
}

/*!
 * @brief Read a slice's counter.
 *
 * @param[in] slice Slice number, 0 to 7.
 *
 * @return Counter value, 0 to 65535.
 */
uint32_t
drv_s5_hw_pwm_count (uint32_t slice)
{
    return (in_w(pwm_reg(slice, PWM_CTR_OFFSET)) & PWM_CC_A_MASK);
}

/*!
 * @brief Reset a slice's counter to zero.
 *
 * @param[in] slice Slice number, 0 to 7.
 */
void
drv_s5_hw_pwm_clear_count (uint32_t slice)
{
    out_w(pwm_reg(slice, PWM_CTR_OFFSET), 0u);
}

/*!
 * @brief Connect a pin to its PWM slice.
 *
 * @param[in] pin     GPIO number.
 * @param[in] b_input true for a channel B input (output driver off,
 *                    pull-down on so an unplugged sensor reads low);
 *                    false for an output.
 */
void
drv_s5_hw_pin_to_pwm (uint32_t pin, bool b_input)
{
    uint32_t pad = pad_reg(pin);

    if (b_input)
    {
        out_w(pad + ALIAS_CLR, PAD_PUE_BIT);
        out_w(pad + ALIAS_SET, PAD_OD_BIT | PAD_IE_BIT | PAD_PDE_BIT);
    }
    else
    {
        out_w(pad + ALIAS_CLR, PAD_OD_BIT | PAD_PUE_BIT | PAD_PDE_BIT);
        out_w(pad + ALIAS_SET, PAD_IE_BIT);
    }

    out_w(IO_BANK0_ADDR + (IO_CTRL_STRIDE * pin) + IO_CTRL_OFFSET,
          IO_FUNCSEL_PWM);
}

/*!
 * @brief Make a pin a software-driven output, starting low.
 *
 * @param[in] pin GPIO number.
 */
void
drv_s5_hw_pin_output (uint32_t pin)
{
    uint32_t mask = 1u << pin;

    out_w(SIO_GPIO_OUT_CLR, mask);
    out_w(SIO_GPIO_OE_SET, mask);
    out_w(pad_reg(pin) + ALIAS_CLR, PAD_OD_BIT | PAD_PUE_BIT | PAD_PDE_BIT);
    out_w(IO_BANK0_ADDR + (IO_CTRL_STRIDE * pin) + IO_CTRL_OFFSET,
          IO_FUNCSEL_SIO);
}

/*!
 * @brief Make a pin a plain input with the pad pull-up on, e.g. for a
 *        button that pulls the pin to ground when pressed.
 *
 * @param[in] pin GPIO number.
 *
 * NOTE: The SIO output enable is left as it is (off after reset), so the
 * pin is never driven.
 */
void
drv_s5_hw_pin_input_pullup (uint32_t pin)
{
    uint32_t pad = pad_reg(pin);

    out_w(pad + ALIAS_CLR, PAD_PDE_BIT);
    out_w(pad + ALIAS_SET, PAD_IE_BIT | PAD_PUE_BIT);
    out_w(IO_BANK0_ADDR + (IO_CTRL_STRIDE * pin) + IO_CTRL_OFFSET,
          IO_FUNCSEL_SIO);
}

/*!
 * @brief Drive an output pin high or low.
 *
 * @param[in] pin    GPIO number set up with drv_s5_hw_pin_output().
 * @param[in] b_high true for high.
 */
void
drv_s5_hw_pin_write (uint32_t pin, bool b_high)
{
    if (b_high)
    {
        out_w(SIO_GPIO_OUT_SET, 1u << pin);
    }
    else
    {
        out_w(SIO_GPIO_OUT_CLR, 1u << pin);
    }
}

/*!
 * @brief Read a pin's input level, whatever function it is set to.
 *
 * @param[in] pin GPIO number.
 *
 * @return true if the pin is high.
 */
bool
drv_s5_hw_pin_read (uint32_t pin)
{
    return (0u != (in_w(SIO_GPIO_IN) & (1u << pin)));
}

/*!
 * @brief Current time from the RP2040's 1 MHz timer.
 *
 * @return Microseconds, wrapping every 71.6 minutes. Differences of two
 *         readings are correct across the wrap when taken as uint32_t.
 */
uint32_t
drv_s5_hw_now_us (void)
{
    return (in_w(TIMER_RAWL_REG));
}

/*!
 * @brief Busy-wait for a short time.
 *
 * @param[in] delay_us Microseconds to wait.
 *
 * WARNING: This keeps the CPU busy. Use it only for delays of a few tens
 * of microseconds; use tk_dly_tsk() for anything longer.
 */
void
drv_s5_hw_wait_us (uint32_t delay_us)
{
    uint32_t start_us = drv_s5_hw_now_us();

    while ((drv_s5_hw_now_us() - start_us) < delay_us)
    {
        // Busy-wait: the delay is too short for the 10 ms kernel tick.
    }
}

/*!
 * @brief Address of one register of a PWM slice.
 *
 * @param[in] slice  Slice number, 0 to 7.
 * @param[in] offset Register offset inside the slice.
 *
 * @return Register address.
 */
static uint32_t
pwm_reg (uint32_t slice, uint32_t offset)
{
    return (PWM_ADDR + (PWM_SLICE_STRIDE * slice) + offset);
}

/*!
 * @brief Address of a pin's pad-control register.
 *
 * @param[in] pin GPIO number.
 *
 * @return Register address.
 */
static uint32_t
pad_reg (uint32_t pin)
{
    return (PADS_BANK0_ADDR + PADS_OFFSET + (PADS_STRIDE * pin));
}

/*** end of file ***/
