/** @file drv_lsm303.c
 *
 * @brief LSM303DLHC driver on I2C0, polled (no kernel I2C driver).
 *        See drv_lsm303.h. Register numbers: LSM303DLHC datasheet,
 *        Table 17.
 *
 * @par Identity
 * The accelerometer has no WHO_AM_I register, so it is checked by writing
 * CTRL_REG1_A and CTRL_REG4_A and reading them back. The magnetometer's
 * IRA/IRB/IRC_REG_M must read 'H' '4' '3'.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_lsm303.h"

/* ---- RP2040 I2C0, driven directly by polling (datasheet 4.3). The
 * kernel's interrupt driver ("iica") took about 1.4 ms per byte on the
 * car (2026-11-01: 320 ms per 20 ms cycle), so it is not used and
 * DEVCNF_USE_IIC stays 0. At 400 kHz one 6-byte sample takes ~0.25 ms. */
#define I2C0_BASE           (0x40044000u)
#define IC_CON              (I2C0_BASE + 0x00u)
#define IC_TAR              (I2C0_BASE + 0x04u)
#define IC_DATA_CMD         (I2C0_BASE + 0x10u)
#define IC_FS_SCL_HCNT      (I2C0_BASE + 0x1Cu)
#define IC_FS_SCL_LCNT      (I2C0_BASE + 0x20u)
#define IC_RAW_INTR_STAT    (I2C0_BASE + 0x34u)
#define IC_RX_TL            (I2C0_BASE + 0x38u)
#define IC_TX_TL            (I2C0_BASE + 0x3Cu)
#define IC_CLR_TX_ABRT      (I2C0_BASE + 0x54u)
#define IC_CLR_STOP_DET     (I2C0_BASE + 0x60u)
#define IC_ENABLE           (I2C0_BASE + 0x6Cu)
#define IC_TXFLR            (I2C0_BASE + 0x74u)
#define IC_RXFLR            (I2C0_BASE + 0x78u)
#define IC_SDA_HOLD         (I2C0_BASE + 0x7Cu)
#define IC_FS_SPKLEN        (I2C0_BASE + 0xA0u)

#define CON_MASTER_FAST     (0x165u)    /* Master, fast, restart, no slave,
                                           TX_EMPTY_CTRL.                  */
#define SCL_HCNT_400K       (126u)      /* Same as the kernel driver.      */
#define SCL_LCNT_400K       (187u)
#define SDA_HOLD_CYCLES     (38u)
#define SPIKE_LEN           (11u)
#define CMD_READ            (0x100u)
#define CMD_STOP            (0x200u)
#define CMD_RESTART         (0x400u)
#define RAW_TX_ABRT         (0x040u)
#define RAW_STOP_DET        (0x200u)
#define TX_FIFO_DEPTH       (16u)
#define DATA_MASK           (0xFFu)

#define S4_RESETS_REG        (0x4000C000u)
#define S4_RESETS_DONE         (0x4000C008u)
#define RESET_I2C0          (0x00000008u)
#define S4_GPIO_CTRL(pin)      (0x40014004u + ((pin) * 8u))
#define S4_PAD_GPIO(pin)       (0x4001C004u + ((pin) * 4u))
#define FUNCSEL_I2C         (3u)
#define PAD_IE_4MA_PU_SMT   (0x5Au)     /* IE, 4 mA, pull-up, Schmitt.     */
#define PIN_SDA             (4u)
#define PIN_SCL             (5u)

#define TIMER_RAWL          (0x40054028u)
#define XFER_TIMEOUT_US     (3000u)

#define ADDR_ACC            (0x19u)
#define ADDR_MAG            (0x1Eu)

#define REG_CTRL1_A         (0x20u)
#define REG_CTRL4_A         (0x23u)
#define REG_CTRL5_A         (0x24u)
#define REG_OUT_X_L_A       (0x28u)
#define REG_FIFO_CTRL_A     (0x2Eu)
#define REG_FIFO_SRC_A      (0x2Fu)
#define REG_IRA_M           (0x0Au)
#define AUTO_INCREMENT      (0x80u)     /* MSB of the sub-address.        */

#define CTRL1_400HZ_XYZ     (0x77u)     /* ODR 400 Hz, X, Y, Z on.        */
#define CTRL4_BDU_8G_HR     (0xA8u)     /* BDU, +/-8 g, high resolution.  */
#define CTRL5_FIFO_EN       (0x40u)
#define FIFO_BYPASS         (0x00u)     /* Also empties the FIFO.         */
#define FIFO_STREAM         (0x80u)
#define FIFO_SRC_OVRN       (0x40u)
#define FIFO_SRC_EMPTY      (0x20u)
#define FIFO_SRC_FSS        (0x1Fu)

#define MAG_ID_A            ('H')
#define MAG_ID_B            ('4')
#define MAG_ID_C            ('3')

#define SAMPLE_BYTES        (6u)
#define LEFT_JUSTIFY_DIV    (16)        /* 12-bit data, left-justified.   */

static ER  reg_read(uint8_t addr, uint8_t reg, uint8_t * p_buf, uint32_t len);
static ER  reg_write(uint8_t addr, uint8_t reg, uint8_t value);
static ER  xfer_finish(uint32_t start_us);
static void i2c_select(uint8_t addr);
static int16_t to_mg(uint8_t lo, uint8_t hi);

/*!
 * @brief Take I2C0 out of reset, route GP4/GP5 to it, 400 kHz master.
 *        Call once, from imu_task.
 *
 * @return E_OK.
 */
ER
drv_lsm303_open (void)
{
    out_w(S4_RESETS_REG, in_w(S4_RESETS_REG) | RESET_I2C0);
    out_w(S4_RESETS_REG, in_w(S4_RESETS_REG) & ~RESET_I2C0);
    while (0u == (in_w(S4_RESETS_DONE) & RESET_I2C0))
    {
        // Wait for the block to leave reset (a few cycles).
    }

    out_w(S4_PAD_GPIO(PIN_SDA), PAD_IE_4MA_PU_SMT);
    out_w(S4_PAD_GPIO(PIN_SCL), PAD_IE_4MA_PU_SMT);
    out_w(S4_GPIO_CTRL(PIN_SDA), FUNCSEL_I2C);
    out_w(S4_GPIO_CTRL(PIN_SCL), FUNCSEL_I2C);

    out_w(IC_ENABLE, 0u);
    out_w(IC_CON, CON_MASTER_FAST);
    out_w(IC_FS_SCL_HCNT, SCL_HCNT_400K);
    out_w(IC_FS_SCL_LCNT, SCL_LCNT_400K);
    out_w(IC_FS_SPKLEN, SPIKE_LEN);
    out_w(IC_SDA_HOLD, SDA_HOLD_CYCLES);
    out_w(IC_RX_TL, 0u);
    out_w(IC_TX_TL, 0u);

    return (E_OK);
}

/*!
 * @brief Configure the accelerometer and its FIFO, then check both
 *        chips' identity.
 *
 * @param[out] p_info What was read back.
 *
 * @return E_OK if the accelerometer answered and read back correctly;
 *         E_IO if it read back wrong; another error if I2C failed.
 *         A wrong magnetometer identity is reported in p_info only.
 */
ER
drv_lsm303_init (drv_lsm303_info_t * p_info)
{
    ER er = E_OK;

    p_info->ctrl1    = 0u;
    p_info->ctrl4    = 0u;
    p_info->b_acc_ok = false;
    p_info->b_mag_ok = false;
    p_info->mag_id[0] = 0u;
    p_info->mag_id[1] = 0u;
    p_info->mag_id[2] = 0u;

    er = reg_write(ADDR_ACC, REG_CTRL1_A, CTRL1_400HZ_XYZ);
    if (E_OK == er)
    {
        er = reg_write(ADDR_ACC, REG_CTRL4_A, CTRL4_BDU_8G_HR);
    }
    if (E_OK == er)
    {
        er = reg_write(ADDR_ACC, REG_CTRL5_A, CTRL5_FIFO_EN);
    }
    if (E_OK == er)
    {
        er = reg_write(ADDR_ACC, REG_FIFO_CTRL_A, FIFO_BYPASS);
    }
    if (E_OK == er)
    {
        er = reg_write(ADDR_ACC, REG_FIFO_CTRL_A, FIFO_STREAM);
    }
    if (E_OK == er)
    {
        er = reg_read(ADDR_ACC, REG_CTRL1_A, &p_info->ctrl1, 1u);
    }
    if (E_OK == er)
    {
        er = reg_read(ADDR_ACC, REG_CTRL4_A, &p_info->ctrl4, 1u);
    }
    if (E_OK == er)
    {
        p_info->b_acc_ok = (CTRL1_400HZ_XYZ == p_info->ctrl1)
                           && (CTRL4_BDU_8G_HR == p_info->ctrl4);
        if (!p_info->b_acc_ok)
        {
            er = E_IO;
        }
    }

    // The magnetometer is only a cross-check: its result does not change
    // the return value.
    if (E_OK == reg_read(ADDR_MAG, REG_IRA_M, p_info->mag_id, 3u))
    {
        p_info->b_mag_ok = ((uint8_t) MAG_ID_A == p_info->mag_id[0])
                           && ((uint8_t) MAG_ID_B == p_info->mag_id[1])
                           && ((uint8_t) MAG_ID_C == p_info->mag_id[2]);
    }

    return (er);
}

/*!
 * @brief Number of unread samples in the accelerometer FIFO.
 *
 * @param[out] p_count     0 ... 32.
 * @param[out] p_b_overrun true if the FIFO was full (samples were lost).
 *
 * @return E_OK, or the I2C error.
 */
ER
drv_lsm303_fifo_count (uint32_t * p_count, bool * p_b_overrun)
{
    uint8_t src = 0u;
    ER      er  = reg_read(ADDR_ACC, REG_FIFO_SRC_A, &src, 1u);

    *p_count     = 0u;
    *p_b_overrun = false;

    if (E_OK == er)
    {
        if (0u != (src & FIFO_SRC_OVRN))
        {
            *p_count     = DRV_LSM303_FIFO_DEPTH;
            *p_b_overrun = true;
        }
        else if (0u != (src & FIFO_SRC_EMPTY))
        {
            *p_count = 0u;
        }
        else
        {
            *p_count = (uint32_t) (src & FIFO_SRC_FSS);
        }
    }

    return (er);
}

/*!
 * @brief Read (and remove) the oldest sample from the FIFO.
 *
 * @param[out] p_sample The sample in mg.
 *
 * @return E_OK, or the I2C error.
 */
ER
drv_lsm303_fifo_read (drv_lsm303_accel_t * p_sample)
{
    uint8_t raw[SAMPLE_BYTES] = {0u};
    ER      er = reg_read(ADDR_ACC, REG_OUT_X_L_A | AUTO_INCREMENT, raw,
                          SAMPLE_BYTES);

    if (E_OK == er)
    {
        p_sample->x_mg = to_mg(raw[0], raw[1]);
        p_sample->y_mg = to_mg(raw[2], raw[3]);
        p_sample->z_mg = to_mg(raw[4], raw[5]);
    }

    return (er);
}

/*!
 * @brief Write a register address, then read len bytes.
 */
static ER
reg_read (uint8_t addr, uint8_t reg, uint8_t * p_buf, uint32_t len)
{
    uint32_t start  = in_w(TIMER_RAWL);
    uint32_t sent   = 0u;
    uint32_t got    = 0u;
    uint32_t cmd    = 0u;
    ER       er     = E_OK;

    i2c_select(addr);
    out_w(IC_DATA_CMD, (uint32_t) reg);

    // Queue the read commands while there is room, and collect the bytes
    // as they arrive (both FIFOs are 16 deep).
    while ((got < len) && (E_OK == er))
    {
        if ((sent < len) && (in_w(IC_TXFLR) < TX_FIFO_DEPTH))
        {
            cmd = CMD_READ;
            if (0u == sent)
            {
                cmd |= CMD_RESTART;
            }
            if ((len - 1u) == sent)
            {
                cmd |= CMD_STOP;
            }
            out_w(IC_DATA_CMD, cmd);
            sent++;
        }

        if (in_w(IC_RXFLR) > 0u)
        {
            // NOTE: The cast is safe: the data is masked to 8 bits.
            p_buf[got] = (uint8_t) (in_w(IC_DATA_CMD) & DATA_MASK);
            got++;
        }

        if (0u != (in_w(IC_RAW_INTR_STAT) & RAW_TX_ABRT))
        {
            er = E_IO;      // No ACK: chip missing or wrong address.
        }
        else if ((in_w(TIMER_RAWL) - start) > XFER_TIMEOUT_US)
        {
            er = E_TMOUT;
        }
        else
        {
            // Keep going.
        }
    }

    if (E_OK == er)
    {
        er = xfer_finish(start);
    }
    else
    {
        (void) in_w(IC_CLR_TX_ABRT);
        out_w(IC_ENABLE, 0u);
    }

    return (er);
}

/*!
 * @brief Write one register.
 */
static ER
reg_write (uint8_t addr, uint8_t reg, uint8_t value)
{
    uint32_t start = in_w(TIMER_RAWL);

    i2c_select(addr);
    out_w(IC_DATA_CMD, (uint32_t) reg);
    out_w(IC_DATA_CMD, (uint32_t) value | CMD_STOP);

    return (xfer_finish(start));
}

/*!
 * @brief Point I2C0 at a slave address and enable it.
 */
static void
i2c_select (uint8_t addr)
{
    out_w(IC_ENABLE, 0u);
    out_w(IC_TAR, (uint32_t) addr);
    out_w(IC_ENABLE, 1u);
    (void) in_w(IC_CLR_TX_ABRT);
    (void) in_w(IC_CLR_STOP_DET);
}

/*!
 * @brief Wait for the STOP condition (or an abort / timeout), then
 *        disable I2C0.
 */
static ER
xfer_finish (uint32_t start_us)
{
    ER       er  = E_OK;
    uint32_t raw = 0u;

    do
    {
        raw = in_w(IC_RAW_INTR_STAT);
        if (0u != (raw & RAW_TX_ABRT))
        {
            er = E_IO;
        }
        else if ((in_w(TIMER_RAWL) - start_us) > XFER_TIMEOUT_US)
        {
            er = E_TMOUT;
        }
        else
        {
            // Not finished yet.
        }
    } while ((E_OK == er) && (0u == (raw & RAW_STOP_DET)));

    (void) in_w(IC_CLR_TX_ABRT);
    (void) in_w(IC_CLR_STOP_DET);
    out_w(IC_ENABLE, 0u);

    return (er);
}

/*!
 * @brief Convert one left-justified 12-bit reading to mg.
 */
static int16_t
to_mg (uint8_t lo, uint8_t hi)
{
    // NOTE: The cast to int16_t reinterprets the two's-complement reading.
    int16_t raw = (int16_t) (uint16_t) ((uint16_t) lo | ((uint16_t) hi << 8));
    int32_t mg  = ((int32_t) raw / LEFT_JUSTIFY_DIV) * DRV_LSM303_MG_PER_LSB;

    // NOTE: The cast is safe: |mg| <= 2048 x 4 = 8192.
    return ((int16_t) mg);
}

/*** end of file ***/
