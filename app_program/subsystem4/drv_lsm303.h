/** @file drv_lsm303.h
 *
 * @brief LSM303DLHC driver (GY-511 board) on I2C0, GP4 = SDA, GP5 = SCL
 *        (Robo Pico Grove 3). Only imu_task may call these functions:
 *        it is the owning task of I2C0 (TEAM.md section 3).
 *
 * @par Configuration
 * Accelerometer at 0x19: 400 Hz, +/-8 g, high resolution, block data
 * update, FIFO in stream mode (32 samples). Magnetometer at 0x1E: only
 * its identity is read ('H' '4' '3'); it is a cross-check (Week 6 report
 * 6.5), not used for control.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#define DRV_LSM303_FIFO_DEPTH   (32u)   /* Accelerometer FIFO samples.    */
#define DRV_LSM303_MG_PER_LSB   (4)     /* +/-8 g, high resolution.       */

/** One accelerometer sample in the sensor's own axes, in mg. */
typedef struct
{
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
} drv_lsm303_accel_t;

/** What drv_lsm303_init() found. */
typedef struct
{
    uint8_t ctrl1;          /* CTRL_REG1_A read back (expect 0x77).      */
    uint8_t ctrl4;          /* CTRL_REG4_A read back (expect 0xA8).      */
    uint8_t mag_id[3];      /* IRA/IRB/IRC_REG_M (expect 'H' '4' '3').   */
    bool    b_acc_ok;       /* Read-back matched.                        */
    bool    b_mag_ok;       /* Identity matched.                         */
} drv_lsm303_info_t;

ER drv_lsm303_open(void);
ER drv_lsm303_init(drv_lsm303_info_t * p_info);
ER drv_lsm303_fifo_count(uint32_t * p_count, bool * p_b_overrun);
ER drv_lsm303_fifo_read(drv_lsm303_accel_t * p_sample);

#endif /* DRV_LSM303_H */

/*** end of file ***/
