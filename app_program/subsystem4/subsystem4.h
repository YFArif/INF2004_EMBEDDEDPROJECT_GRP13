/** @file subsystem4.h
 *
 * @brief Subsystem 4 -- IMU-based motion and terrain monitoring
 *        ("Physics Watchdog"). Student: Kessler, 2502114.
 *
 * @par
 * Public interface of subsystem 4: what other subsystems may use. Other
 * subsystems include it as "../subsystem4/subsystem4.h". The section
 * marked "Private to subsystem 4" is for S4's own files only.
 *
 * @par Task
 * imu_task (priority 10, core 0, every 20 ms): owns I2C0 and the
 * LSM303DLHC (GP4 SDA, GP5 SCL, Grove 3). Calibration, pitch with the
 * drive acceleration removed, hump detection and peak height, lifted /
 * overturned stop, turn rate, motion class (Week 6 report 6.5).
 *
 * @par Links to the other subsystems
 * - S2: reads odometry (subsystem2_odom_get); when lifted or overturned
 *   sets MF_ESTOP
 *   (subsystem2_estop_set).
 * - S5: when lifted signals EV_LIFTED, when overturned EV_OVERTURNED
 *   (subsystem5_mission_signal, after MF_ESTOP); on an
 *   IMU fault sets S5_FAULT_IMU (subsystem5_fault_set), a warning only.
 *   S5 reads subsystem4_cal_done() while CALIBRATING.
 * - S1: reads subsystem4_terrain_get() for telemetry.
 *
 * @par Minimum-bit data (Tribunal 1 curveball)
 * The terrain data is stored packed in 6 bytes (43 bits, see
 * s4_terrain.h); readers get an unpacked copy. Signed fields are clamped,
 * then stored offset-binary, so a value can never wrap.
 *
 * @par Deviations from BARR-C:2018 (same as S2 and S5)
 * - micro T-Kernel types (ID, ER, INT, UINT, T_xxx) appear only where a
 *   kernel call or the task entry signature requires them.
 * - The template's build uses -std=gnu11 rather than C99 (rule 1.1a).
 * - The template's file names subsystem1 ... subsystem5 are not unique in
 *   their first 8 characters (rule 4.1b).
 * - Template section below: mbf_summary and flg_events are declared
 *   extern (rule 4.2c) because S3's example consumer_task still sends to
 *   them. Delete them with that example.
 *
 * @par Abbreviations used in S4 (BARR-C 1.5b)
 * acc: accelerometer; cal: calibration; cdeg: 1/100 degree; cyc: cyclic
 * handler; dps: degree per second; flg: event flag; hdeg: 1/2 degree;
 * mag: magnetometer; mg: 1/1000 g; mms: mm/s; mms2: mm/s^2; mtx: mutex;
 * odom: odometry; pri: priority; us: microsecond.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

/* ---------------------------------------------------------------------
 *  IF-08 terrain: read by S1 (telemetry) and S5 (controller).
 * --------------------------------------------------------------------- */

/** Hump state (2 bits). */
typedef enum
{
    S4_HUMP_FLAT = 0,
    S4_HUMP_CLIMBING,       /* Nose up past the threshold: ON_HUMP.      */
    S4_HUMP_DESCENDING      /* Nose down after a climb: ON_HUMP.         */
} subsystem4_hump_t;

/** Motion class (3 bits), Week 6 report 6.5. */
typedef enum
{
    S4_CLASS_STATIONARY = 0,
    S4_CLASS_MOVING,
    S4_CLASS_ACCELERATING,  /* Speeding up or braking.                   */
    S4_CLASS_TURNING,
    S4_CLASS_CLIMBING_HUMP,
    S4_CLASS_DESCENDING_HUMP,
    S4_CLASS_IMPACT             /* Stopped: lifted / overturned.  */
} subsystem4_class_t;

/** Terrain, unpacked for the reader. Resolution is what is stored. */
typedef struct
{
    int16_t pitch_hdeg;     /* 1/2 degree, + = nose up, -64 ... 63.      */
    int16_t roll_hdeg;      /* 1/2 degree, + = left side up, -64 ... 63. */
    int16_t turn_rate_dps;  /* Degree/s, + = left, -512 ... 511.         */
    uint8_t peak_mm;        /* Highest hump of the run, 0 ... 255 mm.    */
    uint8_t hump_state;     /* subsystem4_hump_t.                        */
    uint8_t motion_class;   /* subsystem4_class_t.                       */
    uint8_t hump_count;     /* Humps passed, 0 ... 7 (saturates).        */
    bool    b_imu_ok;       /* false: IMU_DEGRADED, the rest is stale.   */
    bool    b_cal_done;     /* Offsets measured (S5 CALIBRATING).        */
    bool    b_impact;       /* Lift stop latched until MF_ESTOP clears.  */
} subsystem4_terrain_t;

/** imu_task's health and timing, for the tests and telemetry. */
typedef struct
{
    bool     b_started;     /* Init attempted at least once.             */
    bool     b_acc_ok;      /* Accelerometer read back correctly.        */
    bool     b_mag_ok;      /* Magnetometer identity 'H' '4' '3'.        */
    uint8_t  mag_id[3];
    uint32_t cycles;        /* 20 ms cycles run.                         */
    uint32_t run_max_us;    /* Longest cycle run time.                   */
    uint32_t late_max_us;   /* Latest start after its due time.          */
    uint32_t fifo_overruns; /* Cycles that found the FIFO full.          */
    uint32_t i2c_errors;
    uint32_t impacts;       /* Lift / overturn stops.                    */
} subsystem4_status_t;

/* Public functions: any task may call these. */
void subsystem4_terrain_get(subsystem4_terrain_t * p_terrain);
bool subsystem4_cal_done(void);
void subsystem4_calibrate(void);    /* Ask for a new calibration.        */
void subsystem4_status_get(subsystem4_status_t * p_status);

/* ---------------------------------------------------------------------
 *  Template example (IF-03 of the course example), kept only because
 *  S3's consumer_task still sends to it with TMO_POL. Nobody reads it
 *  any more, so its lines are simply dropped when the buffer is full.
 * --------------------------------------------------------------------- */
#define S4_SUMMARY_MAX          (48)
#define FLG_BATCH               (0x0002)    /* Set by subsystem 3.        */

IMPORT ID mbf_summary;
IMPORT ID flg_events;

/* ---------------------------------------------------------------------
 *  Private to subsystem 4
 * --------------------------------------------------------------------- */
#define S4_PRI_IMU              (10)    /* Week 6 report section 3.       */
#define S4_IMU_PERIOD_MS        (20u)

#define S4_EV_TICK              (0x01u) /* flg_imu: 20 ms period.         */
#define S4_EV_CAL               (0x02u) /* flg_imu: recalibrate.          */

void imu_task(INT stacd, void * exinf);
uint32_t subsystem4_wait(void);
void subsystem4_terrain_put(subsystem4_terrain_t const * p_terrain);
void subsystem4_status_put(subsystem4_status_t const * p_status);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
