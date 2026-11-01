/** @file imu_task.c
 *
 * @brief imu_task: the "Physics Watchdog" (Week 6 report 6.5).
 *
 * @par Every 20 ms (cyc_imu)
 * 1. Read every new accelerometer sample from the FIFO (400 Hz, about 8
 *    per cycle) and turn it into robot axes: forward, left, up (mg).
 * 2. Impact: per sample, |a_xy - a_xy_lp| > 1.0 g (forward and left only,
 *    gravity and slow changes removed) sets MF_ESTOP and EV_IMPACT.
 * 3. Pitch = atan2(a_fwd_measured - a_fwd_drive, sqrt(left^2 + up^2)),
 *    minus the calibration offset. a_fwd_drive comes from S2's speed
 *    over the last 100 ms, so speeding up or braking is not read as tilt.
 * 4. Hump: h += sin(pitch) x (distance driven this cycle). FLAT ->
 *    CLIMBING (pitch > +5 deg for 5 cycles) -> DESCENDING (pitch < -5 deg
 *    for 5 cycles) -> FLAT (|pitch| < 2 deg for 5 cycles): the hump's
 *    highest h is reported, and the run's peak is kept.
 * 5. Turn rate from the wheel speeds; motion class (held 5 cycles).
 * 6. Publish the packed terrain; print a status line every 200 ms.
 *
 * @par Calibration
 * At start-up and on subsystem4_calibrate(): 2 s still and level. The
 * mean gravity gives the "up" axis and the pitch and roll offsets. If the
 * robot moves, calibration starts again.
 *
 * @par Faults
 * An I2C error after 3 tries, or no new sample for 5 cycles (frozen IMU),
 * gives IMU_DEGRADED: imu_ok = 0 and S5_FAULT_IMU (a warning, the run
 * goes on). The chip is set up again every 1 s until it answers.
 *
 * @par Mounting
 * IMU_FWD_AXIS / IMU_FWD_SIGN say which sensor axis points to the front
 * of the robot. Check: tilt the nose up and "p=" must go positive.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem2/subsystem2.h"
#include "../subsystem5/subsystem5.h"
#include "drv_lsm303.h"
#include "s4_math.h"
#include "s4_terrain.h"
#include "subsystem4.h"

/* ---- Mounting: sensor axis that points forward (0 = x, 1 = y, 2 = z). */
#define IMU_FWD_AXIS        (1u)
#define IMU_FWD_SIGN        (-1)        /* Forward = -y: speeding up read */
                                        /* +242 mg on -y as the car sped  */
                                        /* up (2026-11-01 drive test).    */

/* ---- Hump drive test (S4-U02): 10 s after calibration, drive straight
 * DRIVE_TEST_MM at DRIVE_TEST_MMS with S2's MOVE, with impact detection
 * off. Keep 0 in the shared code: with 1 the car drives off by itself. */
#define DRIVE_TEST          (0)
#define DRIVE_TEST_MM       (800)
#define DRIVE_TEST_MMS      (150u)
#define DRIVE_WAIT_CYCLES   (500u)      /* 10 s.                          */
#define DRIVE_TELL_CYCLES   (100u)      /* Countdown every 2 s.           */

/* ---- Start-up and calibration. */
#define BOOT_WAIT_MS        (3000)      /* Time to open a serial monitor. */
#define CAL_CYCLES          (100u)      /* 2 s at 20 ms.                  */
#define CAL_SPREAD_MG       (40)        /* Each axis may vary this much.  */
#define CAL_STILL_MMS       (10)

/* ---- Pitch and hump (preliminary, S4-U01 / S4-U02). */
#define AFWD_CORRECTION     (1)         /* 0: raw pitch, for S4-U01 a/b.  */
#define HUMP_ENTER_CDEG     (500)
#define HUMP_FLAT_CDEG      (200)
#define HUMP_HOLD_CYCLES    (5u)
#define WHEELBASE_MM        (78)        /* Axle to ball, 77.5 mm (ruler). */
#define HUMP_ABANDON_CYCLES (100u)      /* 2 s level and still: not a hump.*/
#define ANGLE_LP_DIV        (4)         /* Pitch / roll low-pass.         */
#define DRIVE_LOG_CYCLES    (400u)      /* Log every cycle for 8 s.       */
#define HUMP_STILL_MMS      (10)

/* ---- Impact (A18, preliminary, S4-U03). */
#define IMPACT_ENABLE       (0 == DRIVE_TEST)   /* Off in the hump drive  */
                                                /* test: hitting a strip  */
                                                /* edge read 2.1 g.       */
#define IMPACT_MG           (2500)      /* Was 1000 (A18). 2026-11-01: a   */
                                        /* hand tap read 1.2 g, a wheel   */
                                        /* hitting a 2 cm strip edge      */
                                        /* 2.1 g. Raised so hump edges    */
                                        /* do not E-stop; tune in S4-U03. */
#define IMPACT_LP_DIV       (16)        /* Low-pass: 1/16 per sample.     */
#define LATENCY_CYCLES      (10u)       /* Look for S2's brake time.      */

/* ---- Motion class (preliminary, S4-U05). */
#define CLASS_STILL_MMS     (10)
#define CLASS_ACCEL_MMS2    (200)       /* 0.2 m/s^2.                     */
#define CLASS_TURN_CDPS     (3000)      /* 30 deg/s.                      */
#define CLASS_HOLD_CYCLES   (5u)

/* ---- Faults, timing and logging. */
#define I2C_TRIES           (3u)
#define FROZEN_CYCLES       (5u)
#define REINIT_CYCLES       (50u)       /* 1 s.                           */
#define LOG_CYCLES          (10u)       /* Status line every 200 ms.      */
#define SPEED_HISTORY       (6u)        /* Now and 100 ms ago.            */
#define SPEED_PER_S         (10)        /* 100 ms -> per second.          */

#define PERIOD_US           (S4_IMU_PERIOD_MS * 1000u)
#define SAMPLE_US           (2500u)     /* 400 Hz.                        */
#define RESYNC_US           (1000000u)
#define TIMER_RAWL_REG      (0x40054028u)
#define G_MMS2              (9807)
#define MG_PER_G            (1000)
#define CDEG_PER_HDEG       (50)
#define CDPS_PER_DPS        (100)
#define CDPS_PER_DIFF       (572958)    /* (180 / pi) x 100 x 100.        */
#define TRACK_SCALE         (100)
#define AXES                (3u)
#define ROUND_HALF          (0.5f)
#define TENTHS              (10.0f)

typedef struct
{
    int32_t fwd;
    int32_t left;
    int32_t up;
} body_t;

typedef struct
{
    /* Orientation. */
    uint32_t fwd_idx;
    int32_t  fwd_sign;
    uint32_t left_idx;
    int32_t  left_sign;
    uint32_t up_idx;
    int32_t  up_sign;

    /* Calibration. */
    bool     b_calibrating;
    bool     b_cal_done;
    uint32_t cal_cycles;
    uint32_t cal_samples;
    int32_t  cal_sum[AXES];
    int32_t  cal_min[AXES];         /* Per-cycle mean, robot axes.        */
    int32_t  cal_max[AXES];
    int32_t  pitch0_cdeg;
    int32_t  roll0_cdeg;

    /* Health. */
    bool     b_imu_ok;
    uint32_t empty_cycles;
    uint32_t down_cycles;

    /* Impact. */
    int32_t  lp_fwd;
    int32_t  lp_left;
    bool     b_lp_ready;
    bool     b_impact;
    int32_t  dxy2_max;              /* Largest |d|^2 since the last log.  */
    uint32_t impact_us;
    uint32_t latency_wait;
    uint32_t brake_us_before;

    /* Motion. */
    int32_t  speed_hist[SPEED_HISTORY];
    uint32_t speed_idx;
    int32_t  prev_dist_mm;
    int32_t  afwd_mms2;
    int32_t  turn_cdps;

    /* Pitch and hump. */
    body_t   mean;
    int32_t  pitch_raw_cdeg;
    int32_t  pitch_cdeg;
    int32_t  roll_cdeg;
    float    h_mm;
    float    hump_peak_mm;
    int32_t  theta_max_cdeg;
    int32_t  run_peak_mm;
    uint32_t hump_count;
    subsystem4_hump_t hump;
    uint32_t hold;
    uint32_t idle_cycles;           /* On a hump but level and still.    */
    int32_t  hump_sign;             /* +1 nose up first, -1 nose down.   */
    uint32_t drive_cycles;          /* Drive test: cycles since cal done.*/
    bool     b_drive_sent;
    uint32_t drive_log;             /* Per-cycle lines printed so far.   */

    /* One-wheel hump (roll). */
    bool     b_side;
    uint32_t side_hold;
    int32_t  side_roll_max_cdeg;    /* Largest |roll| on this hump.      */
    int32_t  side_start_mm;

    /* Motion class. */
    subsystem4_class_t cls;
    subsystem4_class_t cls_pending;
    uint32_t cls_count;

    /* Timing. */
    uint32_t base_us;
    uint32_t ticks;
    uint32_t log_count;
    uint32_t run_period_max_us;
    uint32_t late_period_max_us;
    uint32_t last_samples;

    subsystem4_status_t status;
} imu_state_t;

static imu_state_t g_imu;

static void     imu_setup(void);
static bool     imu_start_chip(void);
static void     imu_cycle(uint32_t events);
static bool     imu_read_samples(uint32_t start_us);
static void     imu_impact_check(body_t const * p_s, uint32_t sample_us);
static void     imu_motion(subsystem2_odom_t const * p_odom);
static void     imu_pitch(void);
static void     imu_cal_begin(void);
static void     imu_cal_step(void);
static void     imu_orientation(int32_t const * p_mean);
static void     imu_hump(subsystem2_odom_t const * p_odom);
static void     imu_side_hump(subsystem2_odom_t const * p_odom);
static void     imu_class(subsystem2_odom_t const * p_odom);
static void     imu_impact_follow_up(void);
static void     imu_degrade(char const * p_why);
static void     imu_publish(void);
static void     imu_log(void);
static void     imu_timing(uint32_t start_us, uint32_t late_us);
static body_t   to_body(drv_lsm303_accel_t const * p_s);
static int32_t  axis_value(int32_t const * p_v, uint32_t idx, int32_t sign);
static uint32_t now_us(void);
static char const * class_name(subsystem4_class_t cls);

/*!
 * @brief imu_task entry (priority 10, core 0).
 *
 * @param[in] stacd   Unused.
 * @param[in] p_exinf Unused.
 */
void
imu_task (INT stacd, void * p_exinf)
{
    uint32_t events = 0u;

    (void) stacd;
    (void) p_exinf;

    (void) tk_dly_tsk((TMO) BOOT_WAIT_MS);
    imu_setup();

    for (;;)
    {
        events = subsystem4_wait();
        if (0u != events)
        {
            imu_cycle(events);
        }
    }
}

/*!
 * @brief Default orientation, open I2C0, set up the chip, start the
 *        calibration.
 */
static void
imu_setup (void)
{
    int32_t const flat[AXES] = {0, 0, MG_PER_G};   /* Board flat, z up. */

    // Until calibration measures gravity, assume the board lies flat.
    g_imu.fwd_idx   = IMU_FWD_AXIS;
    g_imu.fwd_sign  = IMU_FWD_SIGN;
    imu_orientation(flat);
    g_imu.hump      = S4_HUMP_FLAT;
    g_imu.cls       = S4_CLASS_STATIONARY;

    tm_printf((UB *) "\nS4 IMU: LSM303DLHC on I2C0 SDA=GP4 SCL=GP5\n");
    tm_printf((UB *) "S4 terrain stored in %d B (%d bits); unpacked %d B\n",
              (INT) sizeof(s4_terrain_packed_t), (INT) S4_PACKED_BITS,
              (INT) sizeof(subsystem4_terrain_t));

    (void) drv_lsm303_open();

    g_imu.b_imu_ok = true;
    if (imu_start_chip())
    {
        imu_cal_begin();
    }
    else
    {
        imu_degrade("no reply at 0x19");
    }

    imu_publish();
}

/*!
 * @brief Configure the chip and record what it answered.
 *
 * @return true if the accelerometer read back correctly.
 */
static bool
imu_start_chip (void)
{
    drv_lsm303_info_t info = {0};
    ER                er   = drv_lsm303_init(&info);

    g_imu.status.b_started = true;
    g_imu.status.b_acc_ok  = info.b_acc_ok;
    g_imu.status.b_mag_ok  = info.b_mag_ok;
    g_imu.status.mag_id[0] = info.mag_id[0];
    g_imu.status.mag_id[1] = info.mag_id[1];
    g_imu.status.mag_id[2] = info.mag_id[2];
    subsystem4_status_put(&g_imu.status);

    tm_printf((UB *) "S4 ACC CTRL1=0x%02x CTRL4=0x%02x %s; MAG id=%02x %02x"
              " %02x %s\n", (INT) info.ctrl1, (INT) info.ctrl4,
              info.b_acc_ok ? "PASS" : "FAIL", (INT) info.mag_id[0],
              (INT) info.mag_id[1], (INT) info.mag_id[2],
              info.b_mag_ok ? "PASS" : "FAIL");

    g_imu.empty_cycles = 0u;
    g_imu.b_lp_ready   = false;

    return (E_OK == er);
}

/*!
 * @brief One 20 ms cycle.
 *
 * @param[in] events S4_EV_xxx bits that woke the task.
 */
static void
imu_cycle (uint32_t events)
{
    subsystem2_odom_t odom  = {0};
    uint32_t          start = now_us();
    uint32_t          late  = 0u;

    if (0u != (events & S4_EV_CAL))
    {
        imu_cal_begin();
    }

    if (0u == (events & S4_EV_TICK))
    {
        return;
    }

    // Lateness against the ideal 20 ms grid (S4 + S1 timing test). The
    // grid restarts after a long gap, or if this tick came early.
    late = start - (g_imu.base_us + (g_imu.ticks * PERIOD_US));
    if ((0u == g_imu.ticks) || (late > RESYNC_US))
    {
        g_imu.base_us = start;
        g_imu.ticks   = 0u;
        late          = 0u;
    }
    g_imu.ticks++;

    subsystem2_odom_get(&odom);
    imu_motion(&odom);

    if (!g_imu.b_imu_ok)
    {
        g_imu.down_cycles++;
        if (g_imu.down_cycles >= REINIT_CYCLES)
        {
            g_imu.down_cycles = 0u;
            if (imu_start_chip())
            {
                g_imu.b_imu_ok = true;
                tm_printf((UB *) "S4 IMU recovered\n");
                imu_cal_begin();
            }
        }
    }
    else if (imu_read_samples(start))
    {
        imu_pitch();
        if (g_imu.b_calibrating)
        {
            imu_cal_step();
        }
        else
        {
            imu_hump(&odom);
        }
    }
    else
    {
        // Nothing new this cycle; the old values stay.
    }

    g_imu.prev_dist_mm = odom.dist_mm;

    // Hump drive test: one straight MOVE, 10 s after calibration.
    if ((0 != DRIVE_TEST) && g_imu.b_cal_done && !g_imu.b_drive_sent)
    {
        g_imu.drive_cycles++;
        if (0u == (g_imu.drive_cycles % DRIVE_TELL_CYCLES))
        {
            tm_printf((UB *) "S4 DRIVE TEST: driving %d mm in %d s\n",
                      (INT) DRIVE_TEST_MM,
                      (INT) ((DRIVE_WAIT_CYCLES - g_imu.drive_cycles)
                             / (1000u / S4_IMU_PERIOD_MS)));
        }
        if (g_imu.drive_cycles >= DRIVE_WAIT_CYCLES)
        {
            g_imu.b_drive_sent = true;
            tm_printf((UB *) "S4 DRIVE TEST: go\n");
            CHECK(subsystem2_move((int16_t) DRIVE_TEST_MM, DRIVE_TEST_MMS),
                  "subsystem2_move");
        }
    }

    imu_class(&odom);
    imu_impact_follow_up();
    imu_publish();
    imu_timing(start, late);

    g_imu.log_count++;
    // Drive test: one short line every 20 ms while it drives.
    if (g_imu.b_drive_sent && (g_imu.drive_log < DRIVE_LOG_CYCLES))
    {
        g_imu.drive_log++;
        tm_printf((UB *) "S4 D %d %d %d %d %d\n", (INT) g_imu.drive_log,
                  (INT) g_imu.pitch_cdeg, (INT) g_imu.roll_cdeg,
                  (INT) odom.dist_mm, (INT) (g_imu.h_mm * TENTHS));
    }

    if (g_imu.log_count >= LOG_CYCLES)
    {
        g_imu.log_count = 0u;
        imu_log();
    }
}

/*!
 * @brief Read all new samples, check each for an impact, average them.
 *
 * @param[in] start_us Cycle start, to date the samples.
 *
 * @return true if at least one new sample was read.
 */
static bool
imu_read_samples (uint32_t start_us)
{
    drv_lsm303_accel_t sample = {0};
    body_t             body   = {0};
    body_t             sum    = {0};
    uint32_t           count  = 0u;
    uint32_t           idx    = 0u;
    uint32_t           tries  = 0u;
    bool               b_ovr  = false;
    bool               b_ok   = true;
    ER                 er     = E_IO;

    for (tries = 0u; (tries < I2C_TRIES) && (E_OK != er); tries++)
    {
        er = drv_lsm303_fifo_count(&count, &b_ovr);
        if (E_OK != er)
        {
            g_imu.status.i2c_errors++;
        }
    }
    b_ok = (E_OK == er);

    if (b_ovr)
    {
        g_imu.status.fifo_overruns++;
    }

    for (idx = 0u; b_ok && (idx < count); idx++)
    {
        er = E_IO;
        for (tries = 0u; (tries < I2C_TRIES) && (E_OK != er); tries++)
        {
            er = drv_lsm303_fifo_read(&sample);
            if (E_OK != er)
            {
                g_imu.status.i2c_errors++;
            }
        }
        b_ok = (E_OK == er);

        if (b_ok)
        {
            body = to_body(&sample);
            sum.fwd  += body.fwd;
            sum.left += body.left;
            sum.up   += body.up;

            // Sample idx of count was taken about (count - 1 - idx) x
            // 2.5 ms before this cycle started.
            imu_impact_check(&body,
                             start_us - ((count - 1u - idx) * SAMPLE_US));

            if (g_imu.b_calibrating)
            {
                g_imu.cal_sum[0] += sample.x_mg;
                g_imu.cal_sum[1] += sample.y_mg;
                g_imu.cal_sum[2] += sample.z_mg;
                g_imu.cal_samples++;
            }
        }
    }

    if (!b_ok)
    {
        imu_degrade("I2C error after 3 tries");
        count = 0u;
    }
    else if (0u == count)
    {
        g_imu.empty_cycles++;
        if (g_imu.empty_cycles >= FROZEN_CYCLES)
        {
            imu_degrade("frozen: no new sample for 100 ms");
        }
    }
    else
    {
        g_imu.empty_cycles = 0u;

        // NOTE: The casts are safe: count <= 32.
        g_imu.mean.fwd  = sum.fwd / (int32_t) count;
        g_imu.mean.left = sum.left / (int32_t) count;
        g_imu.mean.up   = sum.up / (int32_t) count;
    }

    g_imu.last_samples = count;

    return (b_ok && (count > 0u));
}

/*!
 * @brief Impact test on one sample (A18): the forward/left acceleration
 *        minus its low-pass. z is left out: hump edges jolt vertically.
 *
 * @param[in] p_s       The sample in robot axes.
 * @param[in] sample_us When it was (about) taken.
 */
static void
imu_impact_check (body_t const * p_s, uint32_t sample_us)
{
    int32_t             d_fwd  = 0;
    int32_t             d_left = 0;
    int32_t             d2     = 0;
    subsystem2_status_t s2     = {0};

    if (!g_imu.b_lp_ready)
    {
        g_imu.lp_fwd     = p_s->fwd;
        g_imu.lp_left    = p_s->left;
        g_imu.b_lp_ready = true;
    }

    d_fwd  = p_s->fwd - g_imu.lp_fwd;
    d_left = p_s->left - g_imu.lp_left;
    g_imu.lp_fwd  += d_fwd / IMPACT_LP_DIV;
    g_imu.lp_left += d_left / IMPACT_LP_DIV;

    // |d| <= 16000 mg, so d^2 + d^2 <= 5.2e8 fits int32_t.
    d2 = (d_fwd * d_fwd) + (d_left * d_left);
    if (d2 > g_imu.dxy2_max)
    {
        g_imu.dxy2_max = d2;
    }

    if ((0 != IMPACT_ENABLE) && g_imu.b_cal_done && !g_imu.b_impact
        && (d2 > (IMPACT_MG * IMPACT_MG)))
    {
        subsystem2_status_get(&s2);
        g_imu.brake_us_before = s2.estop_brake_us;

        // Brake first, then tell the controller.
        subsystem2_estop_set();
        subsystem5_mission_signal(EV_IMPACT);

        g_imu.b_impact     = true;
        g_imu.impact_us    = sample_us;
        g_imu.latency_wait = LATENCY_CYCLES;
        g_imu.status.impacts++;

        // NOTE: The cast is safe: sqrt(5.2e8) < 23000.
        tm_printf((UB *) "S4 IMPACT dxy=%d mg (threshold %d) -> MF_ESTOP,"
                  " EV_IMPACT\n",
                  (INT) s4_math_sqrt((float) d2), (INT) IMPACT_MG);
    }
}

/*!
 * @brief Forward acceleration over the last 100 ms and turn rate, from
 *        S2's odometry.
 *
 * @param[in] p_odom This cycle's odometry.
 */
static void
imu_motion (subsystem2_odom_t const * p_odom)
{
    uint32_t oldest = 0u;

    g_imu.speed_hist[g_imu.speed_idx] = p_odom->speed_mms;
    oldest = (g_imu.speed_idx + 1u) % SPEED_HISTORY;
    g_imu.afwd_mms2 = (p_odom->speed_mms - g_imu.speed_hist[oldest])
                      * SPEED_PER_S;
    g_imu.speed_idx = oldest;

    // omega = (v_right - v_left) / track; + = left.
    g_imu.turn_cdps = ((p_odom->speed_r_mms - p_odom->speed_l_mms)
                       * (CDPS_PER_DIFF / TRACK_SCALE)) / S2_TRACK_MM;
}

/*!
 * @brief Pitch and roll from this cycle's mean, offsets removed.
 */
static void
imu_pitch (void)
{
    float fwd   = (float) g_imu.mean.fwd;
    float left  = (float) g_imu.mean.left;
    float up    = (float) g_imu.mean.up;
    float horiz = 0.0f;

    if ((0 != AFWD_CORRECTION) && !g_imu.b_calibrating)
    {
        // a_fwd in mg = mm/s^2 x 1000 / 9807.
        fwd -= (float) ((g_imu.afwd_mms2 * MG_PER_G) / G_MMS2);
    }

    horiz = s4_math_sqrt((left * left) + (up * up));
    g_imu.pitch_raw_cdeg = s4_math_atan2_cdeg(fwd, horiz);
    // Low-pass (1/4 per cycle, about 70 ms): driving vibration made the
    // raw 20 ms pitch jump +/-5 deg (2026-11-01 drive test).
    if (g_imu.b_calibrating)
    {
        g_imu.pitch_cdeg = g_imu.pitch_raw_cdeg - g_imu.pitch0_cdeg;
        g_imu.roll_cdeg  = s4_math_atan2_cdeg(left, up) - g_imu.roll0_cdeg;
    }
    else
    {
        g_imu.pitch_cdeg += ((g_imu.pitch_raw_cdeg - g_imu.pitch0_cdeg)
                             - g_imu.pitch_cdeg) / ANGLE_LP_DIV;
        g_imu.roll_cdeg  += ((s4_math_atan2_cdeg(left, up) - g_imu.roll0_cdeg)
                             - g_imu.roll_cdeg) / ANGLE_LP_DIV;
    }
}

/*!
 * @brief Start (or restart) the calibration.
 */
static void
imu_cal_begin (void)
{
    g_imu.b_calibrating = true;
    g_imu.b_cal_done    = false;
    g_imu.cal_cycles    = 0u;
    g_imu.cal_samples   = 0u;
    g_imu.cal_sum[0]    = 0;
    g_imu.cal_sum[1]    = 0;
    g_imu.cal_sum[2]    = 0;
    g_imu.pitch0_cdeg   = 0;
    g_imu.roll0_cdeg    = 0;

    tm_printf((UB *) "S4 CAL start: keep the robot still and level for 2 s\n");
}

/*!
 * @brief One calibration cycle: check the robot is still, and finish
 *        after 2 s.
 */
static void
imu_cal_step (void)
{
    int32_t            mean[AXES] = {0};
    int32_t            now[AXES]  = {0};
    int32_t            spread     = 0;
    subsystem2_odom_t  odom       = {0};
    uint32_t           idx        = 0u;

    subsystem2_odom_get(&odom);

    now[0] = g_imu.mean.fwd;
    now[1] = g_imu.mean.left;
    now[2] = g_imu.mean.up;
    for (idx = 0u; idx < AXES; idx++)
    {
        if ((0u == g_imu.cal_cycles) || (now[idx] < g_imu.cal_min[idx]))
        {
            g_imu.cal_min[idx] = now[idx];
        }
        if ((0u == g_imu.cal_cycles) || (now[idx] > g_imu.cal_max[idx]))
        {
            g_imu.cal_max[idx] = now[idx];
        }
        if ((g_imu.cal_max[idx] - g_imu.cal_min[idx]) > spread)
        {
            spread = g_imu.cal_max[idx] - g_imu.cal_min[idx];
        }
    }

    if ((spread > CAL_SPREAD_MG)
        || (s4_math_abs(odom.speed_mms) > CAL_STILL_MMS))
    {
        tm_printf((UB *) "S4 CAL robot moved (spread %d mg, speed %d mm/s),"
                  " starting again\n", (INT) spread, (INT) odom.speed_mms);
        imu_cal_begin();
        return;
    }

    g_imu.cal_cycles++;
    if ((g_imu.cal_cycles < CAL_CYCLES) || (0u == g_imu.cal_samples))
    {
        return;
    }

    for (idx = 0u; idx < AXES; idx++)
    {
        // NOTE: The cast is safe: about 800 samples.
        mean[idx] = g_imu.cal_sum[idx] / (int32_t) g_imu.cal_samples;
    }

    imu_orientation(mean);

    {
        float fwd  = (float) axis_value(mean, g_imu.fwd_idx, g_imu.fwd_sign);
        float left = (float) axis_value(mean, g_imu.left_idx,
                                        g_imu.left_sign);
        float up   = (float) axis_value(mean, g_imu.up_idx, g_imu.up_sign);

        g_imu.pitch0_cdeg = s4_math_atan2_cdeg(fwd,
                                s4_math_sqrt((left * left) + (up * up)));
        g_imu.roll0_cdeg  = s4_math_atan2_cdeg(left, up);
    }

    g_imu.b_calibrating = false;
    g_imu.b_cal_done    = true;
    g_imu.pitch_cdeg    = 0;    // Start the low-pass level, not from the
    g_imu.roll_cdeg     = 0;    // pre-calibration angle (false hump).
    g_imu.b_side        = false;
    g_imu.side_hold     = 0u;
    g_imu.hump          = S4_HUMP_FLAT;
    g_imu.h_mm          = 0.0f;
    g_imu.hold          = 0u;

    tm_printf((UB *) "S4 CAL done: g=(%d,%d,%d) mg, up=%c%c fwd=%c%c,"
              " pitch0=%d roll0=%d cdeg\n", (INT) mean[0], (INT) mean[1],
              (INT) mean[2], (g_imu.up_sign > 0) ? '+' : '-',
              (INT) ('x' + (INT) g_imu.up_idx),
              (g_imu.fwd_sign > 0) ? '+' : '-',
              (INT) ('x' + (INT) g_imu.fwd_idx), (INT) g_imu.pitch0_cdeg,
              (INT) g_imu.roll0_cdeg);
}

/*!
 * @brief Find the "up" axis from the mean gravity; left = up x forward.
 *
 * @param[in] p_mean Mean of each sensor axis while still, mg.
 */
static void
imu_orientation (int32_t const * p_mean)
{
    uint32_t up  = 0u;
    uint32_t idx = 0u;
    uint32_t k   = 0u;
    int32_t  eps = 0;

    for (idx = 1u; idx < AXES; idx++)
    {
        if (s4_math_abs(p_mean[idx]) > s4_math_abs(p_mean[up]))
        {
            up = idx;
        }
    }

    if (up == g_imu.fwd_idx)
    {
        tm_printf((UB *) "S4 CAL WARNING: the forward axis points up; fix"
                  " IMU_FWD_AXIS in imu_task.c\n");
        return;
    }

    g_imu.up_idx  = up;
    g_imu.up_sign = (p_mean[up] >= 0) ? 1 : -1;

    // The third axis; (up, fwd, k) cyclic means +1 (right-handed).
    k   = (AXES - ((up + g_imu.fwd_idx) % AXES)) % AXES;
    eps = (((up + 1u) % AXES) == g_imu.fwd_idx) ? 1 : -1;

    g_imu.left_idx  = k;
    g_imu.left_sign = g_imu.up_sign * g_imu.fwd_sign * eps;
}

/*!
 * @brief Hump state machine and height (Week 6 report 6.5).
 *
 * @param[in] p_odom This cycle's odometry.
 */
static void
imu_hump (subsystem2_odom_t const * p_odom)
{
    int32_t ds_mm = p_odom->dist_mm - g_imu.prev_dist_mm;
    int32_t pitch = g_imu.pitch_cdeg;
    int32_t peak  = 0;
    int32_t l_sin = 0;
    int32_t signed_pitch = 0;

    g_imu.h_mm += s4_math_sin_cdeg(pitch) * (float) ds_mm;

    imu_side_hump(p_odom);

    // A lift by hand, or a climb that never comes down, is dropped after
    // 2 s level and still, without counting a hump.
    if ((S4_HUMP_FLAT != g_imu.hump)
        && (s4_math_abs(pitch) < HUMP_FLAT_CDEG)
        && (s4_math_abs(p_odom->speed_mms) < HUMP_STILL_MMS))
    {
        g_imu.idle_cycles++;
        if (g_imu.idle_cycles >= HUMP_ABANDON_CYCLES)
        {
            tm_printf((UB *) "S4 HUMP abandoned: level and still for 2 s\n");
            g_imu.hump = S4_HUMP_FLAT;
            g_imu.hold = 0u;
            g_imu.h_mm = 0.0f;
        }
    }
    else
    {
        g_imu.idle_cycles = 0u;
    }

    // Sign-aware: which way the nose tips first depends on whether the
    // wheels or the ball caster reach the hump first, so the first tilt
    // past 5 deg in either direction starts the hump, and the hump ends
    // when the robot is level again (with or without an opposite tilt).
    // 2026-11-01, 2 cm strips: nose up, then down, peak 28 mm.
    signed_pitch = pitch * g_imu.hump_sign;

    switch (g_imu.hump)
    {
        case S4_HUMP_FLAT:
        {
            if (s4_math_abs(pitch) > HUMP_ENTER_CDEG)
            {
                g_imu.hold++;
                if (g_imu.hold >= HUMP_HOLD_CYCLES)
                {
                    g_imu.hump           = S4_HUMP_CLIMBING;
                    g_imu.hold           = 0u;
                    g_imu.hump_sign      = (pitch > 0) ? 1 : -1;
                    g_imu.hump_peak_mm   = 0.0f;
                    g_imu.theta_max_cdeg = s4_math_abs(pitch);
                    tm_printf((UB *) "S4 HUMP climbing (nose %s)\n",
                              (pitch > 0) ? "up" : "down");
                }
            }
            else
            {
                g_imu.hold = 0u;
                if (s4_math_abs(pitch) < HUMP_FLAT_CDEG)
                {
                    g_imu.h_mm = 0.0f;
                }
            }
        }
        break;

        case S4_HUMP_CLIMBING:
        case S4_HUMP_DESCENDING:
        default:
        {
            if (s4_math_abs(pitch) > g_imu.theta_max_cdeg)
            {
                g_imu.theta_max_cdeg = s4_math_abs(pitch);
            }
            if (s4_math_abs((int32_t) g_imu.h_mm)
                > (int32_t) g_imu.hump_peak_mm)
            {
                // NOTE: The cast is safe: a hump is far below 2^31 mm.
                g_imu.hump_peak_mm = (float) s4_math_abs((int32_t) g_imu.h_mm);
            }

            if ((S4_HUMP_CLIMBING == g_imu.hump)
                && (signed_pitch < -HUMP_ENTER_CDEG))
            {
                g_imu.hump = S4_HUMP_DESCENDING;
                tm_printf((UB *) "S4 HUMP descending\n");
            }

            if (s4_math_abs(pitch) < HUMP_FLAT_CDEG)
            {
                g_imu.hold++;
                if (g_imu.hold >= HUMP_HOLD_CYCLES)
                {
                    l_sin = (int32_t) (((float) WHEELBASE_MM
                            * s4_math_sin_cdeg(g_imu.theta_max_cdeg))
                            + ROUND_HALF);

                    // With the wheelbase known, L.sin(theta_max) is the
                    // estimate (short hump, A17); else the integrated h.
                    peak = (WHEELBASE_MM > 0) ? l_sin
                           : (int32_t) (g_imu.hump_peak_mm + ROUND_HALF);
                    g_imu.hump_count++;
                    if (peak > g_imu.run_peak_mm)
                    {
                        g_imu.run_peak_mm = peak;
                    }

                    tm_printf((UB *) "S4 HUMP #%d peak=%d mm theta_max=%d"
                              " cdeg L.sin=%d mm integrated=%d mm (run peak"
                              " %d mm)\n", (INT) g_imu.hump_count, (INT) peak,
                              (INT) g_imu.theta_max_cdeg, (INT) l_sin,
                              (INT) (g_imu.hump_peak_mm + ROUND_HALF),
                              (INT) g_imu.run_peak_mm);

                    g_imu.hump = S4_HUMP_FLAT;
                    g_imu.hold = 0u;
                    g_imu.h_mm = 0.0f;
                }
            }
            else
            {
                g_imu.hold = 0u;
            }
        }
        break;
    }
}

/*!
 * @brief One-wheel hump: one wheel rides over the hump, so the robot
 *        rolls instead of pitching (2026-11-01 drive test: roll -19 deg
 *        over ~140 mm, pitch stayed near 0). Height = track x
 *        sin(largest |roll|). Reported like a pitch hump.
 *
 * @param[in] p_odom This cycle's odometry.
 */
static void
imu_side_hump (subsystem2_odom_t const * p_odom)
{
    int32_t roll   = s4_math_abs(g_imu.roll_cdeg);
    int32_t height = 0;

    if (!g_imu.b_side)
    {
        g_imu.side_hold = (roll > HUMP_ENTER_CDEG) ? (g_imu.side_hold + 1u)
                                                   : 0u;
        if (g_imu.side_hold >= HUMP_HOLD_CYCLES)
        {
            g_imu.b_side             = true;
            g_imu.side_hold          = 0u;
            g_imu.side_roll_max_cdeg = roll;
            g_imu.side_start_mm      = p_odom->dist_mm;
            tm_printf((UB *) "S4 HUMP one wheel up (%s side)\n",
                      (g_imu.roll_cdeg > 0) ? "left" : "right");
        }
    }
    else
    {
        if (roll > g_imu.side_roll_max_cdeg)
        {
            g_imu.side_roll_max_cdeg = roll;
        }

        g_imu.side_hold = (roll < HUMP_FLAT_CDEG) ? (g_imu.side_hold + 1u)
                                                  : 0u;
        if (g_imu.side_hold >= HUMP_HOLD_CYCLES)
        {
            // NOTE: The cast is safe: |height| <= 112 mm.
            height = (int32_t) (((float) S2_TRACK_MM
                                 * s4_math_sin_cdeg(g_imu.side_roll_max_cdeg))
                                + ROUND_HALF);
            g_imu.hump_count++;
            if (height > g_imu.run_peak_mm)
            {
                g_imu.run_peak_mm = height;
            }

            tm_printf((UB *) "S4 HUMP #%d (one wheel) peak=%d mm roll_max=%d"
                      " cdeg over %d mm (run peak %d mm)\n",
                      (INT) g_imu.hump_count, (INT) height,
                      (INT) g_imu.side_roll_max_cdeg,
                      (INT) (p_odom->dist_mm - g_imu.side_start_mm),
                      (INT) g_imu.run_peak_mm);

            g_imu.b_side    = false;
            g_imu.side_hold = 0u;
        }
    }
}

/*!
 * @brief Motion class; a new class must hold for 5 cycles (100 ms)
 *        before it is reported, except IMPACT, reported at once.
 *
 * @param[in] p_odom This cycle's odometry.
 */
static void
imu_class (subsystem2_odom_t const * p_odom)
{
    subsystem4_class_t cand = S4_CLASS_MOVING;

    if (g_imu.b_impact && !subsystem2_estop_is_set())
    {
        g_imu.b_impact = false;
        tm_printf((UB *) "S4 IMPACT cleared (MF_ESTOP cleared)\n");
    }

    if (g_imu.b_impact)
    {
        cand = S4_CLASS_IMPACT;
    }
    else if (S4_HUMP_CLIMBING == g_imu.hump)
    {
        cand = S4_CLASS_CLIMBING_HUMP;
    }
    else if (S4_HUMP_DESCENDING == g_imu.hump)
    {
        cand = S4_CLASS_DESCENDING_HUMP;
    }
    else if (s4_math_abs(p_odom->speed_mms) < CLASS_STILL_MMS)
    {
        cand = S4_CLASS_STATIONARY;
    }
    else if (s4_math_abs(g_imu.afwd_mms2) > CLASS_ACCEL_MMS2)
    {
        cand = S4_CLASS_ACCELERATING;
    }
    else if (s4_math_abs(g_imu.turn_cdps) > CLASS_TURN_CDPS)
    {
        cand = S4_CLASS_TURNING;
    }
    else
    {
        cand = S4_CLASS_MOVING;
    }

    if (cand == g_imu.cls_pending)
    {
        g_imu.cls_count++;
    }
    else
    {
        g_imu.cls_pending = cand;
        g_imu.cls_count   = 1u;
    }

    if ((cand != g_imu.cls) && ((S4_CLASS_IMPACT == cand)
                                || (g_imu.cls_count >= CLASS_HOLD_CYCLES)))
    {
        g_imu.cls = cand;
        tm_printf((UB *) "S4 CLASS %s t=%d ms\n", class_name(cand),
                  (INT) (now_us() / 1000u));
    }
}

/*!
 * @brief After an impact, report how long after the sample S2 braked.
 */
static void
imu_impact_follow_up (void)
{
    subsystem2_status_t s2 = {0};

    if (g_imu.latency_wait > 0u)
    {
        g_imu.latency_wait--;
        subsystem2_status_get(&s2);

        if (s2.estop_brake_us != g_imu.brake_us_before)
        {
            tm_printf((UB *) "S4 IMPACT braked %d us after the sample\n",
                      (INT) (s2.estop_brake_us - g_imu.impact_us));
            g_imu.latency_wait = 0u;
        }
        else if (0u == g_imu.latency_wait)
        {
            tm_printf((UB *) "S4 IMPACT no brake time from S2\n");
        }
        else
        {
            // Keep looking.
        }
    }
}

/*!
 * @brief IMU_DEGRADED: stale data, warning bit for S5, retry every 1 s.
 *
 * @param[in] p_why Reason, for the log.
 */
static void
imu_degrade (char const * p_why)
{
    if (g_imu.b_imu_ok)
    {
        g_imu.b_imu_ok    = false;
        g_imu.down_cycles = 0u;
        subsystem5_fault_set(S5_FAULT_IMU);
        tm_printf((UB *) "S4 FAULT %s -> IMU_DEGRADED (S5_FAULT_IMU)\n",
                  p_why);
    }
}

/*!
 * @brief Store the terrain (packed, under the mutex).
 */
static void
imu_publish (void)
{
    subsystem4_terrain_t t = {0};

    // NOTE: The casts are safe: s4_terrain_pack clamps every field.
    t.pitch_hdeg    = (int16_t) s4_math_clamp(g_imu.pitch_cdeg / CDEG_PER_HDEG,
                                              INT16_MIN, INT16_MAX);
    t.roll_hdeg     = (int16_t) s4_math_clamp(g_imu.roll_cdeg / CDEG_PER_HDEG,
                                              INT16_MIN, INT16_MAX);
    t.turn_rate_dps = (int16_t) s4_math_clamp(g_imu.turn_cdps / CDPS_PER_DPS,
                                              INT16_MIN, INT16_MAX);
    t.peak_mm       = (uint8_t) s4_math_clamp(g_imu.run_peak_mm, 0,
                                              S4_PEAK_MM_MAX);
    t.hump_state    = (uint8_t) g_imu.hump;
    t.motion_class  = (uint8_t) g_imu.cls;
    t.hump_count    = (uint8_t) s4_math_clamp((int32_t) g_imu.hump_count, 0,
                                              S4_HUMP_COUNT_MAX);
    t.b_imu_ok      = g_imu.b_imu_ok;
    t.b_cal_done    = g_imu.b_cal_done;
    t.b_impact      = g_imu.b_impact;

    subsystem4_terrain_put(&t);
}

/*!
 * @brief Run time and lateness of this cycle (S4 + S1 timing test).
 */
static void
imu_timing (uint32_t start_us, uint32_t late_us)
{
    uint32_t run = now_us() - start_us;

    g_imu.status.cycles++;
    if (run > g_imu.status.run_max_us)
    {
        g_imu.status.run_max_us = run;
    }
    if (late_us > g_imu.status.late_max_us)
    {
        g_imu.status.late_max_us = late_us;
    }
    if (run > g_imu.run_period_max_us)
    {
        g_imu.run_period_max_us = run;
    }
    if (late_us > g_imu.late_period_max_us)
    {
        g_imu.late_period_max_us = late_us;
    }

    subsystem4_status_put(&g_imu.status);
}

/*!
 * @brief One status line (every 200 ms).
 *
 * @par Fields
 * p, r: pitch, roll (1/100 deg); f, l, u: acceleration forward, left, up
 * (mg); af: drive acceleration (mm/s^2); h: hump height now (1/10 mm);
 * pk: run peak (mm); hs: hump state; cl: class; w: turn rate (1/100
 * deg/s); n: samples this cycle; dxy: largest impact signal (mg); run,
 * late: worst cycle time and lateness in this 200 ms (us); ok, cal.
 */
static void
imu_log (void)
{
    // NOTE: The casts are safe: every value is far inside INT's range.
    tm_printf((UB *) "S4 p=%d r=%d f=%d l=%d u=%d af=%d h=%d pk=%d hs=%d"
              " cl=%d w=%d n=%d dxy=%d run=%d late=%d ok=%d cal=%d\n",
              (INT) g_imu.pitch_cdeg, (INT) g_imu.roll_cdeg,
              (INT) g_imu.mean.fwd, (INT) g_imu.mean.left,
              (INT) g_imu.mean.up, (INT) g_imu.afwd_mms2,
              (INT) (g_imu.h_mm * TENTHS), (INT) g_imu.run_peak_mm,
              (INT) g_imu.hump, (INT) g_imu.cls, (INT) g_imu.turn_cdps,
              (INT) g_imu.last_samples,
              (INT) s4_math_sqrt((float) g_imu.dxy2_max),
              (INT) g_imu.run_period_max_us, (INT) g_imu.late_period_max_us,
              g_imu.b_imu_ok ? 1 : 0, g_imu.b_cal_done ? 1 : 0);

    g_imu.dxy2_max           = 0;
    g_imu.run_period_max_us  = 0u;
    g_imu.late_period_max_us = 0u;
}

/*!
 * @brief Sensor axes -> robot axes (forward, left, up).
 */
static body_t
to_body (drv_lsm303_accel_t const * p_s)
{
    int32_t v[AXES] = {0};
    body_t  b       = {0};

    v[0] = p_s->x_mg;
    v[1] = p_s->y_mg;
    v[2] = p_s->z_mg;

    b.fwd  = axis_value(v, g_imu.fwd_idx, g_imu.fwd_sign);
    b.left = axis_value(v, g_imu.left_idx, g_imu.left_sign);
    b.up   = axis_value(v, g_imu.up_idx, g_imu.up_sign);

    return (b);
}

/*!
 * @brief One component of a vector, with a sign.
 */
static int32_t
axis_value (int32_t const * p_v, uint32_t idx, int32_t sign)
{
    return ((sign >= 0) ? p_v[idx % AXES] : -p_v[idx % AXES]);
}

/*!
 * @brief The RP2040's free-running 1 MHz timer (low 32 bits).
 */
static uint32_t
now_us (void)
{
    return (in_w(TIMER_RAWL_REG));
}

/*!
 * @brief Name of a motion class, for the log.
 */
static char const *
class_name (subsystem4_class_t cls)
{
    static char const * const names[] =
    {
        "STATIONARY", "MOVING", "ACCELERATING", "TURNING",
        "CLIMBING_HUMP", "DESCENDING_HUMP", "IMPACT"
    };
    uint32_t idx = (uint32_t) cls;

    return ((idx < (sizeof(names) / sizeof(names[0]))) ? names[idx] : "?");
}

/*** end of file ***/
