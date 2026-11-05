/** @file subsystem4_test.c
 *
 * @brief On-target tests for subsystem 4 (make TEST=1). Results go in
 *        testing/subsystem4/TEST_LOG.md.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "s4_terrain.h"
#include "subsystem4.h"

#if APP_TEST

#define WAIT_STEP_MS    (100)
#define WAIT_STEPS      (100u)      /* 10 s for imu_task's start-up.     */

void
subsystem4_test (void)
{
    subsystem4_status_t  status = {0};
    subsystem4_terrain_t in     = {0};
    subsystem4_terrain_t out    = {0};
    s4_terrain_packed_t  packed = {0u, 0u};
    uint32_t             idx    = 0u;

    TEST_INIT();

    for (idx = 0u; (idx < WAIT_STEPS) && !status.b_started; idx++)
    {
        (void) tk_dly_tsk(WAIT_STEP_MS);
        subsystem4_status_get(&status);
    }

    TEST_CHECK("S4-U06", status.b_acc_ok,
               "accelerometer CTRL1/CTRL4 read back (0x19)");
    TEST_CHECK("S4-U07", status.b_mag_ok,
               "magnetometer identity H43 (0x1E)");

    // Minimum-bit packing: clamp, never wrap; round trip exact.
    in.pitch_hdeg    = 200;
    in.roll_hdeg     = -200;
    in.turn_rate_dps = 600;
    in.peak_mm       = 255u;
    s4_terrain_pack(&in, &packed);
    s4_terrain_unpack(&packed, &out);
    TEST_CHECK("S4-U08", (63 == out.pitch_hdeg) && (-64 == out.roll_hdeg)
               && (511 == out.turn_rate_dps) && (255u == out.peak_mm),
               "packed fields clamp at their limits");

    in.pitch_hdeg    = -13;
    in.roll_hdeg     = 7;
    in.turn_rate_dps = -600;
    in.peak_mm       = 42u;
    in.hump_state    = (uint8_t) S4_HUMP_DESCENDING;
    in.motion_class  = (uint8_t) S4_CLASS_TURNING;
    in.hump_count    = 3u;
    in.b_imu_ok      = true;
    in.b_cal_done    = true;
    in.b_impact      = false;
    s4_terrain_pack(&in, &packed);
    s4_terrain_unpack(&packed, &out);
    TEST_CHECK("S4-U09", (-13 == out.pitch_hdeg) && (7 == out.roll_hdeg)
               && (-512 == out.turn_rate_dps) && (42u == out.peak_mm)
               && (in.hump_state == out.hump_state)
               && (in.motion_class == out.motion_class)
               && (3u == out.hump_count) && out.b_imu_ok && out.b_cal_done
               && !out.b_impact,
               "packed terrain round trip");

    TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */

/*** end of file ***/
