/** @file test_probe.c
 * @brief Deterministic tests of the opt-in GP27 controller.
 */
#include <assert.h>
#include <stdio.h>
#include "s3_probe.h"
#include "s3_ir_zone.h"

static s3_probe_t g_probe;
static s3_probe_input_t g_in;
static s3_probe_output_t g_out;

static void reset(bool b_junctions);
static void step(uint32_t dt_ms);
static void crossing(void);

static void
reset (bool b_junctions)
{
    g_in = (s3_probe_input_t) {0};
    g_in.left = 800;
    g_in.b_idle = true;
    s3_probe_start(&g_probe, &g_in, b_junctions);
}

static void
step (uint32_t dt_ms)
{
    g_in.now_ms += dt_ms;
    s3_probe_step(&g_probe, &g_in, &g_out);
}

static void
crossing (void)
{
    g_in.b_arm = true;
    step(20u);
    g_in.b_arm = false;
    for (uint32_t i = 0u; i < 5u; i++)
    {
        step(20u);
    }
    g_in.right = 800;
    step(20u);
    g_in.dist_mm = 2;
    step(20u);
}

int
main (void)
{
    assert(S3_IR_ZONE_INVALID == s3_ir_zone(-1));
    assert(S3_IR_ZONE_INVALID == s3_ir_zone(1001));
    assert(S3_IR_ZONE_WHITE == s3_ir_zone(0));
    assert(S3_IR_ZONE_WHITE == s3_ir_zone(249));
    assert(S3_IR_ZONE_GREY == s3_ir_zone(250));
    assert(S3_IR_ZONE_GREY == s3_ir_zone(500));
    assert(S3_IR_ZONE_GREY == s3_ir_zone(750));
    assert(S3_IR_ZONE_BLACK == s3_ir_zone(751));
    assert(S3_IR_ZONE_BLACK == s3_ir_zone(1000));
    puts("PASS diagnostic WHITE/GREY/BLACK exact boundaries and invalid inputs");
    reset(true);
    g_in.right = 900;
    for (uint32_t i = 0u; i < 100u; i++)
    {
        g_in.dist_mm++;
        step(20u);
    }
    assert(S3_PROBE_FOLLOW == g_probe.state);
    puts("PASS unarmed both-black never turns");

    reset(false);
    crossing();
    assert(S3_PROBE_FOLLOW == g_probe.state);
    puts("PASS recovery-only image ignores arm");

    reset(true);
    g_in.right = 900;
    g_in.b_arm = true;
    step(20u);
    g_in.b_arm = false;
    g_in.dist_mm = 20;
    step(20u);
    assert(S3_PROBE_FOLLOW == g_probe.state);
    puts("PASS arming while already black requires white then black");

    reset(true);
    crossing();
    assert(S3_PROBE_ALIGN == g_probe.state);
    assert(0u == g_probe.armed);
    g_in.dist_mm = S3_PROBE_ALIGN_MM - 1;
    step(20u);
    assert(S3_PROBE_ALIGN == g_probe.state);
    g_in.dist_mm++;
    step(20u);
    assert(S3_PROBE_TURN_BRAKE == g_probe.state);
    assert(S3_PROBE_STOP == g_out.action);
    step(120u);
    assert(S3_PROBE_ROTATE == g_out.action);
    assert(90 == g_out.angle_deg);
    g_in.left = 0;
    step(20u);
    assert(S3_PROBE_TURN == g_probe.state);
    g_in.done++;
    step(20u);
    assert(S3_PROBE_BRAKE == g_probe.state);
    g_in.left = 800;
    step(120u);
    step(20u);
    step(20u);
    assert(S3_PROBE_FOLLOW == g_probe.state);
    assert(S3_PROBE_DRIVE == g_out.action);
    g_in.dist_mm += 200;
    step(20u);
    assert(S3_PROBE_FOLLOW == g_probe.state);
    assert(0u == g_probe.armed);
    puts("PASS 139mm alignment, brake, 90deg, reacquire, one-shot disarm");

    reset(false);
    g_in.left = 100;
    step(20u);
    assert(S3_PROBE_STOP == g_out.action);
    step(120u);
    assert(S3_PROBE_ROTATE == g_out.action);
    assert(4 == g_out.angle_deg);
    g_in.left = 800;
    step(20u);
    step(20u);
    step(20u);
    assert(S3_PROBE_STOP == g_out.action);
    step(120u);
    step(20u);
    step(20u);
    assert(S3_PROBE_DRIVE == g_out.action);
    puts("PASS GP27 loss brakes, small search, stable black resumes");

    reset(false);
    g_in.left = 0;
    step(20u);
    for (uint32_t i = 0u; i < 5u; i++)
    {
        step(120u);
        assert(S3_PROBE_ROTATE == g_out.action);
        g_in.done++;
        step(20u);
    }
    step(120u);
    assert(S3_PROBE_FAULT == g_probe.state);
    assert(S3_PROBE_NOT_FOUND == g_probe.reason);
    puts("PASS absent line exhausts five search legs and stops");

    reset(true);
    crossing();
    g_in.left = 0;
    step(20u);
    assert(S3_PROBE_ALIGN_LOST == g_probe.reason);
    assert(S3_PROBE_STOP == g_out.action);
    puts("PASS loss during alignment refuses blind turn");

    reset(false);
    g_in.left = 0;
    step(20u);
    step(8000u);
    assert(S3_PROBE_TIMEOUT == g_probe.reason);
    puts("PASS maneuver timeout");

    reset(false);
    g_in.left = 0;
    step(20u);
    step(120u);
    g_in.heading_cdeg = 1900;
    step(20u);
    assert(S3_PROBE_DRIVE_LIMIT == g_probe.reason);
    puts("PASS search angle bound");

    reset(true);
    g_in.b_estop = true;
    g_in.b_arm = true;
    step(20u);
    assert(S3_PROBE_ESTOP == g_probe.reason);
    assert(S3_PROBE_STOP == g_out.action);
    reset(true);
    g_in.b_stop = true;
    step(20u);
    assert(S3_PROBE_STOPPED == g_probe.state);
    puts("PASS E-stop and manual stop override arm");

    reset(false);
    step(3600000u);
    assert(S3_PROBE_FOLLOW == g_probe.state);
    puts("PASS no overall run timer");
    puts("ALL PROBE TESTS PASSED");
    return 0;
}
