/** @file s3_probe.h
 * @brief Single-sensor experiment, deterministic state machine.
 */
#ifndef S3_PROBE_H
#define S3_PROBE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_PROBE_SPEED_MMS (40u)
#define S3_PROBE_TURN_MMS (60u)
#define S3_PROBE_LOSS (400)
#define S3_PROBE_BLACK (650)
/* 130 mm sensor lead plus half an 18 mm crossing strip. */
#define S3_PROBE_ALIGN_MM (139)

typedef enum
{
    S3_PROBE_STOPPED = 0,
    S3_PROBE_FOLLOW,
    S3_PROBE_BRAKE,
    S3_PROBE_SEARCH,
    S3_PROBE_ALIGN,
    S3_PROBE_TURN,
    S3_PROBE_TURN_BRAKE,
    S3_PROBE_FAULT
} s3_probe_state_t;

typedef enum
{
    S3_PROBE_NONE = 0,
    S3_PROBE_STOP,
    S3_PROBE_DRIVE,
    S3_PROBE_ROTATE
} s3_probe_action_t;

typedef enum
{
    S3_PROBE_OK = 0,
    S3_PROBE_ESTOP,
    S3_PROBE_TIMEOUT,
    S3_PROBE_NOT_FOUND,
    S3_PROBE_ALIGN_LOST,
    S3_PROBE_DRIVE_LIMIT
} s3_probe_reason_t;

typedef struct
{
    uint32_t now_ms;
    uint32_t done;
    int32_t dist_mm;
    int32_t heading_cdeg;
    int32_t left;
    int32_t right;
    bool b_idle;
    bool b_estop;
    bool b_stop;
    bool b_arm;
} s3_probe_input_t;

typedef struct
{
    s3_probe_state_t state;
    s3_probe_reason_t reason;
    uint32_t since_ms;
    uint32_t done;
    int32_t mark_mm;
    int32_t start_mm;
    int32_t heading_cdeg;
    uint8_t search_step;
    uint8_t found;
    uint8_t clear;
    unsigned int armed : 1;
    unsigned int junctions : 1;
    unsigned int eligible : 1;
    unsigned int candidate : 1;
    unsigned int verify : 1;
} s3_probe_t;

typedef struct
{
    s3_probe_action_t action;
    int16_t angle_deg;
} s3_probe_output_t;

void s3_probe_start(s3_probe_t * p_probe, s3_probe_input_t const * p_in,
                    bool b_junctions);
void s3_probe_step(s3_probe_t * p_probe, s3_probe_input_t const * p_in,
                   s3_probe_output_t * p_out);

#endif
