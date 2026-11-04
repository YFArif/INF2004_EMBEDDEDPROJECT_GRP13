/** @file s3_ir_zone.h
 * @brief Diagnostic zones of a calibrated IR reading, not material labels.
 */
#ifndef S3_IR_ZONE_H
#define S3_IR_ZONE_H

#include <stdint.h>

typedef enum
{
    S3_IR_ZONE_INVALID = 0,
    S3_IR_ZONE_WHITE,
    S3_IR_ZONE_GREY,
    S3_IR_ZONE_BLACK
} s3_ir_zone_t;

static inline s3_ir_zone_t
s3_ir_zone (int32_t norm)
{
    if ((norm < 0) || (norm > 1000))
    {
        return S3_IR_ZONE_INVALID;
    }
    if (norm < 250)
    {
        return S3_IR_ZONE_WHITE;
    }
    return (norm > 750) ? S3_IR_ZONE_BLACK : S3_IR_ZONE_GREY;
}

#endif
