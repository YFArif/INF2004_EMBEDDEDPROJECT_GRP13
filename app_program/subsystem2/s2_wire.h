/** @file s2_wire.h
 *
 * @brief Compact IF-04 motion-command encoding inside subsystem 2.
 *
 * Public S2 command, odometry and status types remain unchanged so peer
 * subsystems retain their established interface. Only the message-buffer
 * payload uses this 28-bit encoding.
 */

#ifndef S2_WIRE_H
#define S2_WIRE_H

#include <stdint.h>

#define S2_WIRE_TYPE_MAX        (4u)
#define S2_WIRE_TYPE_BITS       (3u)
#define S2_WIRE_SPEED_BITS      (9u)
#define S2_WIRE_VALUE_BITS      (16u)
#define S2_WIRE_SPEED_MAX       (500u)
#define S2_WIRE_TYPE_MASK       (0x07u)
#define S2_WIRE_SPEED_MASK      (0x1FFu)
#define S2_WIRE_VALUE_MASK      (0xFFFFu)
#define S2_WIRE_SPEED_POS       (S2_WIRE_TYPE_BITS)
#define S2_WIRE_VALUE_POS       (S2_WIRE_TYPE_BITS + S2_WIRE_SPEED_BITS)
#define S2_WIRE_BYTES           (4u)

_Static_assert(28u == (S2_WIRE_TYPE_BITS + S2_WIRE_SPEED_BITS +
                       S2_WIRE_VALUE_BITS),
               "S2 wire command allocation must total 28 bits");
_Static_assert(0u != (S2_WIRE_TYPE_MAX < (1u << S2_WIRE_TYPE_BITS)),
               "S2 command type does not fit its wire field");

static inline uint32_t
s2_wire_pack (uint8_t type, int16_t value, uint16_t speed_mms)
{
    uint32_t wire_type  = (type <= S2_WIRE_TYPE_MAX) ? type : 7u;
    uint32_t wire_speed = (speed_mms > S2_WIRE_SPEED_MAX) ?
                          S2_WIRE_SPEED_MAX : speed_mms;

    return (wire_type |
            (wire_speed << S2_WIRE_SPEED_POS) |
            (((uint32_t) (uint16_t) value) << S2_WIRE_VALUE_POS));
}

static inline void
s2_wire_unpack (uint32_t packed, uint8_t * p_type, int16_t * p_value,
                uint16_t * p_speed_mms)
{
    uint32_t raw_value = (packed >> S2_WIRE_VALUE_POS) &
                          S2_WIRE_VALUE_MASK;
    int32_t  value     = ((0u != (raw_value & 0x8000u)) ?
                          ((int32_t) raw_value - 65536) :
                          (int32_t) raw_value);
    uint32_t speed     = (packed >> S2_WIRE_SPEED_POS) & S2_WIRE_SPEED_MASK;

    *p_type = (uint8_t) (packed & S2_WIRE_TYPE_MASK);
    *p_value = (int16_t) value;
    *p_speed_mms = (uint16_t) ((speed > S2_WIRE_SPEED_MAX) ?
                               S2_WIRE_SPEED_MAX : speed);
}

#endif /* S2_WIRE_H */
