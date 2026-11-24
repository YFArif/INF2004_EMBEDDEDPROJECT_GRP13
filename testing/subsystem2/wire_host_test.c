#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem2/s2_wire.h"

static unsigned int g_passed;
static unsigned int g_failed;

static void
check (bool condition, char const * p_name)
{
    if (condition)
    {
        g_passed++;
        (void) printf("[S2-WIRE] PASS %s\n", p_name);
    }
    else
    {
        g_failed++;
        (void) printf("[S2-WIRE] FAIL %s\n", p_name);
    }
}

int
main (void)
{
    static int16_t const values[] =
    {
        INT16_MIN, -720, -180, -90, -1, 0, 1, 90, 180, 720, INT16_MAX
    };
    static uint16_t const speeds[] =
    {
        0u, 40u, 50u, 150u, 300u, 500u, 501u, UINT16_MAX
    };
    uint32_t     packed = 0u;
    uint8_t      type = 0u;
    int16_t      value = 0;
    uint16_t     speed = 0u;
    unsigned int type_idx;
    unsigned int value_idx;
    unsigned int speed_idx;

    for (type_idx = 0u; type_idx <= S2_WIRE_TYPE_MAX; type_idx++)
    {
        for (value_idx = 0u;
             value_idx < (sizeof(values) / sizeof(values[0]));
             value_idx++)
        {
            for (speed_idx = 0u;
                 speed_idx < (sizeof(speeds) / sizeof(speeds[0]));
                 speed_idx++)
            {
                packed = s2_wire_pack((uint8_t) type_idx, values[value_idx],
                                      speeds[speed_idx]);
                s2_wire_unpack(packed, &type, &value, &speed);
                check((type == type_idx) &&
                      (value == values[value_idx]) &&
                      (speed == ((speeds[speed_idx] > S2_WIRE_SPEED_MAX) ?
                                 S2_WIRE_SPEED_MAX : speeds[speed_idx])),
                      "command round-trip");
            }
        }
    }

    packed = s2_wire_pack(UINT8_MAX, INT16_MIN, UINT16_MAX);
    s2_wire_unpack(packed, &type, &value, &speed);
    check((7u == type) && (INT16_MIN == value) &&
          (S2_WIRE_SPEED_MAX == speed),
          "invalid input saturates safely");
    check(4u == S2_WIRE_BYTES, "message-buffer payload is four bytes");

    (void) printf("S2 WIRE: %u passed, %u failed\n", g_passed, g_failed);
    return (0u == g_failed) ? 0 : 1;
}
