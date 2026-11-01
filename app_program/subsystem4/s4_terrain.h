/** @file s4_terrain.h
 *
 * @brief Minimum-bit storage of the terrain data (Tribunal 1 curveball).
 *
 * @par Layout (43 bits in 6 bytes)
 * word0, 32 bits:
 *   bits  0- 6  pitch, 1/2 degree, 7-bit signed  (offset 64)
 *   bits  7-13  roll,  1/2 degree, 7-bit signed  (offset 64)
 *   bits 14-23  turn rate, degree/s, 10-bit signed (offset 512)
 *   bits 24-31  peak hump height, mm, 8 bits
 * word1, 16 bits:
 *   bits  0- 1  hump state       bits  2- 4  motion class
 *   bit   5     imu_ok           bit   6     cal_done
 *   bits  7- 9  hump count (saturates at 7)
 *   bit  10     impact latched
 *
 * Built with masks and shifts, never with bit-fields of signed types, so
 * the layout does not depend on the compiler. Each value is clamped to
 * its range before it is packed, so it can never wrap. imu_task is the
 * only writer; readers copy the 6 bytes under the mutex, then unpack.
 */

#ifndef S4_TERRAIN_H
#define S4_TERRAIN_H

#include <stdint.h>

#include "subsystem4.h"

typedef struct
{
    uint32_t word0;
    uint16_t word1;
} s4_terrain_packed_t;

#define S4_PACKED_BITS          (43u)
#define S4_HDEG_MIN             (-64)
#define S4_HDEG_MAX             (63)
#define S4_TURN_DPS_MIN         (-512)
#define S4_TURN_DPS_MAX         (511)
#define S4_PEAK_MM_MAX          (255)
#define S4_HUMP_COUNT_MAX       (7)

void s4_terrain_pack(subsystem4_terrain_t const * p_in,
                     s4_terrain_packed_t * p_out);
void s4_terrain_unpack(s4_terrain_packed_t const * p_in,
                       subsystem4_terrain_t * p_out);

#endif /* S4_TERRAIN_H */

/*** end of file ***/
