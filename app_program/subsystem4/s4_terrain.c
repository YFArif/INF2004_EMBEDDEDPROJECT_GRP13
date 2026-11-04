/** @file s4_terrain.c
 *
 * @brief Pack and unpack the terrain data (see s4_terrain.h).
 */

#include <stdbool.h>
#include <stdint.h>

#include "s4_math.h"
#include "s4_terrain.h"

#define PITCH_SHIFT         (0u)
#define ROLL_SHIFT          (7u)
#define TURN_SHIFT          (14u)
#define PEAK_SHIFT          (24u)
#define MASK_7              (0x7Fu)
#define MASK_8              (0xFFu)
#define MASK_10             (0x3FFu)

#define STATE_SHIFT         (0u)
#define CLASS_SHIFT         (2u)
#define OK_SHIFT            (5u)
#define CAL_SHIFT           (6u)
#define COUNT_SHIFT         (7u)
#define IMPACT_SHIFT        (10u)
#define MASK_1              (0x1u)
#define MASK_2              (0x3u)
#define MASK_3              (0x7u)

#define STATE_MAX           (2)
#define CLASS_MAX           (6)

static uint32_t to_offset(int32_t value, int32_t lo, int32_t hi);

/*!
 * @brief Clamp every field to its range and pack it.
 *
 * @param[in]  p_in  The terrain data.
 * @param[out] p_out The packed form.
 */
void
s4_terrain_pack (subsystem4_terrain_t const * p_in,
                 s4_terrain_packed_t * p_out)
{
    uint32_t w0 = 0u;
    uint32_t w1 = 0u;

    w0 |= (to_offset(p_in->pitch_hdeg, S4_HDEG_MIN, S4_HDEG_MAX) & MASK_7)
          << PITCH_SHIFT;
    w0 |= (to_offset(p_in->roll_hdeg, S4_HDEG_MIN, S4_HDEG_MAX) & MASK_7)
          << ROLL_SHIFT;
    w0 |= (to_offset(p_in->turn_rate_dps, S4_TURN_DPS_MIN, S4_TURN_DPS_MAX)
           & MASK_10) << TURN_SHIFT;
    w0 |= ((uint32_t) s4_math_clamp(p_in->peak_mm, 0, S4_PEAK_MM_MAX)
           & MASK_8) << PEAK_SHIFT;

    w1 |= ((uint32_t) s4_math_clamp(p_in->hump_state, 0, STATE_MAX)
           & MASK_2) << STATE_SHIFT;
    w1 |= ((uint32_t) s4_math_clamp(p_in->motion_class, 0, CLASS_MAX)
           & MASK_3) << CLASS_SHIFT;
    w1 |= (p_in->b_imu_ok ? MASK_1 : 0u) << OK_SHIFT;
    w1 |= (p_in->b_cal_done ? MASK_1 : 0u) << CAL_SHIFT;
    w1 |= ((uint32_t) s4_math_clamp(p_in->hump_count, 0, S4_HUMP_COUNT_MAX)
           & MASK_3) << COUNT_SHIFT;
    w1 |= (p_in->b_impact ? MASK_1 : 0u) << IMPACT_SHIFT;

    p_out->word0 = w0;

    // NOTE: The cast is safe: w1 uses bits 0-10 only.
    p_out->word1 = (uint16_t) w1;
}

/*!
 * @brief Unpack the terrain data.
 *
 * @param[in]  p_in  The packed form.
 * @param[out] p_out The terrain data.
 */
void
s4_terrain_unpack (s4_terrain_packed_t const * p_in,
                   subsystem4_terrain_t * p_out)
{
    uint32_t w0 = p_in->word0;
    uint32_t w1 = (uint32_t) p_in->word1;

    // NOTE: The casts are safe: each field is at most 10 bits wide, and
    // the offset brings it back into the signed range of its field.
    p_out->pitch_hdeg    = (int16_t) ((int32_t) ((w0 >> PITCH_SHIFT) & MASK_7)
                                      + S4_HDEG_MIN);
    p_out->roll_hdeg     = (int16_t) ((int32_t) ((w0 >> ROLL_SHIFT) & MASK_7)
                                      + S4_HDEG_MIN);
    p_out->turn_rate_dps = (int16_t) ((int32_t) ((w0 >> TURN_SHIFT) & MASK_10)
                                      + S4_TURN_DPS_MIN);
    p_out->peak_mm       = (uint8_t) ((w0 >> PEAK_SHIFT) & MASK_8);
    p_out->hump_state    = (uint8_t) ((w1 >> STATE_SHIFT) & MASK_2);
    p_out->motion_class  = (uint8_t) ((w1 >> CLASS_SHIFT) & MASK_3);
    p_out->b_imu_ok      = (0u != ((w1 >> OK_SHIFT) & MASK_1));
    p_out->b_cal_done    = (0u != ((w1 >> CAL_SHIFT) & MASK_1));
    p_out->hump_count    = (uint8_t) ((w1 >> COUNT_SHIFT) & MASK_3);
    p_out->b_impact      = (0u != ((w1 >> IMPACT_SHIFT) & MASK_1));
}

/*!
 * @brief Clamp a signed value to [lo, hi] and store it as value - lo.
 */
static uint32_t
to_offset (int32_t value, int32_t lo, int32_t hi)
{
    // NOTE: The cast is safe: the clamped value minus lo is >= 0.
    return ((uint32_t) (s4_math_clamp(value, lo, hi) - lo));
}

/*** end of file ***/
