/** @file s3_analog.c
 *
 * @brief Subsystem 3 -- analogue barcode widths. See s3_analog.h.
 */

#include <stddef.h>
#include <stdint.h>

#include "s3_analog.h"

#define IDX_MAX         (65535u)    /* Extremum index is 16-bit.         */
#define PCT             (100u)
#define FRAC_ONE        (1000)      /* Interpolation in 1/1000 sample.   */

/** One peak or valley. */
typedef struct
{
    uint16_t idx;           /* Sample index (window <= 65535 samples).   */
    uint8_t  val;           /* 8-bit sample.                             */
} extremum_t;

/* Static: keeps the task stack small. Single caller (task_barcode). */
static extremum_t g_ext[S3_AN_MAX_EXTREMA];
static uint32_t   g_cross_us[S3_AN_MAX_EXTREMA];

static uint32_t find_extrema(uint8_t const * p_x, uint32_t count,
                             uint32_t prom);
static uint32_t merge_fragments(uint32_t * p_dur, uint32_t count);
static uint32_t find_crossings(uint8_t const * p_x, uint32_t n_ext,
                               uint32_t dt_us);

/*!
 * @brief Element durations from one analogue window. See s3_analog.h.
 *
 * @return Number of durations written (29 for one clean *X* frame).
 */
uint32_t
s3_analog_widths (uint8_t const * p_x, uint32_t count, uint32_t dt_us,
                  uint32_t * p_dur, uint32_t max_dur, s3_an_stats_t * p_stats)
{
    uint8_t  low   = 0xFFu;
    uint8_t  high  = 0u;
    uint32_t prom;
    uint32_t n_ext;
    uint32_t n_cross;
    uint32_t n_out = 0u;
    uint32_t idx;

    if ((NULL == p_x) || (NULL == p_dur) || (count < 3u))
    {
        return 0u;
    }
    if (count > IDX_MAX)
    {
        count = IDX_MAX;
    }

    for (idx = 0u; idx < count; idx++)
    {
        low  = (p_x[idx] < low) ? p_x[idx] : low;
        high = (p_x[idx] > high) ? p_x[idx] : high;
    }
    /* The casts are safe: high and low are 8-bit samples with high >= low, so
     * the range is 0 ... 255, and the prominence (a percentage of it) is at
     * most 255, which fits uint8_t.
     */
    prom = ((uint32_t) (high - low) * S3_AN_PROM_PCT) / PCT;

    /* Thick bars set hi - lo, so a % of it hides blurred 3 mm bars (car
     * 30 Oct: only the 6 thick bars were found). Use the noise on the
     * white lead-in instead, never stricter than the % rule. */
    {
        uint8_t  lead_lo = 0xFFu;
        uint8_t  lead_hi = 0u;
        uint32_t noise_prom;
        uint32_t n_lead  = (count < S3_AN_LEAD_SAMPLES) ? count :
                                                         S3_AN_LEAD_SAMPLES;

        for (idx = 0u; idx < n_lead; idx++)
        {
            lead_lo = (p_x[idx] < lead_lo) ? p_x[idx] : lead_lo;
            lead_hi = (p_x[idx] > lead_hi) ? p_x[idx] : lead_hi;
        }
        /* The cast is safe: lead_hi >= lead_lo, both 8 bits, so the range is 0
         * ... 255.
         */
        noise_prom = (uint32_t) (lead_hi - lead_lo) * S3_AN_NOISE_X;
        prom       = (noise_prom < prom) ? noise_prom : prom;
    }
    prom = (prom < S3_AN_MIN_PROM) ? S3_AN_MIN_PROM : prom;

    if (NULL != p_stats)
    {
        p_stats->min        = low;
        p_stats->max        = high;
        /* The cast is safe: prom is at most 255 here (a percentage of an 8-bit
         * range, or the 8-bit noise floor), which fits uint8_t.
         */
        p_stats->prominence = (uint8_t) prom;
        p_stats->extrema    = 0u;
    }
    /* The cast is safe: high >= low, both 8 bits. */
    if ((uint32_t) (high - low) < S3_AN_MIN_RANGE)
    {
        return 0u;
    }

    /* 1. Alternating peaks / valleys with hysteresis prom. */
    n_ext = find_extrema(p_x, count, prom);
    if (NULL != p_stats)
    {
        p_stats->extrema = (uint8_t) n_ext;
    }

    /* 2. Edge time = where the signal crosses halfway between neighbours. */
    n_cross = find_crossings(p_x, n_ext, dt_us);

    /* 3. Durations between consecutive edges. */
    for (idx = 0u; ((idx + 1u) < n_cross) && (n_out < max_dur); idx++)
    {
        p_dur[n_out] = g_cross_us[idx + 1u] - g_cross_us[idx];
        n_out++;
    }
    return merge_fragments(p_dur, n_out);
}

/*!
 * @brief Remove noise stripes: one much shorter than the median stripe
 *        (< median / S3_AN_FRAG_DIV) is joined with the stripes on both
 *        sides (same colour as each other), or dropped at either end.
 *        Relative, so it works at any speed and any narrow width (3 to
 *        6 mm codes). Car 30 Oct: 55-65 stripes, many 8-30 ms, for 29.
 *
 * @return New number of durations.
 */
static uint32_t
merge_fragments (uint32_t * p_dur, uint32_t count)
{
    uint32_t sorted[S3_AN_MAX_EXTREMA];
    uint32_t thr;
    uint32_t idx;
    uint32_t pos;
    uint32_t dst;

    if (count < 3u)
    {
        return count;
    }
    for (idx = 0u; idx < count; idx++)  /* Insertion sort for the median. */
    {
        uint32_t dur = p_dur[idx];

        for (pos = idx; (pos > 0u) && (sorted[pos - 1u] > dur); pos--)
        {
            sorted[pos] = sorted[pos - 1u];
        }
        sorted[pos] = dur;
    }
    thr = sorted[count / 2u] / S3_AN_FRAG_DIV;

    idx = 0u;
    while (idx < count)
    {
        if (p_dur[idx] >= thr)
        {
            idx++;
        }
        else if ((0u == idx) || ((idx + 1u) == count))
        {
            for (dst = idx; (dst + 1u) < count; dst++)  /* At an end: drop. */
            {
                p_dur[dst] = p_dur[dst + 1u];
            }
            count--;
            idx = (0u == idx) ? 0u : count;
        }
        else
        {
            /* prev + frag + next */
            p_dur[idx - 1u] += p_dur[idx] + p_dur[idx + 1u];
            for (dst = idx; (dst + 2u) < count; dst++)
            {
                p_dur[dst] = p_dur[dst + 2u];
            }
            count -= 2u;

            /* The joined stripe may still be next to another fragment. */
            idx = idx - 1u;
        }
    }
    return count;
}

/*!
 * @brief Alternating peaks and valleys that stand out by at least prom.
 *
 * NOTE: The casts in this function are safe: idx is below IDX_MAX, which
 * fits uint16_t, and each 8-bit difference cast to uint32_t is either
 * known to be positive or tested for it in the same condition.
 *
 * @return Number of extrema stored in g_ext.
 */
static uint32_t
find_extrema (uint8_t const * p_x, uint32_t count, uint32_t prom)
{
    uint32_t   n_ext = 0u;
    int32_t    dir   = 0;       /* +1 looking for a peak, -1 a valley.  */
    extremum_t cand_hi;
    extremum_t cand_lo;
    uint32_t   idx;

    cand_hi.idx = 0u;
    cand_hi.val = p_x[0];
    cand_lo     = cand_hi;

    for (idx = 1u; (idx < count) && (n_ext < S3_AN_MAX_EXTREMA); idx++)
    {
        uint8_t value = p_x[idx];

        if (value > cand_hi.val)
        {
            cand_hi.idx = (uint16_t) idx;
            cand_hi.val = value;
        }
        if (value < cand_lo.val)
        {
            cand_lo.idx = (uint16_t) idx;
            cand_lo.val = value;
        }

        if (0 == dir)
        {
            if (((uint32_t) (value - cand_lo.val) >= prom) &&
                (value > cand_lo.val))
            {
                g_ext[n_ext] = cand_lo;         /* Started on white.     */
                n_ext++;
                dir          = 1;
                cand_hi.idx  = (uint16_t) idx;
                cand_hi.val  = value;
            }
            else if ((cand_hi.val > value) &&
                     ((uint32_t) (cand_hi.val - value) >= prom))
            {
                g_ext[n_ext] = cand_hi;         /* Started on black.     */
                n_ext++;
                dir          = -1;
                cand_lo.idx  = (uint16_t) idx;
                cand_lo.val  = value;
            }
            else
            {
                ;   /* Not yet a clear start. */
            }
        }
        else if ((1 == dir) && (cand_hi.val > value) &&
                 ((uint32_t) (cand_hi.val - value) >= prom))
        {
            g_ext[n_ext] = cand_hi;             /* Confirmed peak.       */
            n_ext++;
            dir          = -1;
            cand_lo.idx  = (uint16_t) idx;
            cand_lo.val  = value;
        }
        else if ((-1 == dir) && (value > cand_lo.val) &&
                 ((uint32_t) (value - cand_lo.val) >= prom))
        {
            g_ext[n_ext] = cand_lo;             /* Confirmed valley.     */
            n_ext++;
            dir          = 1;
            cand_hi.idx  = (uint16_t) idx;
            cand_hi.val  = value;
        }
        else
        {
            ;   /* Still climbing or falling. */
        }
    }

    /* The last pending extreme counts if the signal settled there. */
    if ((n_ext > 0u) && (n_ext < S3_AN_MAX_EXTREMA))
    {
        if ((1 == dir) &&
            ((uint32_t) (cand_hi.val - g_ext[n_ext - 1u].val) >= prom))
        {
            g_ext[n_ext] = cand_hi;
            n_ext++;
        }
        else if ((-1 == dir) &&
                 ((uint32_t) (g_ext[n_ext - 1u].val - cand_lo.val) >= prom))
        {
            g_ext[n_ext] = cand_lo;
            n_ext++;
        }
        else
        {
            ;   /* Nothing pending. */
        }
    }
    return n_ext;
}

/*!
 * @brief Time of each stripe edge: where the signal crosses halfway
 *        between two neighbouring extrema, interpolated inside the step.
 *
 * @return Number of edge times stored in g_cross_us.
 */
static uint32_t
find_crossings (uint8_t const * p_x, uint32_t n_ext, uint32_t dt_us)
{
    uint32_t n_cross = 0u;
    uint32_t idx;
    uint32_t sample;

    for (idx = 0u; (idx + 1u) < n_ext; idx++)
    {
        uint32_t first    = g_ext[idx].idx;
        uint32_t last     = g_ext[idx + 1u].idx;
        /* The casts are safe: the samples are 8 bits, which int32_t holds, so
         * the sums and differences cannot overflow.
         */
        int32_t  mid      = ((int32_t) g_ext[idx].val +
                             (int32_t) g_ext[idx + 1u].val) / 2;
        int32_t  b_rising = (g_ext[idx + 1u].val > g_ext[idx].val) ? 1 : 0;

        for (sample = first + 1u; sample <= last; sample++)
        {
            /* The casts are safe: 8-bit samples always fit int32_t. */
            int32_t x_prev = (int32_t) p_x[sample - 1u];
            int32_t x_now  = (int32_t) p_x[sample];

            if (((0 != b_rising) && (x_now >= mid)) ||
                ((0 == b_rising) && (x_now <= mid)))
            {
                int32_t frac = (x_now != x_prev) ?
                               (((mid - x_prev) * FRAC_ONE) /
                                (x_now - x_prev)) :
                               FRAC_ONE;

                /* (j - 1) * dt + frac * dt / 1000: no 32-bit overflow. */
                g_cross_us[n_cross] = ((sample - 1u) * dt_us) +
                                      (((uint32_t) frac * dt_us) /
                                       (uint32_t) FRAC_ONE);
                n_cross++;
                break;
            }
        }
    }
    return n_cross;
}

/*!
 * @brief Digital path (car 30 Oct): DOUT flicks white for a few ms inside
 *        a black bar (tb 46, nw 9, tb 54), adding 2 stripes. Join any
 *        WHITE stripe shorter than the median white / S3_AN_FRAG_DIV with
 *        the bars around it. Black stripes are never joined: narrow bars
 *        really do read as short DOUT pulses (6-25 ms). Stripe 0 = black.
 *
 * @return New number of durations.
 */
uint32_t
s3_merge_gaps (uint32_t * p_dur, uint32_t count)
{
    uint32_t sorted[S3_AN_MAX_EXTREMA];
    uint32_t n_w = 0u;
    uint32_t thr;
    uint32_t idx;
    uint32_t pos;
    uint32_t dst;

    if ((count < 3u) || (count > S3_AN_MAX_EXTREMA))
    {
        return count;
    }
    for (idx = 1u; idx < count; idx += 2u)  /* Insertion sort of whites. */
    {
        uint32_t dur = p_dur[idx];

        for (pos = n_w; (pos > 0u) && (sorted[pos - 1u] > dur); pos--)
        {
            sorted[pos] = sorted[pos - 1u];
        }
        sorted[pos] = dur;
        n_w++;
    }
    thr = sorted[n_w / 2u] / S3_AN_FRAG_DIV;

    for (idx = 1u; (idx + 1u) < count; )
    {
        if (p_dur[idx] < thr)
        {
            p_dur[idx - 1u] += p_dur[idx] + p_dur[idx + 1u]; /* bar+gap+bar */
            for (dst = idx; (dst + 2u) < count; dst++)
            {
                p_dur[dst] = p_dur[dst + 2u];
            }
            count -= 2u;
        }
        else
        {
            idx += 2u;
        }
    }
    return count;
}

/*** end of file ***/

