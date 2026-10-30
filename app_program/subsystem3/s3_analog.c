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

static uint32_t find_extrema(uint8_t const * p_x, uint32_t n, uint32_t prom);
static uint32_t merge_fragments(uint32_t * p_dur, uint32_t n);
static uint32_t find_crossings(uint8_t const * p_x, uint32_t n_ext,
                               uint32_t dt_us);

/*!
 * @brief Element durations from one analogue window. See s3_analog.h.
 *
 * @return Number of durations written (29 for one clean *X* frame).
 */
uint32_t
s3_analog_widths (uint8_t const * p_x, uint32_t n, uint32_t dt_us,
                  uint32_t * p_dur, uint32_t max_dur, s3_an_stats_t * p_stats)
{
    uint8_t  lo = 0xFFu;
    uint8_t  hi = 0u;
    uint32_t prom;
    uint32_t n_ext;
    uint32_t n_cross;
    uint32_t n_out = 0u;
    uint32_t i;

    if ((NULL == p_x) || (NULL == p_dur) || (n < 3u))
    {
        return 0u;
    }
    if (n > IDX_MAX)
    {
        n = IDX_MAX;
    }

    for (i = 0u; i < n; i++)
    {
        lo = (p_x[i] < lo) ? p_x[i] : lo;
        hi = (p_x[i] > hi) ? p_x[i] : hi;
    }
    prom = ((uint32_t) (hi - lo) * S3_AN_PROM_PCT) / PCT;

    /* Thick bars set hi - lo, so a % of it hides blurred 3 mm bars (car
     * 30 Oct: only the 6 thick bars were found). Use the noise on the
     * white lead-in instead, never stricter than the % rule. */
    {
        uint8_t  lead_lo = 0xFFu;
        uint8_t  lead_hi = 0u;
        uint32_t noise_prom;
        uint32_t n_lead = (n < S3_AN_LEAD_SAMPLES) ? n : S3_AN_LEAD_SAMPLES;

        for (i = 0u; i < n_lead; i++)
        {
            lead_lo = (p_x[i] < lead_lo) ? p_x[i] : lead_lo;
            lead_hi = (p_x[i] > lead_hi) ? p_x[i] : lead_hi;
        }
        noise_prom = (uint32_t) (lead_hi - lead_lo) * S3_AN_NOISE_X;
        prom       = (noise_prom < prom) ? noise_prom : prom;
    }
    prom = (prom < S3_AN_MIN_PROM) ? S3_AN_MIN_PROM : prom;

    if (NULL != p_stats)
    {
        p_stats->min        = lo;
        p_stats->max        = hi;
        p_stats->prominence = (uint8_t) prom;
        p_stats->extrema    = 0u;
    }
    if ((uint32_t) (hi - lo) < S3_AN_MIN_RANGE)
    {
        return 0u;
    }

    /* 1. Alternating peaks / valleys with hysteresis prom. */
    n_ext = find_extrema(p_x, n, prom);
    if (NULL != p_stats)
    {
        p_stats->extrema = (uint8_t) n_ext;
    }

    /* 2. Edge time = where the signal crosses halfway between neighbours. */
    n_cross = find_crossings(p_x, n_ext, dt_us);

    /* 3. Durations between consecutive edges. */
    for (i = 0u; ((i + 1u) < n_cross) && (n_out < max_dur); i++)
    {
        p_dur[n_out] = g_cross_us[i + 1u] - g_cross_us[i];
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
merge_fragments (uint32_t * p_dur, uint32_t n)
{
    uint32_t sorted[S3_AN_MAX_EXTREMA];
    uint32_t thr;
    uint32_t i;
    uint32_t j;
    uint32_t k;

    if (n < 3u)
    {
        return n;
    }
    for (i = 0u; i < n; i++)            /* Insertion sort for the median. */
    {
        uint32_t v = p_dur[i];

        for (j = i; (j > 0u) && (sorted[j - 1u] > v); j--)
        {
            sorted[j] = sorted[j - 1u];
        }
        sorted[j] = v;
    }
    thr = sorted[n / 2u] / S3_AN_FRAG_DIV;

    i = 0u;
    while (i < n)
    {
        if (p_dur[i] >= thr)
        {
            i++;
        }
        else if ((0u == i) || ((i + 1u) == n))
        {
            for (k = i; (k + 1u) < n; k++)  /* At an end: drop it.        */
            {
                p_dur[k] = p_dur[k + 1u];
            }
            n--;
            i = (0u == i) ? 0u : n;
        }
        else
        {
            p_dur[i - 1u] += p_dur[i] + p_dur[i + 1u];  /* prev+frag+next */
            for (k = i; (k + 2u) < n; k++)
            {
                p_dur[k] = p_dur[k + 2u];
            }
            n -= 2u;
            i = i - 1u;                 /* The joined stripe may still be */
        }                               /* next to another fragment.      */
    }
    return n;
}

/*!
 * @brief Alternating peaks and valleys that stand out by at least prom.
 *
 * @return Number of extrema stored in g_ext.
 */
static uint32_t
find_extrema (uint8_t const * p_x, uint32_t n, uint32_t prom)
{
    uint32_t   n_ext = 0u;
    int32_t    dir   = 0;       /* +1 looking for a peak, -1 a valley.  */
    extremum_t cand_hi;
    extremum_t cand_lo;
    uint32_t   i;

    cand_hi.idx = 0u;
    cand_hi.val = p_x[0];
    cand_lo     = cand_hi;

    for (i = 1u; (i < n) && (n_ext < S3_AN_MAX_EXTREMA); i++)
    {
        uint8_t value = p_x[i];

        if (value > cand_hi.val)
        {
            cand_hi.idx = (uint16_t) i;
            cand_hi.val = value;
        }
        if (value < cand_lo.val)
        {
            cand_lo.idx = (uint16_t) i;
            cand_lo.val = value;
        }

        if (0 == dir)
        {
            if (((uint32_t) (value - cand_lo.val) >= prom) &&
                (value > cand_lo.val))
            {
                g_ext[n_ext] = cand_lo;         /* Started on white.     */
                n_ext++;
                dir         = 1;
                cand_hi.idx = (uint16_t) i;
                cand_hi.val = value;
            }
            else if ((cand_hi.val > value) &&
                     ((uint32_t) (cand_hi.val - value) >= prom))
            {
                g_ext[n_ext] = cand_hi;         /* Started on black.     */
                n_ext++;
                dir         = -1;
                cand_lo.idx = (uint16_t) i;
                cand_lo.val = value;
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
            dir         = -1;
            cand_lo.idx = (uint16_t) i;
            cand_lo.val = value;
        }
        else if ((-1 == dir) && (value > cand_lo.val) &&
                 ((uint32_t) (value - cand_lo.val) >= prom))
        {
            g_ext[n_ext] = cand_lo;             /* Confirmed valley.     */
            n_ext++;
            dir         = 1;
            cand_hi.idx = (uint16_t) i;
            cand_hi.val = value;
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
    uint32_t i;
    uint32_t j;

    for (i = 0u; (i + 1u) < n_ext; i++)
    {
        uint32_t a        = g_ext[i].idx;
        uint32_t b        = g_ext[i + 1u].idx;
        int32_t  mid      = ((int32_t) g_ext[i].val +
                             (int32_t) g_ext[i + 1u].val) / 2;
        int32_t  b_rising = (g_ext[i + 1u].val > g_ext[i].val) ? 1 : 0;

        for (j = a + 1u; j <= b; j++)
        {
            int32_t x0 = (int32_t) p_x[j - 1u];
            int32_t x1 = (int32_t) p_x[j];

            if (((0 != b_rising) && (x1 >= mid)) ||
                ((0 == b_rising) && (x1 <= mid)))
            {
                int32_t frac = (x1 != x0) ?
                               (((mid - x0) * FRAC_ONE) / (x1 - x0)) :
                               FRAC_ONE;

                /* (j - 1) * dt + frac * dt / 1000: no 32-bit overflow. */
                g_cross_us[n_cross] = ((j - 1u) * dt_us) +
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
s3_merge_gaps (uint32_t * p_dur, uint32_t n)
{
    uint32_t sorted[S3_AN_MAX_EXTREMA];
    uint32_t n_w = 0u;
    uint32_t thr;
    uint32_t i;
    uint32_t j;
    uint32_t k;

    if ((n < 3u) || (n > S3_AN_MAX_EXTREMA))
    {
        return n;
    }
    for (i = 1u; i < n; i += 2u)        /* Insertion sort of the whites.  */
    {
        uint32_t v = p_dur[i];

        for (j = n_w; (j > 0u) && (sorted[j - 1u] > v); j--)
        {
            sorted[j] = sorted[j - 1u];
        }
        sorted[j] = v;
        n_w++;
    }
    thr = sorted[n_w / 2u] / S3_AN_FRAG_DIV;

    for (i = 1u; (i + 1u) < n; )
    {
        if (p_dur[i] < thr)
        {
            p_dur[i - 1u] += p_dur[i] + p_dur[i + 1u];  /* bar+gap+bar   */
            for (k = i; (k + 2u) < n; k++)
            {
                p_dur[k] = p_dur[k + 2u];
            }
            n -= 2u;
        }
        else
        {
            i += 2u;
        }
    }
    return n;
}

/*** end of file ***/
