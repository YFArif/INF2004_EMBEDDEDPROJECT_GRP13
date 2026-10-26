/*
 *	s3_analog.c -- see s3_analog.h
 */
#include "s3_analog.h"

typedef struct
{
    uint16_t idx;          /* sample index, window <= 16384 samples */
    uint8_t  val;          /* 8-bit sample                          */
} extremum_t;

uint32_t
s3_analog_widths (uint8_t const * p_x, uint32_t n, uint32_t dt_us,
                  uint32_t * p_dur, uint32_t max_dur, s3_an_stats_t * p_stats)
{
    static extremum_t ext[S3_AN_MAX_EXTREMA];      /* static: keeps the task stack small (single caller) */
    uint32_t   n_ext = 0U;
    static uint32_t   cross_us[S3_AN_MAX_EXTREMA];
    uint32_t   n_cross = 0U;
    uint8_t    lo = 0xFFU;
    uint8_t    hi = 0U;
    uint32_t   prom;
    uint32_t   i;
    int32_t    dir = 0;                 /* +1 looking for a peak, -1 for a valley */
    extremum_t cand_hi;
    extremum_t cand_lo;
    uint32_t   n_out = 0U;

    if ((0 == p_x) || (0 == p_dur) || (n < 3U))
    {
        return 0U;
    }
    if (n > 65535U)
    {
        n = 65535U;                               /* extremum index is 16-bit */
    }

    for (i = 0U; i < n; i++)
    {
        lo = (p_x[i] < lo) ? p_x[i] : lo;
        hi = (p_x[i] > hi) ? p_x[i] : hi;
    }

    prom = ((uint32_t)(hi - lo) * S3_AN_PROM_PCT) / 100U;
    prom = (prom < S3_AN_MIN_PROM) ? S3_AN_MIN_PROM : prom;

    if (0 != p_stats)
    {
        p_stats->min        = lo;
        p_stats->max        = hi;
        p_stats->prominence = (uint8_t)prom;
        p_stats->extrema    = 0U;
    }

    if ((uint32_t)(hi - lo) < S3_AN_MIN_RANGE)
    {
        return 0U;
    }

    /* 1. Alternating peaks/valleys with hysteresis 'prom'. */
    cand_hi.idx = 0U; cand_hi.val = p_x[0];
    cand_lo = cand_hi;

    for (i = 1U; (i < n) && (n_ext < S3_AN_MAX_EXTREMA); i++)
    {
        uint8_t v = p_x[i];

        if (v > cand_hi.val) { cand_hi.idx = (uint16_t)i; cand_hi.val = v; }
        if (v < cand_lo.val) { cand_lo.idx = (uint16_t)i; cand_lo.val = v; }

        if (0 == dir)
        {
            if ((uint32_t)(v - cand_lo.val) >= prom && (v > cand_lo.val))
            {
                ext[n_ext++] = cand_lo;           /* started on white */
                dir = 1; cand_hi.idx = (uint16_t)i; cand_hi.val = v;
            }
            else if ((cand_hi.val > v) && ((uint32_t)(cand_hi.val - v) >= prom))
            {
                ext[n_ext++] = cand_hi;           /* started on black */
                dir = -1; cand_lo.idx = (uint16_t)i; cand_lo.val = v;
            }
        }
        else if ((1 == dir) && (cand_hi.val > v) && ((uint32_t)(cand_hi.val - v) >= prom))
        {
            ext[n_ext++] = cand_hi;               /* confirmed peak */
            dir = -1; cand_lo.idx = (uint16_t)i; cand_lo.val = v;
        }
        else if ((-1 == dir) && (v > cand_lo.val) && ((uint32_t)(v - cand_lo.val) >= prom))
        {
            ext[n_ext++] = cand_lo;               /* confirmed valley */
            dir = 1; cand_hi.idx = (uint16_t)i; cand_hi.val = v;
        }
    }

    /* the last pending extreme counts if the signal settled there */
    if ((n_ext > 0U) && (n_ext < S3_AN_MAX_EXTREMA))
    {
        if (1 == dir)  { if ((uint32_t)(cand_hi.val - ext[n_ext - 1U].val) >= prom) { ext[n_ext++] = cand_hi; } }
        if (-1 == dir) { if ((uint32_t)(ext[n_ext - 1U].val - cand_lo.val) >= prom) { ext[n_ext++] = cand_lo; } }
    }

    if (0 != p_stats)
    {
        p_stats->extrema = (uint8_t)n_ext;
    }

    /* 2. Edge time = where the signal crosses halfway between neighbours. */
    for (i = 0U; (i + 1U) < n_ext; i++)
    {
        uint32_t a   = ext[i].idx;
        uint32_t b   = ext[i + 1U].idx;
        int32_t  mid = ((int32_t)ext[i].val + (int32_t)ext[i + 1U].val) / 2;
        int32_t  rising = (ext[i + 1U].val > ext[i].val);
        uint32_t j;

        for (j = a + 1U; j <= b; j++)
        {
            int32_t x0 = (int32_t)p_x[j - 1U];
            int32_t x1 = (int32_t)p_x[j];

            if ((rising && (x1 >= mid)) || ((!rising) && (x1 <= mid)))
            {
                /* linear interpolation inside the sample step, in 1/1000 sample */
                int32_t frac = (x1 != x0) ? (((mid - x0) * 1000) / (x1 - x0)) : 1000;
                /* (j-1)*dt + frac*dt/1000: no 32-bit overflow for long windows */
                cross_us[n_cross++] = ((j - 1U) * dt_us) + (((uint32_t)frac * dt_us) / 1000U);
                break;
            }
        }
    }

    /* 3. Durations between consecutive edges. */
    for (i = 0U; ((i + 1U) < n_cross) && (n_out < max_dur); i++)
    {
        p_dur[n_out++] = cross_us[i + 1U] - cross_us[i];
    }

    return n_out;
}
