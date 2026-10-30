/*
 *	test_analog.c -- host (laptop) unit test for s3_analog.c + s3_code39.c
 *
 *	  gcc -Wall -Wextra -I../../../app_program/subsystem3 test_analog.c \
 *	      ../../../app_program/subsystem3/s3_analog.c \
 *	      ../../../app_program/subsystem3/s3_code39.c -lm -o tan && ./tan
 *
 *	Builds a simulated sensor signal: the real *A* / *Z* stripe pattern
 *	(3 mm / 9 mm), blurred by a Gaussian sensor spot of sigma mm, mapped
 *	to ADC counts (white 200, black 3800), sampled every 0.5 ms.
 */
#include <stdio.h>
#include <math.h>
#include "s3_code39.h"
#include "s3_analog.h"

static int g_el[29];
static int g_fail = 0;

static void build (char const * p_bits3)   /* 27 chars: '*' + X + '*' patterns */
{
    int k = 0;
    int i;
    for (i = 0; i < 27; i++)
    {
        g_el[k++] = (p_bits3[i] == '1') ? 3 : 1;
        if ((i == 8) || (i == 17)) { g_el[k++] = 1; }
    }
}

static int black_at (double x)
{
    double p = 0.0;
    int i;
    for (i = 0; i < 29; i++) { double w = g_el[i] * 3.0; if ((x >= p) && (x < p + w)) { return (i % 2) == 0; } p += w; }
    return 0;
}

static double refl (double x, double s)
{
    double sum = 0.0, ws = 0.0, d;
    for (d = -3.0 * s; d <= 3.0 * s; d += 0.05) { double w = exp(-d * d / (2.0 * s * s)); sum += w * black_at(x + d); ws += w; }
    return sum / ws;
}

static double g_amp = 3600.0;        /* white-to-black swing in ADC counts */
static double   g_noise = 0.0;       /* +- ADC counts of uniform noise on each sample */
static uint32_t g_seed  = 12345U;
static int      g_spikes = 0;        /* 1 = a 2 ms full-swing noise spike every 350 ms */
static double g_max_gap_ms;          /* longest time with no 200-count step (bring-up trigger) */

static void run (char const * p_id, char const * p_bits, double sigma, double speed_mm_s,
                 int reversed, char want, int want_ok)
{
    static uint8_t x[20000];      /* 8-bit samples, as stored on the robot */
    uint32_t n = 0U, m;
    uint32_t dur[80];
    double   pos;
    char     ch = '?';
    uint8_t  rev = 0U;
    s3_c39_result_t res;
    s3_an_stats_t   st;
    int ok;

    build(p_bits);
    for (pos = -20.0; (pos < 161.0) && (n < 20000U); pos += speed_mm_s * 0.5e-3)
    {
        double p = reversed ? (141.0 - pos) : pos;
        g_seed = (g_seed * 1103515245U) + 12345U;
        x[n++] = (uint8_t)(((uint32_t)(200.0 + g_amp * refl(p, sigma) +
                                       g_noise * ((double)((g_seed >> 16) % 2001U) / 1000.0 - 1.0))) >> 4);   /* 12-bit ADC -> 8-bit */
    }
    {   /* replay the s3_bringup.c activity trigger: step of 200 counts from last reference */
        uint32_t i, last = 0U, first = 0U, gap = 0U, seen = 0U; uint8_t ref = x[0];
        for (i = 1U; i < n; i++) {
            int d = (int)x[i] - (int)ref;
            if ((d > 6) || (d < -6))     /* DRV_BARCODE_ACT_STEP */ { ref = x[i]; if (seen && ((i - last) > gap)) { gap = i - last; } if (!seen) { first = i; } seen = 1U; last = i; }
        }
        (void)first;
        g_max_gap_ms = seen ? (gap * 0.5) : -1.0;
    }
    if (0 != g_spikes)
    {
        uint32_t i;
        for (i = 100U; i < n; i++) { if ((i % 700U) < 4U) { x[i] = (x[i] > 128U) ? 12U : 200U; } }
    }
    m   = s3_analog_widths(x, n, 500U, dur, 80U, &st);
    res = s3_code39_decode(dur, m, &ch, &rev);
    ok  = want_ok ? ((S3_C39_OK == res) && (ch == want)) : (S3_C39_OK != res);
    {   /* every duration must be shorter than the whole window (catches 32-bit overflow) */
        uint32_t k;
        for (k = 0U; k < m; k++) { if (dur[k] > (n * 500U)) { ok = 0; } }
    }
    printf("[TEST] %-34s %s  (elements=%u extrema=%u res=%d ch=%c, trigger gap max %.0f ms)\n", p_id, ok ? "PASS" : "FAIL",
           (unsigned)m, (unsigned)st.extrema, (int)res, ch, g_max_gap_ms);
    g_fail += !ok;
}

int main (void)
{
    static char const k_a[] = "010010100" "100001001" "010010100";
    static char const k_z[] = "010010100" "011010000" "010010100";

    run("sharp *A* at 0.30 m/s",            k_a, 0.3, 300.0, 0, 'A', 1);
    run("blurred 1.5 mm *A* at 0.30 m/s",    k_a, 1.5, 300.0, 0, 'A', 1);
    run("blurred 1.8 mm *Z* at 0.15 m/s",    k_z, 1.8, 150.0, 0, 'Z', 1);
    run("blurred 1.5 mm *A* reversed",       k_a, 1.5, 300.0, 1, 'A', 1);
    run("blurred 1.5 mm *A* at 0.40 m/s",    k_a, 1.5, 400.0, 0, 'A', 1);
    run("too blurred (3 mm) -> reject",      k_a, 3.0, 300.0, 0, 'A', 0);

    g_amp = 1500.0;                  /* weak contrast, like the 4 mm desk sweep */
    run("hand 25 mm/s, swing 1500, blur 1.5", k_a, 1.5, 25.0, 0, 'A', 1);
    run("hand 50 mm/s, swing 1500, blur 1.8", k_z, 1.8, 50.0, 1, 'Z', 1);
    g_amp = 3600.0;

    /* Car 30 Oct: 3 mm bars blurred so much that a % of the thick-bar
     * swing hid them (only the 6 thick bars found). Noise +-16 counts (car white: 160-176). */
    g_noise = 16.0;
    run("car: blur 2.2 mm, noise, 50 mm/s",  k_a, 2.2, 50.0, 0, 'A', 1);
    run("car: blur 2.4 mm -> reject, not wrong", k_z, 2.4, 40.0, 1, 'Z', 0);
    run("car: blur 2.6 mm -> reject, not wrong", k_a, 2.6, 80.0, 0, 'A', 0);
    g_noise = 0.0;

    /* Car 30 Oct: 55-65 stripes for 29, many 8-30 ms noise fragments. */
    g_spikes = 1;
    run("noise spikes inside stripes, 50 mm/s", k_a, 1.5, 50.0, 0, 'A', 1);
    run("noise spikes, reversed *Z*, 40 mm/s",   k_z, 1.5, 40.0, 1, 'Z', 1);
    g_spikes = 0;

    {   /* flat white paper: no edges at all */
        static uint8_t w[4000]; uint32_t i, d[80];
        for (i = 0U; i < 4000U; i++) { w[i] = (uint8_t)((200U + (i % 7U)) >> 4); }
        i = s3_analog_widths(w, 4000U, 500U, d, 80U, 0);
        printf("[TEST] %-34s %s  (elements=%u)\n", "plain white -> no edges", (0U == i) ? "PASS" : "FAIL", (unsigned)i);
        g_fail += (0U != i);
    }

    {   /* Car 30 Oct 15:49, digital *A*: a 9 ms DOUT flicker split a bar (31
         * stripes, rejected); the good pass has a 6 ms narrow bar to keep. */
        static uint32_t const k_split[31] = {67,192,46,9,54,102,208,82,202,78,41,91,209,88,54,96,
                                             50,260,30,104,202,105,17,278,42,99,220,79,224,68,72};
        static uint32_t const k_good[29]  = {71,231,24,82,180,71,190,88,35,96,208,88,52,111,42,
                                             275,30,104,198,111,6,271,36,102,210,77,226,73,65};
        uint32_t d[31], m, k; char c = '?'; uint8_t r = 0U; int ok1, ok2;
        for (k = 0U; k < 31U; k++) { d[k] = k_split[k] * 1000U; }
        m   = s3_merge_gaps(d, 31U);
        ok1 = (S3_C39_OK == s3_code39_decode(d, m, &c, &r)) && ('A' == c);
        printf("[TEST] %-34s %s  (stripes 31 -> %u, ch=%c)\n", "car: DOUT flicker inside a bar", ok1 ? "PASS" : "FAIL", (unsigned)m, c);
        for (k = 0U; k < 29U; k++) { d[k] = k_good[k] * 1000U; }
        m   = s3_merge_gaps(d, 29U); c = '?';
        ok2 = (29U == m) && (S3_C39_OK == s3_code39_decode(d, m, &c, &r)) && ('A' == c);
        printf("[TEST] %-34s %s  (stripes 29 -> %u, ch=%c)\n", "car: 6 ms narrow bar kept", ok2 ? "PASS" : "FAIL", (unsigned)m, c);
        g_fail += !ok1; g_fail += !ok2;

        /* Pass began on black: a 40 ms white first, so the bars sit on odd
         * stripes. Merging from stripe 1 (task_barcode retry) keeps the 6 ms bar. */
        d[0] = 40000U;
        for (k = 0U; k < 29U; k++) { d[k + 1U] = k_good[k] * 1000U; }
        m   = s3_merge_gaps(&d[1], 29U) + 1U; c = '?';
        ok1 = (30U == m) && (S3_C39_OK == s3_code39_decode(d, m, &c, &r)) && ('A' == c);
        printf("[TEST] %-34s %s  (stripes 30 -> %u, ch=%c)\n", "car: pass starts on black", ok1 ? "PASS" : "FAIL", (unsigned)m, c);
        g_fail += !ok1;
    }

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
