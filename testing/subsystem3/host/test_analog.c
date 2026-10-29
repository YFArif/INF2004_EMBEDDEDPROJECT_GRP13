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
        x[n++] = (uint8_t)(((uint32_t)(200.0 + g_amp * refl(p, sigma))) >> 4);   /* 12-bit ADC -> 8-bit */
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

    {   /* flat white paper: no edges at all */
        static uint8_t w[4000]; uint32_t i, d[80];
        for (i = 0U; i < 4000U; i++) { w[i] = (uint8_t)((200U + (i % 7U)) >> 4); }
        i = s3_analog_widths(w, 4000U, 500U, d, 80U, 0);
        printf("[TEST] %-34s %s  (elements=%u)\n", "plain white -> no edges", (0U == i) ? "PASS" : "FAIL", (unsigned)i);
        g_fail += (0U != i);
    }

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
