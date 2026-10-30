/*
 *	test_code39_all.c -- host test: EVERY Code 39 character, both scan
 *	directions, at several print ratios, jitter levels and speed drifts.
 *
 *	Instructor requirement (28 Oct): decode A to Z, and decode the same
 *	letter whether the car scans left-to-right or right-to-left.
 *	test_nav.c S3-C01 already decodes every character both ways on ideal
 *	timings (ratio 3 and 2); this test adds print-ratio, jitter and speed
 *	drift, and checks the table against an independent reference.
 *
 *	Build and run on any PC/WSL with gcc, no Pico needed:
 *	  gcc -std=c99 -Wall -Wextra -I../../../app_program/subsystem3 \
 *	      test_code39_all.c ../../../app_program/subsystem3/s3_code39.c \
 *	      -o t39all && ./t39all
 *
 *	Reference patterns: python-barcode 0.15 (barcode/charsets/code39.py),
 *	converted to 9 bits, 1 = wide, first element = MSB. They are written
 *	here independently of s3_code39.c, so a typo in either table fails C03.
 */
#include <stdio.h>
#include <stdint.h>
#include "s3_code39.h"

#define N_DATA      (43U)           /* 0-9, A-Z, - . space $ / + %      */
#define TRIALS      (20U)           /* Random jitter draws per setting. */
#define UNIT        (1000U)         /* Narrow element, any time unit.   */

static char const      g_ref_ch[N_DATA + 1U] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
static uint16_t const  g_ref_pat[N_DATA] =
{
    0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124, 0x064,
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C,
    0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
    0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0, 0x085, 0x184, 0x0C4, 0x0A8,
    0x0A2, 0x08A, 0x02A
};
#define REF_STAR    (0x094U)

static int      g_fail = 0;
static uint32_t g_rand = 1U;

static void check (char const * p_id, int ok, char const * p_what)
{
    printf("[TEST] %-8s %s  %s\n", p_id, ok ? "PASS" : "FAIL", p_what);
    g_fail += !ok;
}

/* Small LCG: the same numbers on every PC, so a failure can be repeated. */
static uint32_t rand_u32 (void)
{
    g_rand = (g_rand * 1103515245U) + 12345U;
    return (g_rand >> 8);
}

/* *ch* as 29 durations. ratio_x10 = wide/narrow x 10. jitter_pct: each
 * element is scaled by a random factor in +-jitter_pct %. drift_pct: the
 * speed changes linearly by this much from the first to the last element
 * (negative = speeding up, so later elements are shorter). */
static void build (char ch, uint32_t ratio_x10, uint32_t jitter_pct,
                   int32_t drift_pct, uint32_t * p_dur)
{
    uint16_t pat[3];
    uint32_t n = 0U;
    uint32_t s;
    int32_t  i;

    (void)s3_code39_pattern('*', &pat[0]);
    (void)s3_code39_pattern(ch, &pat[1]);
    pat[2] = pat[0];
    for (s = 0U; s < 3U; s++)
    {
        for (i = 8; i >= 0; i--)
        {
            p_dur[n++] = (0U != ((pat[s] >> i) & 1U)) ? (UNIT * ratio_x10) / 10U : UNIT;
        }
        if (s < 2U)
        {
            p_dur[n++] = UNIT;                       /* narrow gap */
        }
    }
    for (n = 0U; n < S3_C39_ELEMENTS; n++)
    {
        int32_t jit   = (0U == jitter_pct) ? 0 :
                        ((int32_t)(rand_u32() % (2U * jitter_pct + 1U)) - (int32_t)jitter_pct);
        int32_t drift = (drift_pct * (int32_t)n) / (int32_t)(S3_C39_ELEMENTS - 1U);

        p_dur[n] = (uint32_t)(((int64_t)p_dur[n] * (100 + jit) * (100 + drift)) / 10000);
    }
}

int main (void)
{
    static uint32_t const ratios_x10[] = { 30U, 25U, 22U, 20U };
    static uint32_t const jitters[]    = { 0U, 10U, 15U };
    uint32_t dur[S3_C39_ELEMENTS];
    uint32_t rev_dur[S3_C39_ELEMENTS];
    uint32_t i, r, j, t, k;
    uint32_t bad_tab = 0U, fwd = 0U, bwd = 0U, wrong = 0U, total = 0U;
    uint32_t az = 0U;
    uint16_t pat;
    char     ch;
    char     letter;
    uint8_t  rev;
    char     line[96];

    /* C03: the decoder's table matches the independent reference. */
    for (i = 0U; i < N_DATA; i++)
    {
        if ((0 == s3_code39_pattern(g_ref_ch[i], &pat)) || (pat != g_ref_pat[i]))
        {
            printf("  table mismatch for '%c'\n", g_ref_ch[i]);
            bad_tab++;
        }
    }
    if ((0 == s3_code39_pattern('*', &pat)) || (REF_STAR != pat))
    {
        bad_tab++;
    }
    check("S3-C03", 0U == bad_tab, "all 44 table entries match python-barcode");

    /* C04: A-Z, ideal print, both directions. */
    for (letter = 'A'; letter <= 'Z'; letter++)
    {
        char    ch_b  = '?';
        uint8_t rev_b = 0U;

        build(letter, 30U, 0U, 0, dur);
        for (k = 0U; k < S3_C39_ELEMENTS; k++)
        {
            rev_dur[k] = dur[S3_C39_ELEMENTS - 1U - k];
        }
        if ((S3_C39_OK == s3_code39_decode(dur, S3_C39_ELEMENTS, &ch, &rev)) &&
            (letter == ch) && (0U == rev) &&
            (S3_C39_OK == s3_code39_decode(rev_dur, S3_C39_ELEMENTS, &ch_b, &rev_b)) &&
            (letter == ch_b) && (1U == rev_b))
        {
            az++;
        }
    }
    check("S3-C04", 26U == az, "A-Z decode forwards (rev 0) and backwards (rev 1)");

    /* C05: every character, both directions, ratio 2.0-3.0, jitter up to
     * +-15 %, speed drift -20 % / +25 %. 0 wrong letters is the hard rule
     * (NFR3: a wrong command is worse than none). */
    for (r = 0U; r < (sizeof(ratios_x10) / sizeof(ratios_x10[0])); r++)
    {
        for (j = 0U; j < (sizeof(jitters) / sizeof(jitters[0])); j++)
        {
            for (t = 0U; t < TRIALS; t++)
            {
                for (i = 0U; i < N_DATA; i++)
                {
                    build(g_ref_ch[i], ratios_x10[r], jitters[j], (0U != (t & 1U)) ? 25 : -20, dur);
                    for (k = 0U; k < S3_C39_ELEMENTS; k++)
                    {
                        rev_dur[k] = dur[S3_C39_ELEMENTS - 1U - k];
                    }
                    total++;
                    if (S3_C39_OK == s3_code39_decode(dur, S3_C39_ELEMENTS, &ch, &rev))
                    {
                        if ((g_ref_ch[i] == ch) && (0U == rev)) { fwd++; } else { wrong++; }
                    }
                    if (S3_C39_OK == s3_code39_decode(rev_dur, S3_C39_ELEMENTS, &ch, &rev))
                    {
                        if ((g_ref_ch[i] == ch) && (1U == rev)) { bwd++; } else { wrong++; }
                    }
                }
            }
        }
    }
    printf("  forwards %u/%u, backwards %u/%u, wrong letters %u\n",
           (unsigned)fwd, (unsigned)total, (unsigned)bwd, (unsigned)total, (unsigned)wrong);
    (void)snprintf(line, sizeof(line), "all 43 chars x 2 directions x 12 settings x %u trials, 0 wrong",
                   (unsigned)TRIALS);
    check("S3-C05", (fwd == total) && (bwd == total) && (0U == wrong), line);

    /* C06: sensor-spot blur (3 mm stripes): every bar longer and every
     * space shorter by 2/3 of a narrow element, ratio 2.5. Mixing bars
     * and spaces in one wide/narrow split failed here. */
    {
        uint32_t ok_blur = 0U;
        uint32_t blur    = (2U * UNIT) / 3U;

        for (i = 0U; i < N_DATA; i++)
        {
            build(g_ref_ch[i], 25U, 0U, 0, dur);
            for (k = 0U; k < S3_C39_ELEMENTS; k++)
            {
                dur[k] = (0U == (k & 1U)) ? (dur[k] + blur) : (dur[k] - blur);
            }
            if ((S3_C39_OK == s3_code39_decode(dur, S3_C39_ELEMENTS, &ch, &rev)) &&
                (g_ref_ch[i] == ch))
            {
                ok_blur++;
            }
        }
        check("S3-C06", N_DATA == ok_blur, "all chars decode with bars +2/3, spaces -2/3 narrow (blur)");
    }

    /* C07: a stray stripe before and one after the code (31 + 2 elements)
     * still decodes: the 29-element window slides over the pass. */
    {
        uint32_t pass[S3_C39_ELEMENTS + 4U];
        uint32_t ok_stray = 0U;

        for (letter = 'A'; letter <= 'Z'; letter++)
        {
            build(letter, 30U, 0U, 0, dur);
            pass[0] = 5U * UNIT;                     /* stray bar   */
            pass[1] = 4U * UNIT;                     /* white       */
            for (k = 0U; k < S3_C39_ELEMENTS; k++)
            {
                pass[k + 2U] = dur[k];
            }
            pass[S3_C39_ELEMENTS + 2U] = 4U * UNIT;  /* white       */
            pass[S3_C39_ELEMENTS + 3U] = UNIT;       /* stray bar   */
            if ((S3_C39_OK == s3_code39_decode(pass, S3_C39_ELEMENTS + 4U, &ch, &rev)) &&
                (letter == ch))
            {
                ok_stray++;
            }
        }
        check("S3-C07", 26U == ok_stray, "A-Z decode with a stray stripe before and after the code");
    }

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
