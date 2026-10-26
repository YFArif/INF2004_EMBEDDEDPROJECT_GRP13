/*
 *	test_code39.c -- host (laptop) unit test for s3_code39.c
 *
 *	Build and run on any PC/WSL with gcc, no Pico needed:
 *	  gcc -Wall -Wextra -I../../../app_program/subsystem3 \
 *	      test_code39.c ../../../app_program/subsystem3/s3_code39.c -o t39 && ./t39
 *
 *	Element widths below are measured (in pixels) from Barcode Sample.pdf
 *	page 1 ("*A*") and page 3 ("*Z*"); unit does not matter.
 */
#include <stdio.h>
#include <string.h>
#include "s3_code39.h"

static uint32_t const g_a[29] = {17,49,17,16,50,16,50,16,17, 16,
                                 50,16,17,16,17,49,17,16,50, 16,
                                 17,49,17,16,50,16,50,16,17};
static uint32_t const g_z[29] = {17,49,17,16,50,16,50,16,17, 16,
                                 17,49,50,16,50,16,17,16,17, 16,
                                 17,49,17,16,50,16,50,16,17};
static int g_fail = 0;

static void check (char const * p_id, uint32_t const * p_dur, uint32_t n,
                   s3_c39_result_t want_res, char want_ch, uint8_t want_rev)
{
    char            ch  = '?';
    uint8_t         rev = 9U;
    s3_c39_result_t res = s3_code39_decode(p_dur, n, &ch, &rev);
    int ok = (res == want_res) &&
             ((S3_C39_OK != res) || ((ch == want_ch) && (rev == want_rev)));

    printf("[TEST] %-28s %s  (res=%d ch=%c rev=%u)\n", p_id, ok ? "PASS" : "FAIL",
           (int)res, ch, (unsigned)rev);
    g_fail += !ok;
}

int main (void)
{
    uint32_t buf[29];
    uint32_t i;

    check("sample *A*",              g_a, 29U, S3_C39_OK, 'A', 0U);
    check("sample *Z*",              g_z, 29U, S3_C39_OK, 'Z', 0U);

    for (i = 0U; i < 29U; i++) { buf[i] = g_a[28U - i]; }
    check("*A* read backwards",      buf, 29U, S3_C39_OK, 'A', 1U);

    for (i = 0U; i < 29U; i++) { buf[i] = g_a[i] * 4U; }          /* 4x slower */
    check("*A* at quarter speed",    buf, 29U, S3_C39_OK, 'A', 0U);

    for (i = 0U; i < 29U; i++) { buf[i] = g_a[i] * (100U + i) / 100U; } /* 28 % slow-down across frame */
    check("*A* gradual slow-down",   buf, 29U, S3_C39_OK, 'A', 0U);

    check("only 27 elements",        g_a, 27U, S3_C39_BAD_COUNT, 0, 0U);

    memcpy(buf, g_a, sizeof(buf)); buf[12] = 40U;                  /* blurred narrow bar */
    check("blurred bar -> reject",   buf, 29U, S3_C39_BAD_RATIO, 0, 0U);

    for (i = 0U; i < 29U; i++) { buf[i] = 20U; }                   /* all equal = noise */
    check("all-equal noise -> reject", buf, 29U, S3_C39_BAD_RATIO, 0, 0U);

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
