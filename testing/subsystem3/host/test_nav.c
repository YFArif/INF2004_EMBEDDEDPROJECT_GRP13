/*
 *	test_nav.c -- host test: every stored Code 39 character decodes (both
 *	directions) and the run-time letter -> command map works. Normally
 *	run by ../run_host_tests.sh, or:
 *
 *	  gcc -std=c99 -Wall -Wextra -I../../../app_program/subsystem3 test_nav.c \
 *	      ../../../app_program/subsystem3/s3_code39.c \
 *	      -o tnav && ./tnav
 */
#include <stdio.h>
#include <string.h>
#include "s3_code39.h"

static int g_fail = 0;

static void check (char const * p_id, int ok, char const * p_what)
{
    printf("[TEST] %-8s %s  %s\n", p_id, ok ? "PASS" : "FAIL", p_what);
    g_fail += !ok;
}

/* *X* as 29 element widths: narrow = n, wide = w, inter-character gap = n */
static void build (char ch, uint32_t n, uint32_t w, int reversed, uint32_t * p_out)
{
    char const frame[3] = {'*', ch, '*'};
    uint32_t   el[29];
    uint32_t   k = 0U, c, b;
    uint16_t   pat = 0U;

    for (c = 0U; c < 3U; c++)
    {
        (void)s3_code39_pattern(frame[c], &pat);
        for (b = 0U; b < 9U; b++)
        {
            el[k++] = ((pat >> (8U - b)) & 1U) ? w : n;
        }
        if (c < 2U)
        {
            el[k++] = n;
        }
    }
    for (k = 0U; k < 29U; k++)
    {
        p_out[k] = reversed ? el[28U - k] : el[k];
    }
}

int main (void)
{
    static char const k_all[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    uint32_t d[29];
    char     map[5];
    char     ch;
    uint8_t  rev;
    uint32_t i;
    int      ok, r;

    /* C01: all 43 data characters decode, forwards and backwards, at ratio 3 and 2 */
    ok = 1;
    for (i = 0U; i < (uint32_t)strlen(k_all); i++)
    {
        for (r = 0; r < 2; r++)
        {
            build(k_all[i], 300U, 900U, r, d);
            ch = '?'; rev = 9U;
            if ((S3_C39_OK != s3_code39_decode(d, 29U, &ch, &rev)) || (ch != k_all[i]) || (rev != (uint8_t)r)) { ok = 0; printf("  fail %c rev=%d\n", k_all[i], r); }
            build(k_all[i], 300U, 600U, r, d);
            if ((S3_C39_OK != s3_code39_decode(d, 29U, &ch, &rev)) || (ch != k_all[i])) { ok = 0; printf("  fail2 %c rev=%d\n", k_all[i], r); }
        }
    }
    check("S3-C01", ok, "all 43 Code 39 characters decode forwards and backwards (ratio 3 and 2)");

    /* C02: 6-bit character code used in the packed S3_NAV_MSG / S3_STATUS */
    ok = 1;
    for (i = 0U; i < (uint32_t)strlen(k_all); i++)
    {
        uint8_t idx = s3_code39_index(k_all[i]);
        if ((idx > 43U) || (s3_code39_char(idx) != k_all[i])) { ok = 0; }
    }
    ok = ok && (S3_C39_IDX_NONE == s3_code39_index('?')) && ('?' == s3_code39_char(63U)) &&
         ('*' == s3_code39_char(s3_code39_index('*')));
    check("S3-C02", ok, "every character survives the 6-bit index round trip; unknown -> 63");

    /* N01: default before the day's command = write-up example A,B,C,D */
    s3_nav_reset();
    check("S3-N01", (S3_NAV_LEFT == s3_nav_from_char('A')) && (S3_NAV_UTURN == s3_nav_from_char('D')) &&
                    (S3_NAV_NONE == s3_nav_from_char('Z')), "default map ABCD");

    /* N02: the day's command "DCBA" */
    check("S3-N02", (0 == s3_nav_set_map("DCBA")) &&
                    (S3_NAV_LEFT == s3_nav_from_char('D')) && (S3_NAV_RIGHT == s3_nav_from_char('C')) &&
                    (S3_NAV_STRAIGHT == s3_nav_from_char('B')) && (S3_NAV_UTURN == s3_nav_from_char('A')),
          "\"DCBA\" -> D=LEFT C=RIGHT B=STRAIGHT A=U-TURN");

    /* N03: any letters, digits, lower case */
    check("S3-N03", (0 == s3_nav_set_map("zx1w")) && (S3_NAV_LEFT == s3_nav_from_char('Z')) &&
                    (S3_NAV_STRAIGHT == s3_nav_from_char('1')) && (S3_NAV_NONE == s3_nav_from_char('A')),
          "\"zx1w\" accepted (upper-cased); letters not in the map -> NONE");

    /* N04: bad commands are rejected and the previous map stays */
    (void)s3_nav_set_map("DCBA");
    ok = (-1 == s3_nav_set_map("DCB"))   && (-1 == s3_nav_set_map("DCBAE")) &&
         (-1 == s3_nav_set_map("DDBA"))  && (-1 == s3_nav_set_map("DC*A"))  &&
         (-1 == s3_nav_set_map("DC?A"))  && (-1 == s3_nav_set_map(0));
    s3_nav_get_map(map);
    check("S3-N04", ok && (0 == strcmp(map, "DCBA")), "short/long/repeat/'*'/invalid/NULL rejected, old map kept");

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
