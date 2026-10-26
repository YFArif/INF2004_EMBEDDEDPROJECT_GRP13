/*
 *	s3_code39.c
 *	Subsystem 3 -- Code 39 frame decoder (pure C, no kernel calls).
 *
 *	Speed-independent: in every 9-element symbol exactly 3 elements are
 *	wide, so the 3 longest durations are taken as wide.  A constant robot
 *	speed scales all durations equally and cancels out.
 */
#include "s3_code39.h"

/* Element patterns, bar/space alternating starting with a bar, MSB first,
   1 = wide.  Verified against Barcode Sample.pdf for '*', 'A' and 'Z'. */
typedef struct
{
    char     ch;
    uint16_t pattern;
} s3_c39_entry_t;

static s3_c39_entry_t const g_c39_table[] =
{
    {'0', 0x034}, {'1', 0x121}, {'2', 0x061}, {'3', 0x160}, {'4', 0x031},
    {'5', 0x130}, {'6', 0x070}, {'7', 0x025}, {'8', 0x124}, {'9', 0x064},
    {'A', 0x109}, {'B', 0x049}, {'C', 0x148}, {'D', 0x019}, {'E', 0x118},
    {'F', 0x058}, {'G', 0x00D}, {'H', 0x10C}, {'I', 0x04C}, {'J', 0x01C},
    {'K', 0x103}, {'L', 0x043}, {'M', 0x142}, {'N', 0x013}, {'O', 0x112},
    {'P', 0x052}, {'Q', 0x007}, {'R', 0x106}, {'S', 0x046}, {'T', 0x016},
    {'U', 0x181}, {'V', 0x0C1}, {'W', 0x1C0}, {'X', 0x091}, {'Y', 0x190},
    {'Z', 0x0D0}, {'-', 0x085}, {'.', 0x184}, {' ', 0x0C4}, {'*', 0x094},
    {'$', 0x0A8}, {'/', 0x0A2}, {'+', 0x08A}, {'%', 0x02A}
};

#define C39_TABLE_LEN   (sizeof(g_c39_table) / sizeof(g_c39_table[0]))
#define SYMBOL_LEN      9U

static s3_c39_result_t
decode_symbol (uint32_t const * p_el, char * p_char, uint32_t * p_min_wide)
{
    uint8_t  is_wide[SYMBOL_LEN] = {0U};
    uint32_t min_wide   = UINT32_MAX;
    uint32_t max_narrow = 0U;
    uint16_t pattern    = 0U;
    uint32_t k;
    uint32_t i;

    /* Mark the 3 longest elements as wide. */
    for (k = 0U; k < 3U; k++)
    {
        uint32_t best = SYMBOL_LEN;

        for (i = 0U; i < SYMBOL_LEN; i++)
        {
            if ((0U == is_wide[i]) &&
                ((SYMBOL_LEN == best) || (p_el[i] > p_el[best])))
            {
                best = i;
            }
        }
        is_wide[best] = 1U;
    }

    for (i = 0U; i < SYMBOL_LEN; i++)
    {
        pattern = (uint16_t)((pattern << 1) | is_wide[i]);

        if (1U == is_wide[i])
        {
            min_wide = (p_el[i] < min_wide) ? p_el[i] : min_wide;
        }
        else
        {
            max_narrow = (p_el[i] > max_narrow) ? p_el[i] : max_narrow;
        }
    }

    if ((min_wide * 10U) < (max_narrow * S3_C39_RATIO_X10))
    {
        return S3_C39_BAD_RATIO;
    }

    for (i = 0U; i < C39_TABLE_LEN; i++)
    {
        if (g_c39_table[i].pattern == pattern)
        {
            *p_char     = g_c39_table[i].ch;
            *p_min_wide = min_wide;
            return S3_C39_OK;
        }
    }

    return S3_C39_BAD_SYMBOL;
}

static s3_c39_result_t
decode_frame (uint32_t const * p_dur, char * p_char)
{
    char            sym[3];
    uint32_t        min_wide[3];
    uint32_t        s;
    s3_c39_result_t res;

    for (s = 0U; s < 3U; s++)
    {
        res = decode_symbol(&p_dur[s * (SYMBOL_LEN + 1U)], &sym[s], &min_wide[s]);
        if (S3_C39_OK != res)
        {
            return res;
        }
    }

    /* The two inter-character gaps must be narrow. */
    if ((p_dur[9] >= min_wide[0]) || (p_dur[19] >= min_wide[1]))
    {
        return S3_C39_BAD_RATIO;
    }

    if (('*' != sym[0]) || ('*' != sym[2]))
    {
        return S3_C39_NO_START_STOP;
    }

    *p_char = sym[1];
    return S3_C39_OK;
}

s3_c39_result_t
s3_code39_decode (uint32_t const * p_dur, uint32_t count,
                  char * p_char, uint8_t * p_reversed)
{
    uint32_t        rev[S3_C39_ELEMENTS];
    uint32_t        i;
    s3_c39_result_t res;

    if ((0 == p_dur) || (S3_C39_ELEMENTS != count))
    {
        return S3_C39_BAD_COUNT;
    }

    res = decode_frame(p_dur, p_char);
    if (S3_C39_OK == res)
    {
        *p_reversed = 0U;
        return res;
    }

    for (i = 0U; i < S3_C39_ELEMENTS; i++)
    {
        rev[i] = p_dur[S3_C39_ELEMENTS - 1U - i];
    }

    if (S3_C39_OK == decode_frame(rev, p_char))
    {
        *p_reversed = 1U;
        return S3_C39_OK;
    }

    return res;   /* report the forward-read failure reason */
}

int s3_code39_pattern (char ch, uint16_t * p_pattern)
{
    uint32_t i;

    for (i = 0U; i < C39_TABLE_LEN; i++)
    {
        if (g_c39_table[i].ch == ch)
        {
            *p_pattern = g_c39_table[i].pattern;
            return 1;
        }
    }
    return 0;
}

uint8_t s3_code39_index (char ch)
{
    uint32_t i;

    for (i = 0U; i < C39_TABLE_LEN; i++)
    {
        if (g_c39_table[i].ch == ch)
        {
            return (uint8_t)i;
        }
    }
    return S3_C39_IDX_NONE;
}

char s3_code39_char (uint8_t idx)
{
    return (idx < C39_TABLE_LEN) ? g_c39_table[idx].ch : '?';
}
