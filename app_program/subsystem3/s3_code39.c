/** @file s3_code39.c
 *
 * @brief Subsystem 3 -- Code 39 frame decoder and the letter to command
 *        map. See s3_code39.h.
 *
 * @par
 * Speed-independent: in every 9-element symbol exactly 3 elements are
 * wide, so the 3 longest durations are taken as wide. A constant robot
 * speed scales all durations equally and cancels out.
 */

#include <stddef.h>
#include <stdint.h>

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
#define SYMBOL_LEN      (9u)
#define WIDE_PER_SYMBOL (3u)
#define SYMBOLS         (3u)    /* start, character, stop */

static s3_c39_result_t
decode_symbol (uint32_t const * p_el, char * p_char, uint32_t * p_min_wide)
{
    uint8_t  is_wide[SYMBOL_LEN] = {0u};
    uint32_t min_wide   = UINT32_MAX;
    uint32_t max_narrow = 0u;
    uint16_t pattern    = 0u;
    uint32_t k;
    uint32_t i;

    /* Mark the 3 longest elements as wide. */
    for (k = 0u; k < WIDE_PER_SYMBOL; k++)
    {
        uint32_t best = SYMBOL_LEN;

        for (i = 0u; i < SYMBOL_LEN; i++)
        {
            if ((0u == is_wide[i]) &&
                ((SYMBOL_LEN == best) || (p_el[i] > p_el[best])))
            {
                best = i;
            }
        }
        is_wide[best] = 1u;
    }

    for (i = 0u; i < SYMBOL_LEN; i++)
    {
        pattern = (uint16_t)((pattern << 1) | is_wide[i]);

        if (1u == is_wide[i])
        {
            min_wide = (p_el[i] < min_wide) ? p_el[i] : min_wide;
        }
        else
        {
            max_narrow = (p_el[i] > max_narrow) ? p_el[i] : max_narrow;
        }
    }

    if ((min_wide * 10u) < (max_narrow * S3_C39_RATIO_X10))
    {
        return S3_C39_BAD_RATIO;
    }

    for (i = 0u; i < C39_TABLE_LEN; i++)
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
    char            sym[SYMBOLS];
    uint32_t        min_wide[SYMBOLS];
    uint32_t        s;
    s3_c39_result_t res;

    for (s = 0u; s < SYMBOLS; s++)
    {
        res = decode_symbol(&p_dur[s * (SYMBOL_LEN + 1u)], &sym[s],
                            &min_wide[s]);
        if (S3_C39_OK != res)
        {
            return res;
        }
    }

    /* The two inter-character gaps must be narrow. */
    if ((p_dur[SYMBOL_LEN] >= min_wide[0]) ||
        (p_dur[(2u * SYMBOL_LEN) + 1u] >= min_wide[1]))
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

    if ((NULL == p_dur) || (NULL == p_char) || (NULL == p_reversed) ||
        (S3_C39_ELEMENTS != count))
    {
        return S3_C39_BAD_COUNT;
    }

    res = decode_frame(p_dur, p_char);
    if (S3_C39_OK == res)
    {
        *p_reversed = 0u;
        return res;
    }

    for (i = 0u; i < S3_C39_ELEMENTS; i++)
    {
        rev[i] = p_dur[S3_C39_ELEMENTS - 1u - i];
    }

    if (S3_C39_OK == decode_frame(rev, p_char))
    {
        *p_reversed = 1u;
        return S3_C39_OK;
    }

    return res;   /* report the forward-read failure reason */
}

int s3_code39_pattern (char ch, uint16_t * p_pattern)
{
    uint32_t i;

    for (i = 0u; i < C39_TABLE_LEN; i++)
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

    for (i = 0u; i < C39_TABLE_LEN; i++)
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

/* ------------------------------------------------------------------ nav */

/* g_nav_map[i] = letter for command (i + 1): LEFT, RIGHT, STRAIGHT,
 * U-TURN. Not thread-safe on its own: s3_api.c calls the setter and the
 * lookup under mtx_s3. */
static char g_nav_map[S3_NAV_MAP_LEN] = {'A', 'B', 'C', 'D'};

static char
nav_to_upper (char ch)
{
    return ((ch >= 'a') && (ch <= 'z')) ? (char) ((ch - 'a') + 'A') : ch;
}

/*!
 * @brief Back to the write-up example map "ABCD".
 */
void
s3_nav_reset (void)
{
    g_nav_map[0] = 'A';
    g_nav_map[1] = 'B';
    g_nav_map[2] = 'C';
    g_nav_map[3] = 'D';
}

/*!
 * @brief Set the letters for LEFT, RIGHT, STRAIGHT, U-TURN, e.g. "DCBA".
 *
 * @return 0 if accepted; -1 if rejected (the old map is kept).
 */
int
s3_nav_set_map (char const * p_map)
{
    char     tmp[S3_NAV_MAP_LEN];
    uint16_t pat;
    uint32_t i;
    uint32_t j;

    if (NULL == p_map)
    {
        return -1;
    }
    for (i = 0u; i < S3_NAV_MAP_LEN; i++)
    {
        tmp[i] = nav_to_upper(p_map[i]);
        if (('\0' == tmp[i]) || ('*' == tmp[i]) ||
            (0 == s3_code39_pattern(tmp[i], &pat)))
        {
            return -1;  /* Too short, start/stop char, or not Code 39.  */
        }
        for (j = 0u; j < i; j++)
        {
            if (tmp[j] == tmp[i])
            {
                return -1;  /* Same letter for two commands.            */
            }
        }
    }
    if ('\0' != p_map[S3_NAV_MAP_LEN])
    {
        return -1;      /* Too long.                                     */
    }
    for (i = 0u; i < S3_NAV_MAP_LEN; i++)
    {
        g_nav_map[i] = tmp[i];  /* Commit only a fully valid map.        */
    }
    return 0;
}

/*!
 * @brief Copy the current map, e.g. "DCBA" plus '\0'.
 */
void
s3_nav_get_map (char * p_out5)
{
    uint32_t i;

    for (i = 0u; i < S3_NAV_MAP_LEN; i++)
    {
        p_out5[i] = g_nav_map[i];
    }
    p_out5[S3_NAV_MAP_LEN] = '\0';
}

/*!
 * @brief The command for a decoded letter; S3_NAV_NONE if not in the map.
 */
s3_nav_cmd_t
s3_nav_from_char (char ch)
{
    s3_nav_cmd_t cmd = S3_NAV_NONE;
    uint32_t     i;

    for (i = 0u; i < S3_NAV_MAP_LEN; i++)
    {
        if (g_nav_map[i] == ch)
        {
            cmd = (s3_nav_cmd_t) (i + 1u);
            break;
        }
    }
    return cmd;
}

/*!
 * @brief Console text for a command.
 */
char const *
s3_nav_name (s3_nav_cmd_t cmd)
{
    char const * p_name = "NONE";

    switch (cmd)
    {
        case S3_NAV_LEFT:
            p_name = "TURN LEFT";
        break;

        case S3_NAV_RIGHT:
            p_name = "TURN RIGHT";
        break;

        case S3_NAV_STRAIGHT:
            p_name = "GO STRAIGHT";
        break;

        case S3_NAV_UTURN:
            p_name = "U-TURN";
        break;

        default:
            // S3_NAV_NONE or an invalid value.
        break;
    }
    return p_name;
}

/*** end of file ***/
