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
#include <string.h>

#include "s3_code39.h"

/* Element patterns, bar/space alternating starting with a bar, MSB first,
   1 = wide.  Verified against Barcode Sample.pdf for '*', 'A' and 'Z'. */
typedef struct
{
    char     letter;
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

/*!
 * @brief Mark the n_wide longest of the n elements at p_el[first],
 *        p_el[first + 2], ... (every other one: bars or spaces) as wide.
 *
 * @return 1 if the split is clean (shortest wide >= ratio x longest
 *         narrow, or with no wide element: no element >= ratio x
 *         another). *p_min_wide = shortest wide (UINT32_MAX if none).
 */
static int
split_wide (uint32_t const * p_el, uint32_t first, uint32_t count,
            uint32_t n_wide, uint8_t * p_is_wide, uint32_t * p_min_wide)
{
    uint32_t min_wide   = UINT32_MAX;
    uint32_t max_narrow = 0u;
    uint32_t min_narrow = UINT32_MAX;
    uint32_t found;
    uint32_t idx;

    for (found = 0u; found < n_wide; found++)
    {
        uint32_t best = SYMBOL_LEN;

        for (idx = first; idx < (first + (2u * count)); idx += 2u)
        {
            if ((0u == p_is_wide[idx]) &&
                ((SYMBOL_LEN == best) || (p_el[idx] > p_el[best])))
            {
                best = idx;
            }
        }
        p_is_wide[best] = 1u;
        min_wide = (p_el[best] < min_wide) ? p_el[best] : min_wide;
    }
    for (idx = first; idx < (first + (2u * count)); idx += 2u)
    {
        if (0u == p_is_wide[idx])
        {
            max_narrow = (p_el[idx] > max_narrow) ? p_el[idx] : max_narrow;
            min_narrow = (p_el[idx] < min_narrow) ? p_el[idx] : min_narrow;
        }
    }
    *p_min_wide = min_wide;
    if (0u == n_wide)
    {
        /* All narrow: only reject a clearly wide one (jitter is fine). */
        return ((max_narrow * 10u) < (min_narrow * 2u * S3_C39_RATIO_X10))
               ? 1 : 0;
    }
    return ((min_wide * 10u) >= (max_narrow * S3_C39_RATIO_X10)) ? 1 : 0;
}

/*!
 * @brief Bars and spaces are split into wide / narrow SEPARATELY: the
 *        sensor spot makes every bar look longer and every space shorter
 *        by the same amount, so mixing them (a blurred narrow bar vs a
 *        wide space) is what failed on 3 mm stripes. Every Code 39 symbol
 *        is 2 wide bars + 1 wide space, or ('$', '/', '+', '%') 0 wide
 *        bars + 3 wide spaces.
 *
 * @param[out] p_min_wide  Shortest wide SPACE (for the inter-character
 *                         gap check, which is a space too).
 */
static s3_c39_result_t
decode_symbol (uint32_t const * p_el, char * p_char, uint32_t * p_min_wide)
{
    uint8_t  is_wide[SYMBOL_LEN] = {0u};
    uint32_t min_wide_bar;
    uint32_t min_wide   = UINT32_MAX;
    uint16_t pattern    = 0u;
    uint32_t idx;

    if ((0 == split_wide(p_el, 0u, 5u, 2u, is_wide, &min_wide_bar)) ||
        (0 == split_wide(p_el, 1u, 4u, 1u, is_wide, &min_wide)))
    {
        /* The cast discards memset()'s return value, which is the same pointer.
         */
        (void) memset(is_wide, 0, sizeof(is_wide));
        if ((0 == split_wide(p_el, 0u, 5u, 0u, is_wide, &min_wide_bar)) ||
            (0 == split_wide(p_el, 1u, 4u, 3u, is_wide, &min_wide)))
        {
            return S3_C39_BAD_RATIO;
        }
    }

    for (idx = 0u; idx < SYMBOL_LEN; idx++)
    {
        /* The cast is safe: the pattern is SYMBOL_LEN (9) bits, which fits
         * uint16_t.
         */
        pattern = (uint16_t)((pattern << 1) | is_wide[idx]);
    }

    for (idx = 0u; idx < C39_TABLE_LEN; idx++)
    {
        if (g_c39_table[idx].pattern == pattern)
        {
            *p_char     = g_c39_table[idx].letter;
            *p_min_wide = min_wide;
            return S3_C39_OK;
        }
    }

    return S3_C39_BAD_SYMBOL;
}

/*!
 * @brief Decode one frame (start symbol, letter, stop symbol) read in
 *        one direction.
 *
 * @param[in]  p_dur  Element durations of the frame.
 * @param[out] p_char The letter.
 *
 * @return S3_C39_OK, or the first symbol's failure.
 */
static s3_c39_result_t
decode_frame (uint32_t const * p_dur, char * p_char)
{
    char            sym[SYMBOLS];
    uint32_t        min_wide[SYMBOLS];
    uint32_t        sym_idx;
    s3_c39_result_t res;

    for (sym_idx = 0u; sym_idx < SYMBOLS; sym_idx++)
    {
        res = decode_symbol(&p_dur[sym_idx * (SYMBOL_LEN + 1u)], &sym[sym_idx],
                            &min_wide[sym_idx]);
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

/*!
 * @brief Decode one barcode pass, forwards and then reversed (see
 *        s3_code39.h).
 */
s3_c39_result_t
s3_code39_decode (uint32_t const * p_dur, uint32_t count,
                  char * p_char, uint8_t * p_reversed)
{
    uint32_t        rev[S3_C39_ELEMENTS];
    uint32_t        idx;
    uint32_t        ofs;
    s3_c39_result_t res;
    s3_c39_result_t first_res = S3_C39_BAD_COUNT;

    if ((NULL == p_dur) || (NULL == p_char) || (NULL == p_reversed) ||
        (count < S3_C39_ELEMENTS))
    {
        return S3_C39_BAD_COUNT;
    }

    /* Slide a 29-element window over the pass: a stray edge before or
     * after the code (line edge, paper crease) only shifts the frame. */
    for (ofs = 0u; (ofs + S3_C39_ELEMENTS) <= count; ofs++)
    {
        res = decode_frame(&p_dur[ofs], p_char);
        if (S3_C39_OK == res)
        {
            *p_reversed = 0u;
            return res;
        }

        for (idx = 0u; idx < S3_C39_ELEMENTS; idx++)
        {
            rev[idx] = p_dur[ofs + S3_C39_ELEMENTS - 1u - idx];
        }

        if (S3_C39_OK == decode_frame(rev, p_char))
        {
            *p_reversed = 1u;
            return S3_C39_OK;
        }
        if (0u == ofs)
        {
            first_res = res;
        }
    }

    return first_res;   /* report the first forward-read failure reason */
}

/*!
 * @brief The 9-element pattern of a character (see s3_code39.h).
 */
int s3_code39_pattern (char letter, uint16_t * p_pattern)
{
    uint32_t idx;

    for (idx = 0u; idx < C39_TABLE_LEN; idx++)
    {
        if (g_c39_table[idx].letter == letter)
        {
            *p_pattern = g_c39_table[idx].pattern;
            return 1;
        }
    }
    return 0;
}

/*!
 * @brief The 6-bit table index of a character, or S3_C39_IDX_NONE.
 */
uint8_t s3_code39_index (char letter)
{
    uint32_t idx;

    for (idx = 0u; idx < C39_TABLE_LEN; idx++)
    {
        if (g_c39_table[idx].letter == letter)
        {
            /* The cast is safe: idx is below C39_TABLE_LEN (44), which fits
             * uint8_t.
             */
            return (uint8_t)idx;
        }
    }
    return S3_C39_IDX_NONE;
}

/*!
 * @brief The character at a table index, or '?' for an invalid index.
 */
char s3_code39_char (uint8_t idx)
{
    return (idx < C39_TABLE_LEN) ? g_c39_table[idx].letter : '?';
}

/* ------------------------------------------------------------------ nav */

/* g_nav_map[i] = letter for command (i + 1): LEFT, RIGHT, STRAIGHT,
 * U-TURN. Not thread-safe on its own: s3_api.c calls the setter and the
 * lookup under gh_mtx_s3. */
static char g_nav_map[S3_NAV_MAP_LEN] = {'A', 'B', 'C', 'D'};

/*!
 * @brief A lower-case ASCII letter in upper case; anything else as it is.
 */
static char
nav_to_upper (char letter)
{
    /* The cast is safe: a lower-case ASCII letter minus 'a' plus 'A' is an
     * upper-case ASCII letter.
     */
    return ((letter >= 'a') && (letter <= 'z')) ?
           (char) ((letter - 'a') + 'A') : letter;
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
    uint32_t idx;
    uint32_t pos;

    if (NULL == p_map)
    {
        return -1;
    }
    for (idx = 0u; idx < S3_NAV_MAP_LEN; idx++)
    {
        tmp[idx] = nav_to_upper(p_map[idx]);
        if (('\0' == tmp[idx]) || ('*' == tmp[idx]) ||
            (0 == s3_code39_pattern(tmp[idx], &pat)))
        {
            return -1;  /* Too short, start/stop char, or not Code 39.  */
        }
        for (pos = 0u; pos < idx; pos++)
        {
            if (tmp[pos] == tmp[idx])
            {
                return -1;  /* Same letter for two commands.            */
            }
        }
    }
    if ('\0' != p_map[S3_NAV_MAP_LEN])
    {
        return -1;      /* Too long.                                     */
    }
    for (idx = 0u; idx < S3_NAV_MAP_LEN; idx++)
    {
        g_nav_map[idx] = tmp[idx];  /* Commit only a fully valid map.        */
    }
    return 0;
}

/*!
 * @brief Copy the current map, e.g. "DCBA" plus the NUL.
 */
void
s3_nav_get_map (char * p_out5)
{
    uint32_t idx;

    for (idx = 0u; idx < S3_NAV_MAP_LEN; idx++)
    {
        p_out5[idx] = g_nav_map[idx];
    }
    p_out5[S3_NAV_MAP_LEN] = '\0';
}

/*!
 * @brief The command for a decoded letter; S3_NAV_NONE if not in the map.
 */
s3_nav_cmd_t
s3_nav_from_char (char letter)
{
    s3_nav_cmd_t cmd = S3_NAV_NONE;
    uint32_t     idx;

    for (idx = 0u; idx < S3_NAV_MAP_LEN; idx++)
    {
        if (g_nav_map[idx] == letter)
        {
            /* The cast is safe: idx + 1 is 1 ... S3_NAV_MAP_LEN, each a valid
             * s3_nav_cmd_t value.
             */
            cmd = (s3_nav_cmd_t) (idx + 1u);
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

