/*
 *	s3_nav.c -- see s3_nav.h
 *	Not thread-safe on its own: the kernel layer (s3_bringup.c) calls the
 *	setter and the lookup under mtx_s3.
 */
#include "s3_nav.h"
#include "s3_code39.h"

/* g_map[i] = letter for command (i + 1): LEFT, RIGHT, STRAIGHT, UTURN */
static char g_map[S3_NAV_MAP_LEN] = {'A', 'B', 'C', 'D'};

static char to_upper (char ch)
{
    return ((ch >= 'a') && (ch <= 'z')) ? (char)(ch - 'a' + 'A') : ch;
}

void s3_nav_reset (void)
{
    g_map[0] = 'A'; g_map[1] = 'B'; g_map[2] = 'C'; g_map[3] = 'D';
}

int s3_nav_set_map (char const * p_map)
{
    char     tmp[S3_NAV_MAP_LEN];
    uint16_t pat;
    uint32_t i, j;

    if (0 == p_map)
    {
        return -1;
    }
    for (i = 0U; i < S3_NAV_MAP_LEN; i++)
    {
        tmp[i] = to_upper(p_map[i]);
        if (('\0' == tmp[i]) || ('*' == tmp[i]) || (0 == s3_code39_pattern(tmp[i], &pat)))
        {
            return -1;                           /* too short, start/stop char, or not Code 39 */
        }
        for (j = 0U; j < i; j++)
        {
            if (tmp[j] == tmp[i])
            {
                return -1;                       /* same letter for two commands */
            }
        }
    }
    if ('\0' != p_map[S3_NAV_MAP_LEN])
    {
        return -1;                               /* too long */
    }
    for (i = 0U; i < S3_NAV_MAP_LEN; i++)
    {
        g_map[i] = tmp[i];                       /* commit only a fully valid map */
    }
    return 0;
}

void s3_nav_get_map (char * p_out5)
{
    uint32_t i;
    for (i = 0U; i < S3_NAV_MAP_LEN; i++)
    {
        p_out5[i] = g_map[i];
    }
    p_out5[S3_NAV_MAP_LEN] = '\0';
}

s3_nav_cmd_t s3_nav_from_char (char ch)
{
    uint32_t i;
    for (i = 0U; i < S3_NAV_MAP_LEN; i++)
    {
        if (g_map[i] == ch)
        {
            return (s3_nav_cmd_t)(i + 1U);
        }
    }
    return S3_NAV_NONE;
}

char const * s3_nav_name (s3_nav_cmd_t cmd)
{
    switch (cmd)
    {
        case S3_NAV_LEFT:     return "TURN LEFT";
        case S3_NAV_RIGHT:    return "TURN RIGHT";
        case S3_NAV_STRAIGHT: return "GO STRAIGHT";
        case S3_NAV_UTURN:    return "U-TURN";
        default:              return "NONE";
    }
}
