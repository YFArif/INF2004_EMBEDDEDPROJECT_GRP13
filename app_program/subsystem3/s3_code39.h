/** @file s3_code39.h
 *
 * @brief Subsystem 3 -- Code 39 frame decoder and the barcode letter to
 *        navigation command map (pure C, no kernel calls, unit-tested on
 *        a laptop: testing/subsystem3/host/test_code39*.c, test_nav.c).
 *
 * @par Decoder
 * One frame is *X*: start, one character, stop = 3 symbols of 9 elements
 * plus 2 narrow gaps = 29 durations. Both the digital and the analogue
 * barcode path feed their durations here. Every character decodes read
 * forwards or backwards.
 *
 * @par Navigation map (FR5)
 * WHICH letter means which turn is given only on the demo day and the
 * code may NOT be recompiled, so the mapping is a run-time table:
 *
 *   s3_nav_set_map("DCBA")  ->  D = TURN LEFT, C = TURN RIGHT,
 *                               B = GO STRAIGHT, A = U-TURN
 *
 * i.e. the string lists the letters for LEFT, RIGHT, STRAIGHT, U-TURN in
 * that order (UNVERIFIED: confirm this order with the instructor).
 * Rules: exactly 4 characters, each a Code 39 data character, no repeats;
 * lower case is accepted. An invalid string is rejected and the old map
 * is kept. Before any map is set, the write-up example A/B/C/D applies.
 * Any letter not in the map decodes fine but gives S3_NAV_NONE.
 */
#ifndef S3_CODE39_H
#define S3_CODE39_H

#include <stdint.h>

#define S3_C39_ELEMENTS     (29u)   /* *X* = 3 symbols x 9 + 2 gaps          */
#ifndef S3_C39_RATIO_X10
#define S3_C39_RATIO_X10    (12u)   /* Shortest wide >= 1.2 x longest    */
                                    /* narrow (C05: 0 wrong; 1.1 gave 24 */
                                    /* wrong, 1.3 lost blurred 3 mm).    */
#endif

typedef enum
{
    S3_C39_OK = 0,
    S3_C39_BAD_COUNT,        /* fewer than 29 elements (a longer pass is  */
                             /* searched with a sliding 29 window)        */
    S3_C39_BAD_RATIO,        /* wide/narrow not separable in some symbol  */
    S3_C39_BAD_SYMBOL,       /* pattern is not a Code 39 character        */
    S3_C39_NO_START_STOP     /* decoded, but not framed by '*'            */
} s3_c39_result_t;

/*
 * Decode one frame.
 *   p_dur      element durations (bar, space, bar, ...) in any unit (us)
 *   count      number of durations
 *   p_char     out: the middle character, e.g. 'A'
 *   p_reversed out: 1 if the frame only matched when read backwards
 */
s3_c39_result_t s3_code39_decode(uint32_t const * p_dur, uint32_t count,
                                 char * p_char, uint8_t * p_reversed);

/*
 * The 9-element pattern of a character (bit 8 = first element, 1 = wide).
 * Returns 0 if the character is not in the Code 39 table.  Used by the
 * host test to prove every stored character decodes in both directions.
 */
int s3_code39_pattern(char ch, uint16_t * p_pattern);

/* 6-bit character code for packed messages (Change Requirement 1):
   index 0..43 into the Code 39 table
   ('0'..'9', 'A'..'Z', '-', '.', ' ', '*', '$', '/', '+', '%').
   s3_code39_index returns 63 if the character is not in the table. */
#define S3_C39_IDX_NONE     (63u)
uint8_t s3_code39_index(char ch);
char    s3_code39_char(uint8_t idx);     /* '?' for an invalid index */

/* ------------------------------------------------------------------ nav */
typedef enum
{
    S3_NAV_NONE = 0,
    S3_NAV_LEFT,
    S3_NAV_RIGHT,
    S3_NAV_STRAIGHT,
    S3_NAV_UTURN
} s3_nav_cmd_t;

#define S3_NAV_MAP_LEN   (4u)         /* LEFT, RIGHT, STRAIGHT, U-TURN */

void         s3_nav_reset(void);                    /* Back to "ABCD".    */
int          s3_nav_set_map(char const * p_map);    /* 0 ok, -1 rejected. */
void         s3_nav_get_map(char * p_out5);         /* "DCBA" + '\0'.     */
s3_nav_cmd_t s3_nav_from_char(char ch);
char const * s3_nav_name(s3_nav_cmd_t cmd);

#endif /* S3_CODE39_H */

/*** end of file ***/
