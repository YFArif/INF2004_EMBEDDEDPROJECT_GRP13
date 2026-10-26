/** @file s3_code39.h
 *
 * @brief Subsystem 3 -- Code 39 frame decoder (pure C, no kernel calls,
 *        unit-tested on a laptop: testing/subsystem3/host/test_code39.c).
 *
 * @par
 * One frame is *X*: start, one character, stop = 3 symbols of 9 elements
 * plus 2 narrow gaps = 29 durations. Both the digital and the analogue
 * barcode path feed their durations here. Every character decodes read
 * forwards or backwards.
 */
#ifndef S3_CODE39_H
#define S3_CODE39_H

#include <stdint.h>

#define S3_C39_ELEMENTS     (29u)   /* *X* = 3 symbols x 9 + 2 gaps          */
#ifndef S3_C39_RATIO_X10
#define S3_C39_RATIO_X10    (13u)   /* Shortest wide >= 1.3 x longest    */
                                    /* narrow (sim: 0 wrong decodes).    */
#endif

typedef enum
{
    S3_C39_OK = 0,
    S3_C39_BAD_COUNT,        /* not 29 elements                           */
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

#endif /* S3_CODE39_H */

/*** end of file ***/
