/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file code39_dump.c
 *
 * @brief S1-U24 drift guard: print S3's Code 39 table as S3's own
 *        s3_code39_char() gives it.
 *
 * @par What it is for
 * The dashboard side keeps two copies of S3's table, CODE39 in
 * app_program/subsystem1/dashboard/s1_decode.py and in
 * testing/subsystem1/cmd_rtt.py. run_decoder_test.sh builds this program
 * with S3's app_program/subsystem3/s3_code39.c and compares its output
 * with both copies, so a change to S3's table fails the test instead of
 * sending the wrong letters.
 *
 * @par Output
 * One line: s3_code39_char(i) for i = 0 to 63, each as two upper-case hex
 * digits, so the space at index 38 cannot be lost. Indices 44 to 63 are
 * not in the table and give '?' (3F).
 *
 * @par Test code
 * This module is never built into the firmware. int main(void) is the
 * form the C standard requires (Barr-C 5.2.a does not apply to it).
 */

#include "code39_dump.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../subsystem3/s3_code39.h"

/* Every value a 6-bit letter field can hold. */
#define INDEX_COUNT             64U

/*!
 * @brief Print the table and exit.
 *
 * @return EXIT_SUCCESS.
 */
int
main (void)
{
    uint32_t index;
    uint8_t  symbol;

    for (index = 0U; index < INDEX_COUNT; index++)
    {
        /* index is below 64, so it fits uint8_t; the character's bits are
         * read as an unsigned byte, which every char value fits.
         */
        symbol = (uint8_t)s3_code39_char((uint8_t)index);
        (void)printf("%02X", (unsigned int)symbol);
    }

    (void)printf("\n");

    return (EXIT_SUCCESS);
}

/*** end of file ***/
