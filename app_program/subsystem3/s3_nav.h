/*
 *	s3_nav.h
 *	Subsystem 3 -- barcode letter -> navigation command (pure C, unit-tested).
 *
 *	The decoder already knows every Code 39 character (0-9, A-Z, - . space
 *	$ / + %).  WHICH letter means which turn is given only on the demo day
 *	and the code may NOT be recompiled, so the mapping is a run-time table:
 *
 *	  s3_nav_set_map("DCBA")  ->  D = TURN LEFT, C = TURN RIGHT,
 *	                              B = GO STRAIGHT, A = U-TURN
 *
 *	i.e. the string lists the letters for LEFT, RIGHT, STRAIGHT, U-TURN in
 *	that order (UNVERIFIED: confirm this order with the instructor).
 *	Rules: exactly 4 characters, each a Code 39 data character, no repeats;
 *	lower case is accepted.  An invalid string is rejected and the old map
 *	is kept.  Before any map is set, the write-up example A/B/C/D applies.
 *	Any letter not in the map decodes fine but gives S3_NAV_NONE.
 */
#ifndef S3_NAV_H
#define S3_NAV_H

#include <stdint.h>

typedef enum
{
    S3_NAV_NONE = 0,
    S3_NAV_LEFT,
    S3_NAV_RIGHT,
    S3_NAV_STRAIGHT,
    S3_NAV_UTURN
} s3_nav_cmd_t;

#define S3_NAV_MAP_LEN   4U          /* LEFT, RIGHT, STRAIGHT, U-TURN */

void         s3_nav_reset(void);                         /* back to "ABCD" default */
int          s3_nav_set_map(char const * p_map);         /* 0 = ok, -1 = rejected  */
void         s3_nav_get_map(char * p_out5);              /* e.g. "DCBA" + '\0'     */
s3_nav_cmd_t s3_nav_from_char(char ch);
char const * s3_nav_name(s3_nav_cmd_t cmd);

#endif /* S3_NAV_H */
