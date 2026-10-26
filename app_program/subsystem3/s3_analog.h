/*
 *	s3_analog.h
 *	Subsystem 3 -- turn a sampled analogue barcode signal into element
 *	widths (pure C, no kernel calls, unit-tested on a laptop).
 *
 *	Method: find alternating peaks (black) and valleys (white) that stand
 *	out from their neighbours by at least a "prominence" P, then time each
 *	stripe edge where the signal crosses halfway between a peak and the
 *	next valley.  Each edge uses its OWN local peak/valley, so a narrow
 *	stripe that only reaches mid-grey (blurred by the sensor spot) still
 *	gives an edge -- a single fixed threshold would miss it.
 */
#ifndef S3_ANALOG_H
#define S3_ANALOG_H

#include <stdint.h>

/* Samples are 8-bit (ADC 12-bit >> 4, Change Requirement 1: the barcode only
   needs ~230 grey levels between white ~240 and black ~3960; noise is ~5 counts
   of 4096, i.e. below one 8-bit step).  Thresholds below are in 8-bit units. */
#define S3_AN_MIN_RANGE      19U    /* below this max-min the window is just noise (was 300/16) */
#define S3_AN_MIN_PROM       9U     /* smallest accepted peak/valley prominence    (was 150/16) */
#define S3_AN_PROM_PCT       12U    /* prominence = 12 % of the window's max-min      */
#define S3_AN_MAX_EXTREMA    80U

typedef struct
{
    uint8_t  min;          /* lowest sample in the window (8-bit)         */
    uint8_t  max;          /* highest sample in the window (8-bit)        */
    uint8_t  prominence;   /* P actually used (8-bit)                     */
    uint8_t  extrema;      /* peaks + valleys found (<= S3_AN_MAX_EXTREMA) */
} s3_an_stats_t;

/*
 * p_x       8-bit samples (higher = darker)
 * n         number of samples
 * dt_us     sample period in microseconds
 * p_dur     out: element durations in microseconds (bar, space, bar, ...)
 * max_dur   capacity of p_dur
 * p_stats   out: diagnostics (may be 0)
 * returns   number of durations written (29 for one clean *X* frame)
 */
uint32_t s3_analog_widths(uint8_t const * p_x, uint32_t n, uint32_t dt_us,
                          uint32_t * p_dur, uint32_t max_dur,
                          s3_an_stats_t * p_stats);

#endif /* S3_ANALOG_H */
