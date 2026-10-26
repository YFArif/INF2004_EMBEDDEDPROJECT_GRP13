/*
 *	s3_bringup.c
 *	Subsystem 3 (Buddy 3) -- sensor layer, desk tests and the RUN mode.
 *
 *	S3_TEST = 0 (RUN) is the real subsystem: line_task (calibration, PD
 *	steering, junction, line lost) + barcode task (decode, fuse, publish a
 *	navigation command).  Its public interface is s3_api.h.
 *	S3_TEST = 1..4 are the bench tests used during bring-up.
 *
 *	Reads the barcode sensor TWO independent ways and cross-checks them:
 *	  DIGITAL  the module's comparator output (DOUT): a GPIO interrupt
 *	           timestamps every edge (flips < 2 ms apart = chatter, dropped)
 *	  ANALOGUE the module's analogue output (AOUT): a hardware-timer
 *	           interrupt samples it every 0.5 ms into a ring buffer; after
 *	           the pass, s3_analog_widths() finds stripe edges from local
 *	           peaks/valleys, so blurred narrow stripes still count
 *	Both width lists go through the same Code 39 decoder (s3_code39.c),
 *	then the results are fused:
 *	  both decode the same letter  -> ACCEPT (both agree)
 *	  only one decodes             -> ACCEPT, flagged single-source
 *	  both decode, letters differ  -> REJECT (disagree)
 *	  neither decodes              -> REJECT
 *
 *	It also prints the two line sensors every 200 ms (S3-U01).
 *
 *	ADC: this file drives the ADC registers directly from the sampler
 *	interrupt and does NOT use the kernel ADC driver ("adca"), so there is
 *	exactly one ADC owner.  Both interrupts are enabled from this task,
 *	which is pinned to core 1, because RP2040 interrupt enables are per core.
 *	NOTE: the final design has ONE shared gpio_isr for S2/S3/S5 (IF-11).
 */
#include "subsystem3.h"
#include "s3_code39.h"
#include "s3_analog.h"
#include "s3_line.h"
#include "s3_nav.h"
#include "../subsystem2/subsystem2.h"	/* subsystem2_gpio_isr_register() */

/* ---------------------------------------------------------------- config */
/* Which build this is:
   0 = RUN      the real subsystem, robot wiring: barcode DOUT GP6 + AOUT GP26 (ADC0),
                Centre GP27 (ADC1), outer GP28 (ADC2).  Publishes mbf_nav / status / steer_sp / flg_s3.
   1 = BARCODE  desk: one sensor, DOUT -> GP26, AOUT -> GP27 (ADC1); prints analog + digital, decodes passes
   2 = CENTRE   desk: one sensor, AOUT -> GP27 (ADC1); prints analog + WHITE/EDGE/BLACK
   3 = LEFT     desk: one sensor, AOUT -> GP28 (ADC2); prints analog + WHITE/EDGE/BLACK
   4 = ALL      robot wiring, prints all three raw sensors (no line logic) */
#ifndef S3_TEST
#define S3_TEST             0
#endif
#define S3_RUN              (0 == S3_TEST)
#define S3_DESK_TEST        (1 == S3_TEST)
#define S3_USE_BARCODE      ((0 == S3_TEST) || (1 == S3_TEST) || (4 == S3_TEST))

/* RUN mode settings */
/* Physical finding (Tribunal 1 doc): on the real car the OUTER line sensor (GP28)
   is on the robot's RIGHT, so the Centre sensor rides the RIGHT edge of the line
   and junctions are detected on the right.  Barcode sensor is on the left. */
#ifndef S3_EDGE_SIDE
#define S3_EDGE_SIDE        S3_LN_EDGE_LEFT   /* 26 Oct: a car centred on the line has the Centre near the LEFT edge (Week 6 design) */
#endif
#define S3_EDGE_TXT         ((S3_LN_EDGE_RIGHT == S3_EDGE_SIDE) ? "RIGHT" : "LEFT")
#define S3_STEER_MAX_MMS    200     /* steer_sp at full lock (+-1000 permille), mm/s; <= 511 (10-bit) */
#define S3_LINE_PERIOD_MS   10      /* line_task period                                  */
#define S3_ASSUMED_SPEED_MM_S 100   /* used only until Buddy 2 calls s3_set_odometry()   */
#define S3_BC_COOLDOWN_MS   1500    /* ignore further barcodes this long after publishing */
#define S3_RUN_VERBOSE      1       /* 1 = print status every S3_STATUS_MS + every event */
#define S3_STATUS_MS        200     /* status line period, ms (was 500)                  */
#define S3_JUNC_DEBUG       1       /* 1 = print one "S3 JDBG" line per outer-black episode */
#ifndef S3_DRIVE_TEST
#define S3_DRIVE_TEST       0       /* TEST ONLY: 1 = Button 1 starts FOLLOW on Buddy 2's motors */
#endif
#define S3_DRIVE_MMS        150     /* drive test: robot speed, mm/s                       */
#define S3_DRIVE_MAX_MM     3000U   /* drive test: stop after this distance                */
#define S3_DRIVE_MAX_MS     30000U  /* drive test: stop after this time                    */
#define S3_DRIVE_BTN_PIN    20U     /* Robo Pico Button 1 (S5's start button; read only)   */
#define S3_DRIVE_BTN2_PIN   21U     /* Robo Pico Button 2: next gain preset (read only)   */
#define S3_SWEEP_DEG        35      /* drive test self-calibration: turn +-this on the spot */
#define S3_SWEEP_MMS        100     /* drive test self-calibration: wheel speed, mm/s      */
#define S3_BC_VERBOSE_RUN   0       /* RUN: 1 = print every barcode attempt (barcode tuning);
                                       0 = quiet, only "S3 NAV" on a decode (line tuning) */
#define S3_BC_VERBOSE       ((!S3_RUN) || (0 != S3_BC_VERBOSE_RUN))  /* desk tests always print */
#define S3_BC_LOG(...)      do { if (S3_BC_VERBOSE) { tm_printf(__VA_ARGS__); } } while (0)
#define S3_PRI_LINE         16      /* S3 band 16-17 (TEAM.md section 4) */
#define S3_LINE_BLACK_PCT   75      /* line sensors: n > 75 % = BLACK (design doc 6.1) */
#define S3_LINE_WHITE_PCT   25      /* line sensors: n < 25 % = WHITE */
#define S3_LINE_MIN_RANGE   300     /* need this much white-to-black range before classifying */

#if S3_DESK_TEST
#define S3_PIN_BC           26U
#define S3_ADC_BC_CH        1U
#else
#define S3_PIN_BC           6U
#define S3_ADC_BC_CH        0U
#endif

#define S3_ADC_CH_C         1U      /* Centre IR, GP27 */
#define S3_ADC_CH_L         2U      /* outer IR,  GP28 (named "L" in the code; robot's right) */
#define S3_PRI_BRINGUP      17      /* S3 band 16-17 (TEAM.md); in RUN this is the barcode task */

#if (S3_PIN_BC == (26U + S3_ADC_BC_CH))
#error "barcode DOUT and AOUT cannot be on the same pin"
#endif

/* ---------------------------------------------------------------- hardware */
#define S3_INTNO_IO_BANK0   13U     /* RP2040 IRQ 13 = IO_IRQ_BANK0 */
#define S3_INTNO_TIMER1     1U      /* RP2040 IRQ 1  = TIMER_IRQ_1 (alarm 1) */
#define S3_INT_LEVEL        2

#define S3_BC_REG_OFS       (4U * (S3_PIN_BC / 8U))
#define S3_IO_INTR          (IO_BANK0_BASE + 0x0F0U + S3_BC_REG_OFS)
#define S3_IO_PROC1_INTE    (IO_BANK0_BASE + 0x130U + S3_BC_REG_OFS)
#define S3_IO_PROC1_INTS    (IO_BANK0_BASE + 0x150U + S3_BC_REG_OFS)
#define S3_BC_EDGE_BITS     (0xCUL << (4U * (S3_PIN_BC % 8U)))  /* EDGE_LOW|EDGE_HIGH */

#define S3_ADC_BASE         0x4004C000UL
#define S3_ADC_CS           (S3_ADC_BASE + 0x00U)
#define S3_ADC_RESULT       (S3_ADC_BASE + 0x04U)
#define S3_ADC_CS_EN        (1UL << 0)
#define S3_ADC_CS_START     (1UL << 2)
#define S3_ADC_CS_READY     (1UL << 8)
#define S3_ADC_AINSEL_POS   12U

/* ---------------------------------------------------------------- timing */
#define S3_GLITCH_US        2000U   /* digital: flips closer than this = chatter (measured 0.5-0.85 ms) */
#if S3_DESK_TEST
#define S3_SAMPLE_US        1000U   /* hand sweep ~20 mm/s: 1 kHz = 0.02 mm/sample, ring = 16 s */
#define S3_RAW_DUMP         1       /* print the analogue window after each pass */
#else
#define S3_SAMPLE_US        500U    /* robot 0.30 m/s: 2 kHz = 0.15 mm/sample, ring = 2 s */
#define S3_RAW_DUMP         0
#endif
#define S3_LINE_EVERY       (10000U / S3_SAMPLE_US)   /* line sensors every 10 ms */
#define S3_RAW_AVG          5U      /* raw dump: average of 5 samples per value */
/* Analogue history, 8-bit samples (Change Requirement 1).  RUN: 4096 x 0.5 ms = 2 s,
   enough for a 141 mm barcode at >= 0.10 m/s plus 2 x 200 ms margins.
   Desk: 16384 x 1 ms = 16 s for slow hand sweeps. */
#if S3_DESK_TEST
#define S3_RING             16384U
#else
#define S3_RING             4096U
#endif
#define S3_RING_MASK        (S3_RING - 1U)
#define S3_COPY_GUARD       64U     /* clipped window: stay 64 samples ahead of the sampler */
/* Activity = the signal has moved S3_ACT_STEP counts away from where it was at
   the last activity, however slowly it got there (speed-independent; v6 used
   "250 counts within 5 ms", which a hand sweep under ~100 mm/s never trips). */
#define S3_ACT_STEP         12      /* 8-bit units (= 192 of 4096): well above ADC noise */
#if S3_DESK_TEST
#define S3_QUIET_US         1000000U /* hand sweep: a 9 mm wide stripe at 20 mm/s = 450 ms */
#else
#define S3_QUIET_US         300000U  /* robot at 0.30 m/s: whole code = 470 ms */
#endif
#define S3_PRE_SAMPLES      (200000U / S3_SAMPLE_US)  /* keep 200 ms before/after the pass */
#define S3_DIG_RING         64U     /* power of 2; a barcode needs 30 edges */
#define S3_GAP_SHIFT        4U      /* edge gaps stored in 16 us units in 16 bits: <= 1.05 s */

/* ---------------------------------------------------------------- shared state */
/* Change Requirement 1 -- ISR <-> task data, each with ONE writer:
   Digital edges: single-producer/single-consumer ring of 16-bit gaps.
     g_gap[]  written by the GPIO ISR only        (was 64 x 32-bit timestamps)
     g_gap_wr written by the GPIO ISR only (1 byte)
     g_gap_rd written by the barcode task only (1 byte)
   -> no read-modify-write is shared, so no lock and no interrupt masking. */
LOCAL volatile UH g_gap[S3_DIG_RING];
LOCAL volatile UB g_gap_wr;
LOCAL volatile UB g_gap_rd;
LOCAL volatile UB g_glitches;       /* free-running, ISR-only writer; task takes differences */
LOCAL volatile UB g_gap_lost;       /* free-running count of edges dropped (ring full)      */
LOCAL volatile UW g_last_us;        /* time of the last accepted edge (ISR-only writer)      */

LOCAL volatile UB g_ring[S3_RING];  /* 8-bit analogue samples (sampler ISR is the only writer) */
LOCAL volatile UW g_idx;            /* total samples taken (monotonic, compared across wrap)  */
LOCAL volatile UW g_act_first;
LOCAL volatile UW g_act_last;
LOCAL volatile UB g_act;            /* bit0 = window open, bit1 = opened by AOUT, bit2 = by DOUT */
LOCAL volatile UB g_act_ref;        /* 8-bit sample at the last activity */
LOCAL volatile UH g_line_lc;        /* outer IR bits 0-7, Centre IR bits 8-15 (8-bit each):
                                       one aligned 16-bit store, never torn */
#define S3_LINE_L8()  ((UB)(g_line_lc & 0xFFU))
#define S3_LINE_C8()  ((UB)(g_line_lc >> 8))
#define S3_LINE_L()   ((UH)((UH)S3_LINE_L8() << 4))   /* back to the 12-bit scale */
#define S3_LINE_C()   ((UH)((UH)S3_LINE_C8() << 4))
LOCAL volatile UW g_alarm_next;

LOCAL UB g_win[S3_RING];            /* task-side copy of one pass */

/* Mark sample index i as "something is passing".  Called from both ISRs,
   which run on the same core at the same level, so they never nest. */
LOCAL void s3_mark_active (UW i, UB src)
{
	if (0U == (g_act & 1U)) {
		g_act       = 1U;
		g_act_first = i;
	}
	g_act_last = i;
	g_act      = (UB)(g_act | src);
}

/* ---------------------------------------------------------------- ISRs */
LOCAL UH s3_adc_convert (UW ch)
{
	while (0U == (in_w(S3_ADC_CS) & S3_ADC_CS_READY)) {
		;
	}
	out_w(S3_ADC_CS, (ch << S3_ADC_AINSEL_POS) | S3_ADC_CS_EN | S3_ADC_CS_START);
	while (0U == (in_w(S3_ADC_CS) & S3_ADC_CS_READY)) {
		;				/* about 2 us */
	}
	return (UH)(in_w(S3_ADC_RESULT) & 0x0FFFU);
}

/* Digital path: microseconds only, never waits. */
LOCAL void s3_gpio_isr (UINT intno)
{
	UW now = in_w(TIMER_TIMERAWL);
	UW sts = in_w(S3_IO_PROC1_INTS) & S3_BC_EDGE_BITS;

	(void)intno;
	if (0U == sts) {
		return;
	}
	out_w(S3_IO_INTR, sts);

	s3_mark_active(g_idx, 4U);		/* any edge also opens the analogue window */

	{
		UB wr   = g_gap_wr;
		UB next = (UB)((wr + 1U) & (S3_DIG_RING - 1U));
		UW gap  = (now - g_last_us) >> S3_GAP_SHIFT;	/* 16 us units */

		if ((wr != g_gap_rd) && ((now - g_last_us) < S3_GLITCH_US)) {
			g_glitches = (UB)(g_glitches + 1U);	/* chatter: counted, not recorded */
			return;
		}
		if (next == g_gap_rd) {
			g_gap_lost = (UB)(g_gap_lost + 1U);	/* full: keep the older edges */
		} else {
			g_gap[wr] = (UH)((gap > 0xFFFFU) ? 0xFFFFU : gap);
			g_gap_wr  = next;			/* publish AFTER the data */
		}
		g_last_us = now;
	}
}

/* 12-bit ADC count -> 8-bit, rounded to nearest (max error 8 counts = 0.2 %) */
Inline UB s3_to8 (UW adc)
{
	UW v = (adc + 8U) >> 4;
	return (UB)((v > 255U) ? 255U : v);
}

/* Analogue path: 2 kHz sampler on timer alarm 1. */
LOCAL void s3_sampler_isr (UINT intno)
{
	UW  i = g_idx;
	UB  v;
	INT d;

	(void)intno;
	out_w(TIMER_INTR, 1UL << 1);			/* acknowledge alarm 1 */
	g_alarm_next += S3_SAMPLE_US;
	if ((INT)(g_alarm_next - in_w(TIMER_TIMERAWL)) < 20) {
		g_alarm_next = in_w(TIMER_TIMERAWL) + S3_SAMPLE_US;   /* fell behind */
	}
	out_w(TIMER_ALARM1, g_alarm_next);		/* re-arm */

	v = s3_to8(s3_adc_convert(S3_ADC_BC_CH));	/* 12-bit -> 8-bit, rounded */
	g_ring[i & S3_RING_MASK] = v;

	if (0U == i) {
		g_act_ref = v;
	}
	d = (INT)v - (INT)g_act_ref;
	if ((d > S3_ACT_STEP) || (d < -S3_ACT_STEP)) {
		g_act_ref = v;
		s3_mark_active(i, 2U);
	}

	if (0U == (i % S3_LINE_EVERY)) {		/* every 10 ms: line sensors */
		UH l = s3_to8(s3_adc_convert(S3_ADC_CH_L));
		UH c = s3_to8(s3_adc_convert(S3_ADC_CH_C));
		g_line_lc = (UH)(l | (UH)(c << 8));
	}
	g_idx = i + 1U;
}

/* ---------------------------------------------------------------- setup */
LOCAL void s3_analog_pin (UW pin)
{
	clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
	out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
}

LOCAL void s3_pins_init (void)
{
	s3_analog_pin(26U + S3_ADC_CH_C);
	s3_analog_pin(26U + S3_ADC_CH_L);
	s3_analog_pin(26U + S3_ADC_BC_CH);

	(void)gpio_set_pin(S3_PIN_BC, GPIO_MODE_IN);
	clr_w(GPIO(S3_PIN_BC), GPIO_PUE | GPIO_PDE);
	set_w(GPIO(S3_PIN_BC), GPIO_SHEMITT);

	set_w(S3_ADC_CS, S3_ADC_CS_EN);		/* kernel ADC driver already enabled it; harmless */
}

LOCAL ER s3_irq_init (void)
{
	T_DINT dint = {0};
	ER     er;

	/* IO_IRQ_BANK0 is owned by Subsystem 2 (it takes the wheel encoders on
	   the same interrupt). Only one tk_def_int() per interrupt number is
	   allowed, so S3's handler is attached to S2's. Returns once S2's handler
	   is installed; our own pin interrupt is enabled only after that. */
	er = subsystem2_gpio_isr_register(s3_gpio_isr);
	if (er < E_OK) {
		return er;
	}
	if (S3_USE_BARCODE) {			/* line-only tests leave the DOUT pin alone */
		out_w(S3_IO_INTR, S3_BC_EDGE_BITS);
		set_w(S3_IO_PROC1_INTE, S3_BC_EDGE_BITS);
	}

	dint.intatr = TA_HLNG;

	dint.inthdr = (FP)s3_sampler_isr;
	er = tk_def_int(S3_INTNO_TIMER1, &dint);
	if (er < E_OK) {
		return er;
	}
	out_w(TIMER_INTR, 1UL << 1);
	set_w(TIMER_INTE, 1UL << 1);
	EnableInt(S3_INTNO_TIMER1, S3_INT_LEVEL);
	g_alarm_next = in_w(TIMER_TIMERAWL) + S3_SAMPLE_US;
	out_w(TIMER_ALARM1, g_alarm_next);		/* start sampling */
	return E_OK;
}

/* ---------------------------------------------------------------- console pacing */
/* The USB console keeps only 4 KB and silently DROPS bytes when full.  A RAW
   dump is several KB, so wait for the console to drain before each line
   (max ~0.4 s per call, so an unplugged host cannot stall this task). */
#if defined(TM_CONSOLE_USB_CDC) && TM_CONSOLE_USB_CDC && S3_RAW_DUMP
IMPORT UW tm_usb_pending(void);
IMPORT volatile UW tm_usb_dropped;
LOCAL void s3_console_wait (void)
{
	INT n;
	for (n = 0; (n < 200) && (tm_usb_pending() > 1024U); n++) {
		tk_dly_tsk(2);
	}
}
#define S3_CONSOLE_DROPPED()  ((INT)tm_usb_dropped)
#else
#define s3_console_wait()     ((void)0)
#define S3_CONSOLE_DROPPED()  (0)
#endif

/* ---------------------------------------------------------------- one pass */
LOCAL char const * s3_res_txt (s3_c39_result_t r)
{
	switch (r) {
	case S3_C39_OK:            return "ok";
	case S3_C39_BAD_COUNT:     return "wrong stripe count";
	case S3_C39_BAD_RATIO:     return "wide/narrow unclear";
	case S3_C39_BAD_SYMBOL:    return "not a Code 39 letter";
	default:                   return "no * start/stop";
	}
}

LOCAL void s3_print_result (char const * p_path, s3_c39_result_t res, char ch)
{
	if (S3_C39_OK == res) {
		tm_printf((UB *)"%s'%c'\n", p_path, ch);
	} else {
		tm_printf((UB *)"%sfailed (%s)\n", p_path, s3_res_txt(res));
	}
}

/* Decode one finished pass.  Returns TRUE if accepted; then *p_ch, *p_rev
   and *p_src (1 = both agree, 2 = analog only, 3 = digital only) are set. */
LOCAL BOOL s3_process_pass (char *p_ch, UB *p_rev, UB *p_src)
{
	static UW dur[S3_DIG_RING];		/* static: keep the 2 KB task stack free */
	static UW adur[S3_AN_MAX_EXTREMA];
	static UB glitch_seen;			/* free-running counter value at the last pass */
	static UB lost_seen;
	UW   lost;
	UW   n_dig, glitches, n_win, first = 1U, last = 0U, i, m;
	UB   src = 0U, rd, wr;
	BOOL clipped = FALSE;
	UINT imask;
	char ch_d = '?', ch_a = '?';
	UB   rev_d = 0U, rev_a = 0U;
	BOOL ok = FALSE;
	s3_c39_result_t res_d = S3_C39_BAD_COUNT;
	s3_c39_result_t res_a = S3_C39_BAD_COUNT;
	s3_an_stats_t st = {0};

	/* Digital: drain the gap ring lock-free (ISR owns wr, this task owns rd).
	   Gap 0 is the idle time before the first edge; elements are gaps 1.. */
	wr    = g_gap_wr;
	rd    = g_gap_rd;
	n_dig = (UW)((UB)(wr - rd) & (S3_DIG_RING - 1U));
	for (i = 0U; i < n_dig; i++) {
		UW g = g_gap[(rd + i) & (S3_DIG_RING - 1U)];
		if (i > 0U) {
			dur[i - 1U] = g << S3_GAP_SHIFT;	/* back to microseconds */
		}
	}
	g_gap_rd    = wr;				/* hand the slots back to the ISR */
	glitches    = (UB)(g_glitches - glitch_seen);
	glitch_seen = g_glitches;
	lost        = (UB)(g_gap_lost - lost_seen);
	lost_seen   = g_gap_lost;

	/* Analogue: snapshot the window indices with interrupts off (both ISRs run on this core) */
	DI(imask);
	n_win = 0U;
	if (0U != (g_act & 1U)) {
		first = (g_act_first > S3_PRE_SAMPLES) ? (g_act_first - S3_PRE_SAMPLES) : 0U;
		last  = g_act_last + S3_PRE_SAMPLES;
		if (last >= g_idx) {
			last = g_idx - 1U;
		}
		if ((g_idx - first) > (S3_RING - S3_COPY_GUARD)) {	/* oldest part overwritten or about to be */
			first   = g_idx - S3_RING + S3_COPY_GUARD;
			clipped = TRUE;
		}
		src   = (UB)(g_act >> 1);			/* 1 = AOUT, 2 = DOUT, 3 = both */
		g_act = 0U;
	} else {
		first = 1U;
		last  = 0U;			/* empty window */
	}
	EI(imask);

	/* Copy the window with interrupts ON: the sampler keeps writing NEW
	   samples; the window starts at least S3_COPY_GUARD samples (32 ms RUN)
	   ahead of the write position, longer than this copy plus any preemption. */
	for (i = first; (i <= last) && (n_win < S3_RING); i++) {
		g_win[n_win++] = g_ring[i & S3_RING_MASK];
	}

	/* digital path */
	if ((n_dig >= 2U) && (0U == lost)) {	/* edges dropped -> digital path not trusted */
		res_d = s3_code39_decode((uint32_t const *)dur, n_dig - 1U, &ch_d, (uint8_t *)&rev_d);
	}

	/* analogue path */
	m = 0U;
	if (n_win > 0U) {
		m = s3_analog_widths((uint8_t const *)g_win, n_win, S3_SAMPLE_US,
				     (uint32_t *)adur, S3_AN_MAX_EXTREMA, &st);
		res_a = s3_code39_decode((uint32_t const *)adur, m, &ch_a, (uint8_t *)&rev_a);
	}

#if S3_RAW_DUMP
	{	/* raw analogue trace for offline plotting: one value per S3_RAW_AVG samples */
		UW k, j, sum, col = 0U;
		tm_printf((UB *)"S3 RAW begin dt_ms=%d n=%d\n",
			  (INT)((S3_SAMPLE_US * S3_RAW_AVG) / 1000U), (INT)(n_win / S3_RAW_AVG));
		for (k = 0U; (k + S3_RAW_AVG) <= n_win; k += S3_RAW_AVG) {
			sum = 0U;
			for (j = 0U; j < S3_RAW_AVG; j++) {
				sum += g_win[k + j];
			}
			if (0U == col) {
				s3_console_wait();	/* desk test only: RUN never waits on the console */
				tm_printf((UB *)"S3 RAW");
			}
			tm_printf((UB *)" %d", (INT)((sum / S3_RAW_AVG) << 4));	/* 12-bit scale, as before */
			if (++col >= 20U) {
				tm_printf((UB *)"\n");
				col = 0U;
			}
		}
		tm_printf((UB *)"%sS3 RAW end\n", (0U != col) ? "\n" : "");
	}
#endif

#if S3_RAW_DUMP
	s3_console_wait();
#endif
	S3_BC_LOG((UB *)"S3 PASS digital: edges=%d chatter=%d dropped=%d -> ", (INT)n_dig, (INT)glitches, (INT)lost);
	if (S3_BC_VERBOSE) {
		s3_print_result("", res_d, ch_d);
	}
	S3_BC_LOG((UB *)"S3 PASS analog : window=%dms (opened by %s)%s stripes=%d white=%d black=%d P=%d -> ",
		  (INT)((n_win * S3_SAMPLE_US) / 1000U),
		  (3U == src) ? "both" : ((1U == src) ? "AOUT" : ((2U == src) ? "DOUT" : "-")),
		  clipped ? " TOO SLOW, start lost" : "",
		  (INT)m, (INT)st.min << 4, (INT)st.max << 4, (INT)st.prominence << 4);
	if (S3_BC_VERBOSE) {
		s3_print_result("", res_a, ch_a);
	}

	if ((S3_C39_OK == res_d) && (S3_C39_OK == res_a)) {
		if (ch_d == ch_a) {
			S3_BC_LOG((UB *)"S3 RESULT ACCEPT '%c' (both agree)\n", ch_d);
			*p_ch = ch_d; *p_rev = rev_a; *p_src = 1U; ok = TRUE;
		} else {
			S3_BC_LOG((UB *)"S3 RESULT REJECT (digital '%c' vs analog '%c' disagree)\n", ch_d, ch_a);
		}
	} else if (S3_C39_OK == res_a) {
		S3_BC_LOG((UB *)"S3 RESULT ACCEPT '%c' (analog only)\n", ch_a);
		*p_ch = ch_a; *p_rev = rev_a; *p_src = 2U; ok = TRUE;
	} else if (S3_C39_OK == res_d) {
		S3_BC_LOG((UB *)"S3 RESULT ACCEPT '%c' (digital only)\n", ch_d);
		*p_ch = ch_d; *p_rev = rev_d; *p_src = 3U; ok = TRUE;
	} else {
		S3_BC_LOG((UB *)"S3 RESULT REJECT (neither decoded)\n");
	}
	return ok;
}

/* ---------------------------------------------------------------- RUN mode: objects + status */
EXPORT ID	mbf_nav;
EXPORT ID	mtx_s3;
EXPORT ID	flg_s3;
EXPORT ID	flg_line;
LOCAL S3_STATUS		g_status;		/* packed; written by 2 tasks, ALWAYS under mtx_s3 */
LOCAL volatile S3_ODOM_FN g_odom_fn;

/* IF-02 steer_sp: one aligned 32-bit word, single writer (line task):
   bits 0-9 dv (10-bit two's complement, mm/s), bits 10-17 seq, bit 18 valid. */
LOCAL volatile UW	g_steer_word;
#define S3_SP_DV_MASK	0x3FFU
#define S3_SP_SEQ_POS	10U
#define S3_SP_VALID	(1UL << 18)

/* Change Requirement 1: the packed sizes are part of the interface */
typedef char s3_chk_nav[(sizeof(S3_NAV_MSG) == 4U) ? 1 : -1];
typedef char s3_chk_status[(sizeof(S3_STATUS) == 16U) ? 1 : -1];
typedef char s3_chk_line[(sizeof(s3_line_t) <= 40U) ? 1 : -1];

EXPORT ER s3_init (void)
{
	T_CMBF	cmbf = {0};
	T_CMTX	cmtx = {0};
	T_CFLG	cflg = {0};
	char	m[5];

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * (sizeof(S3_NAV_MSG) + 4);	/* 4 messages (+ kernel header each) = 32 B */
	cmbf.maxmsz = sizeof(S3_NAV_MSG);
	mbf_nav = tk_cre_mbf(&cmbf);
	if (mbf_nav < E_OK) {
		return mbf_nav;
	}
	cmtx.mtxatr = TA_INHERIT;
	mtx_s3 = tk_cre_mtx(&cmtx);
	if (mtx_s3 < E_OK) {
		return mtx_s3;
	}
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0U;
	flg_s3 = tk_cre_flg(&cflg);
	if (flg_s3 < E_OK) {
		return flg_s3;
	}
	cflg.iflgptn = LF_ENABLE;			/* steer from power-up until the VC says otherwise */
	flg_line = tk_cre_flg(&cflg);
	if (flg_line < E_OK) {
		return flg_line;
	}
	s3_nav_reset();
	s3_nav_get_map(m);
	g_status.state     = S3_LINE_UNCAL;
	g_status.last_ch   = S3_C39_IDX_NONE;
	g_status.map_left  = s3_code39_index(m[0]);
	g_status.map_right = s3_code39_index(m[1]);
	g_status.map_str   = s3_code39_index(m[2]);
	g_status.map_uturn = s3_code39_index(m[3]);
	return E_OK;
}

EXPORT ER s3_get_line_status (S3_STATUS *p_st)
{
	ER er = tk_loc_mtx(mtx_s3, TMO_FEVR);	/* E_CTX if called from a handler */
	if (E_OK != er) {
		return er;
	}
	*p_st = g_status;
	tk_unl_mtx(mtx_s3);
	return E_OK;
}

EXPORT BOOL s3_get_steer_sp (H *p_dv_mms, UB *p_seq)
{
	UW w  = g_steer_word;			/* ONE load: dv, seq and valid always belong together */
	INT dv = (INT)(w & S3_SP_DV_MASK);
	if (0U != (dv & 0x200)) {
		dv -= 0x400;			/* sign-extend 10 bits */
	}
	*p_dv_mms = (H)dv;
	*p_seq    = (UB)(w >> S3_SP_SEQ_POS);
	return (0U != (w & S3_SP_VALID)) ? TRUE : FALSE;
}

EXPORT char s3_nav_msg_char (const S3_NAV_MSG *p_msg)
{
	return s3_code39_char((UB)p_msg->ch_idx);
}

/* Run-time tuning (IF-S3e): any TASK may request; the line task applies it
   at its next step, so s3_line_t is only ever touched by the line task.
   The request is handed over under mtx_s3, so kp and kd always travel together. */
LOCAL INT		g_req_kp;
LOCAL INT		g_req_kd;
LOCAL BOOL		g_req_gains;
LOCAL volatile BOOL	g_req_recal;

EXPORT void s3_set_line_gains (INT kp_x100, INT kd_x100)
{
	if ((kp_x100 < 0) || (kd_x100 < 0)) {
		return;				/* ignore nonsense, never get stuck */
	}
	if (E_OK == tk_loc_mtx(mtx_s3, TMO_FEVR)) {
		g_req_kp    = kp_x100;
		g_req_kd    = kd_x100;
		g_req_gains = TRUE;
		tk_unl_mtx(mtx_s3);
	}
}

/* IF-S3f: the day's letter map, e.g. "DCBA" (LEFT, RIGHT, STRAIGHT, U-TURN). */
EXPORT ER s3_set_nav_map (const char *p_map)
{
	INT  rc;
	char m[5];
	ER   er = tk_loc_mtx(mtx_s3, TMO_FEVR);

	if (E_OK != er) {
		return er;
	}
	rc = s3_nav_set_map(p_map);
	s3_nav_get_map(m);
	g_status.map_left  = s3_code39_index(m[0]);
	g_status.map_right = s3_code39_index(m[1]);
	g_status.map_str   = s3_code39_index(m[2]);
	g_status.map_uturn = s3_code39_index(m[3]);
	tk_unl_mtx(mtx_s3);
	tm_printf((UB *)"S3 NAV MAP %s: LEFT=%c RIGHT=%c STRAIGHT=%c U-TURN=%c\n",
		  (0 == rc) ? "set" : "REJECTED (kept)", m[0], m[1], m[2], m[3]);
	return (0 == rc) ? E_OK : E_PAR;
}

EXPORT void s3_request_recalibrate (void)
{
	g_req_recal = TRUE;
}

EXPORT void s3_set_odometry (S3_ODOM_FN fn)
{
	g_odom_fn = fn;
}

LOCAL UW s3_now_ms (void)
{
	SYSTIM t;
	tk_get_tim(&t);
	return (UW)t.lo;
}

#if S3_RUN
LOCAL UW s3_odom_mm (void)
{
	S3_ODOM_FN fn = g_odom_fn;
	return (NULL != fn) ? fn() : 0U;
}

/* Barcode accepted -> navigation command (IF-01), status, event flag. */
LOCAL void s3_publish_barcode (char ch, UB rev, UB src)
{
	LOCAL UW	seq;
	LOCAL UW	last_ms;
	LOCAL BOOL	have_last;
	S3_NAV_MSG	msg = {0};
	UW		now = s3_now_ms();
	ER		er;

	if (have_last && ((now - last_ms) < S3_BC_COOLDOWN_MS)) {
		tm_printf((UB *)"S3 NAV ignored '%c' (cooldown)\n", ch);
		return;
	}
	have_last    = TRUE;
	last_ms      = now;
	seq          = (seq + 1U) & 0xFU;
	msg.seq      = seq;
	msg.ch_idx   = s3_code39_index(ch);
	msg.reversed = (0U != rev) ? 1U : 0U;
	msg.source   = src & 0x3U;
	msg.odom_mm  = s3_odom_mm() & 0xFFFFU;

	tk_loc_mtx(mtx_s3, TMO_FEVR);
	msg.cmd      = (UW)s3_nav_from_char(ch);	/* day's map; s3_nav_cmd_t values == S3_CMD values */
	tk_unl_mtx(mtx_s3);

	er = tk_snd_mbf(mbf_nav, &msg, sizeof(msg), TMO_POL);	/* never block the S3 task */
	tk_loc_mtx(mtx_s3, TMO_FEVR);
	g_status.last_ch  = msg.ch_idx;
	g_status.last_cmd = msg.cmd;
	g_status.last_rev = msg.reversed;
	g_status.barcodes = (g_status.barcodes + 1U) & 0xFFU;
	tk_unl_mtx(mtx_s3);
	tk_set_flg(flg_s3, FLG_S3_BARCODE);

	tm_printf((UB *)"S3 NAV #%d '%c' -> %s%s%s\n", (INT)msg.seq, ch,
		  s3_nav_name((s3_nav_cmd_t)msg.cmd), rev ? " (read backwards)" : "",
		  (E_OK == er) ? "" : "  [mbf_nav full: nobody is reading it]");
}

/* A cyclic handler wakes the line task every S3_LINE_PERIOD_MS, so the period
   is fixed (tk_dly_tsk(10) would give 10 ms + run time + one tick = ~20 ms). */
LOCAL ID	g_tid_line;
LOCAL ID	g_cyc_line;

LOCAL void s3_line_cyclic (void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(g_tid_line);
}

LOCAL s3_ln_mode_t s3_mode_from_flags (UINT ptn)
{
	if (0U != (ptn & LF_SEARCH_L)) {
		return S3_LN_MODE_SEARCH_FROM_LEFT;
	}
	if (0U != (ptn & LF_SEARCH_R)) {
		return S3_LN_MODE_SEARCH_FROM_RIGHT;
	}
	return (0U != (ptn & LF_ENABLE)) ? S3_LN_MODE_FOLLOW : S3_LN_MODE_DISABLED;
}

LOCAL char const * s3_state_txt (UW st, UB overshoot)
{
	switch (st) {
	case S3_LINE_FOLLOW:   return overshoot ? "OVERSHOOT" : "FOLLOW";
	case S3_LINE_JUNCTION: return "JUNCTION";
	case S3_LINE_LOST:     return "LOST";
	case S3_LINE_DISABLED: return "DISABLED (LF_ENABLE clear)";
	case S3_LINE_SEARCH:   return "SEARCH";
	default:               return "UNCALIBRATED (sweep both sensors over white and black)";
	}
}

/* What one line sensor sees, using the same bands as s3_line.c. */
LOCAL char const * s3_zone_txt (INT n_permille)
{
	if (n_permille < S3_LN_WHITE) {
		return "WHITE";
	}
	if (n_permille > S3_LN_BLACK) {
		return "BLACK";
	}
	return "EDGE ";
}

/* line_task: every 10 ms -- calibration, PD steering, junction, line lost. */
LOCAL void s3_line_task (INT stacd, void *exinf)
{
	LOCAL s3_line_t	ln;
	s3_line_out_t	o;
	T_RFLG		rflg;
	UW		t_prev = in_w(TIMER_TIMERAWL);
	UW		t_now, dt_us, odom_prev = 0U, odom_now, moved_um;
	UW		relax_ms = 0U, print_ms = 0U, st, seq = 0U;
	BOOL		jd_outer_black = FALSE;	/* S3_JUNC_DEBUG: outer sensor black last step */
	UW		jd_both_max_um = 0U;
	UW		jd_c_before_um = 0U;
	INT		dv;
	BOOL		odom_started = FALSE;
	S3_ODOM_FN	fn;

	(void)stacd; (void)exinf;
	s3_line_init(&ln, S3_EDGE_SIDE);

	while (1) {
		(void)tk_slp_tsk(TMO_FEVR);		/* woken by s3_line_cyclic */
		(void)tk_can_wup(TSK_SELF);		/* if we overran, do not run twice in a row */
		t_now  = in_w(TIMER_TIMERAWL);
		dt_us  = t_now - t_prev;
		t_prev = t_now;

		/* distance moved this step: odometry if Buddy 2 has given it, else assumed speed */
		fn = g_odom_fn;
		if (NULL != fn) {
			INT d_mm;
			odom_now  = fn();
			d_mm      = (INT)(odom_now - odom_prev);	/* signed: reversing / reset gives <= 0 */
			moved_um  = (odom_started && (d_mm > 0) && (d_mm < 1000)) ? ((UW)d_mm * 1000U) : 0U;
			odom_prev = odom_now;
			odom_started = TRUE;
		} else {
			moved_um = (S3_ASSUMED_SPEED_MM_S * dt_us) / 1000U;
		}

		/* IF-07: what the Vehicle Controller wants (enable / search side) */
		if (E_OK == tk_ref_flg(flg_line, &rflg)) {
			/* acts only when the REQUESTED mode changes (not the raw pattern),
			   so a found line is not re-searched while LF_SEARCH_x is still set */
			s3_line_request_mode(&ln, s3_mode_from_flags(rflg.flgptn));
		}

		{
			BOOL take = FALSE;
			INT  kp = 0, kd = 0;
			tk_loc_mtx(mtx_s3, TMO_FEVR);
			if (g_req_gains) {
				take = TRUE; kp = g_req_kp; kd = g_req_kd; g_req_gains = FALSE;
			}
			tk_unl_mtx(mtx_s3);
			if (take) {
				s3_line_set_gains(&ln, kp, kd);
				tm_printf((UB *)"S3 TUNE kp=%d kd=%d (x100)\n", (INT)ln.kp_x100, (INT)ln.kd_x100);
			}
		}
		if (g_req_recal) {
			g_req_recal = FALSE;
			s3_line_recalibrate(&ln);
			tm_printf((UB *)"S3 RECALIBRATE: sweep both sensors over white and black\n");
		}

		{
			UH lc = g_line_lc;		/* one 16-bit read: both sensors from the same sample */
			s3_line_step(&ln, (UH)((lc & 0xFFU) << 4), (UH)((lc >> 8) << 4), moved_um, dt_us, &o);
		}

		/* Calibration can always WIDEN (s3_cal_learn above), but it only
		   SHRINKS while line following is off (turn / bypass / search).
		   While following, the Centre rides the edge and never sees pure
		   white, so relaxing then dragged "white" up towards the edge value
		   (bench log 26 Oct: 581 -> 1199 counts in ~4 s). */
		relax_ms += dt_us / 1000U;
		if (relax_ms >= 1000U) {
			relax_ms = 0U;
			if ((S3_LN_DISABLED == o.state) || (S3_LN_SEARCH == o.state)) {
				s3_cal_relax(&ln.cal_l);
				s3_cal_relax(&ln.cal_c);
			}
		}

		/* IF-02 steer_sp: permille -> mm/s, 10-bit signed, one atomic store */
		st  = (0U == o.calibrated) ? S3_LINE_UNCAL : (UW)o.state;
		dv  = (o.steer * S3_STEER_MAX_MMS) / 1000;
		dv  = (dv > 511) ? 511 : ((dv < -511) ? -511 : dv);
		seq = (seq + 1U) & 0xFFU;
		g_steer_word = ((UW)dv & S3_SP_DV_MASK) | (seq << S3_SP_SEQ_POS) |
			       (((S3_LINE_FOLLOW == st) || (S3_LINE_JUNCTION == st) || (S3_LINE_LOST == st)) ? S3_SP_VALID : 0U);

		tk_loc_mtx(mtx_s3, TMO_FEVR);	/* packed status: the barcode task writes it too */
		g_status.steer_dv   = dv;
		g_status.pos        = o.pos;
		g_status.state      = st;
		g_status.overshoot  = o.overshoot;
		g_status.n_outer    = (UW)((o.n_l * 255) / 1000);
		g_status.n_centre   = (UW)((o.n_c * 255) / 1000);
		g_status.seq        = seq;
		g_status.junctions  = (g_status.junctions + o.junction_event) & 0xFFU;
		g_status.losts      = (g_status.losts + o.lost_event) & 0xFFU;
		g_status.updated_ms = s3_now_ms() & 0xFFFFU;
		tk_unl_mtx(mtx_s3);

		if (0U != o.junction_event) {
			tk_set_flg(flg_s3, FLG_S3_JUNCTION);
		}
		if (0U != o.lost_event) {
			tk_set_flg(flg_s3, FLG_S3_LOST);
		}
		if (0U != o.line_found_event) {
			tk_set_flg(flg_s3, FLG_S3_LINE_FOUND);
		}
#if S3_JUNC_DEBUG
		/* One line per pass of the outer sensor over black, to see how close
		   each junction attempt came to the rules in s3_line.c:
		   Centre black at most S3_LN_JUNC_LEAD_MM before, both black >= S3_LN_JUNC_MM. */
		if ((0U == o.calibrated) || ((S3_LN_FOLLOW != o.state) && (S3_LN_JUNCTION != o.state))) {
			jd_outer_black = FALSE;		/* report only while really tracking (no noise when uncalibrated) */
		} else if (o.n_l > S3_LN_BLACK) {
			if (!jd_outer_black) {
				jd_outer_black = TRUE;
				jd_both_max_um = 0U;
				jd_c_before_um = ln.c_black_at_l;	/* 0xFFFF = Centre was white */
			}
			if (ln.both_black_um > jd_both_max_um) {
				jd_both_max_um = ln.both_black_um;
			}
		} else if (jd_outer_black) {
			jd_outer_black = FALSE;
			if (0xFFFFU == jd_c_before_um) {
				tm_printf((UB *)"S3 JDBG outer black: Centre was WHITE -> overshoot, not a junction\n");
			} else {
				tm_printf((UB *)"S3 JDBG outer black: Centre black %d mm before (max %d), both black %d mm (need %d)\n",
					  (INT)(jd_c_before_um / 1000U), (INT)S3_LN_JUNC_LEAD_MM,
					  (INT)(jd_both_max_um / 1000U), (INT)S3_LN_JUNC_MM);
			}
		}
#endif
#if S3_RUN_VERBOSE
		if (0U != o.junction_event) {
			tm_printf((UB *)"S3 EVENT JUNCTION\n");
		}
		if (0U != o.lost_event) {
			tm_printf((UB *)"S3 EVENT LINE LOST\n");
		}
		if (0U != o.line_found_event) {
			tm_printf((UB *)"S3 EVENT LINE FOUND\n");
		}
		print_ms += dt_us / 1000U;
		if (print_ms >= S3_STATUS_MS) {
			print_ms = 0U;
			/* e.g.  S3 LINE  C=EDGE   512  outer=WHITE   12  -> turn RIGHT    40 mm/s  FOLLOW   cal C 150-2500  O 130-3300 */
			tm_printf((UB *)"S3 LINE  C=%s %4d  outer=%s %4d  -> turn %s %4d mm/s  %s   cal C %d-%d  O %d-%d\n",
				  s3_zone_txt((INT)o.n_c), (INT)o.n_c, s3_zone_txt((INT)o.n_l), (INT)o.n_l,
				  (dv > 0) ? "RIGHT   " : ((dv < 0) ? "LEFT    " : "STRAIGHT"), (dv < 0) ? -dv : dv,
				  s3_state_txt(st, o.overshoot),
				  (INT)ln.cal_c.white, (INT)ln.cal_c.black, (INT)ln.cal_l.white, (INT)ln.cal_l.black);
		}
#else
		(void)print_ms;
#endif
	}
}
#endif /* S3_RUN */

/* ---------------------------------------------------------------- task */
/* Line-sensor test print: raw value, auto-calibrated n (0 % white .. 100 % black), state.
   Calibration = lowest and highest value seen since reset, so sweep the sensor
   over white paper AND the black line once before trusting the state. */
LOCAL void s3_print_line (char const * p_name, UH v, UH * p_lo, UH * p_hi)
{
	INT range, n;

	*p_lo = (v < *p_lo) ? v : *p_lo;
	*p_hi = (v > *p_hi) ? v : *p_hi;
	range = (INT)*p_hi - (INT)*p_lo;
	if (range < S3_LINE_MIN_RANGE) {
		tm_printf((UB *)"%s analog=%4d  (calibrating: move over white AND black, range=%d)\n",
			  p_name, (INT)v, range);
		return;
	}
	n = (((INT)v - (INT)*p_lo) * 100) / range;
	tm_printf((UB *)"%s analog=%4d  n=%3d%%  %s   (white=%d black=%d)\n", p_name, (INT)v, n,
		  (n > S3_LINE_BLACK_PCT) ? "BLACK" : ((n < S3_LINE_WHITE_PCT) ? "WHITE" : "EDGE "),
		  (INT)*p_lo, (INT)*p_hi);
}

#if S3_RUN && S3_DRIVE_TEST
/* TEST ONLY (S3_DRIVE_TEST 1): line following on Buddy 2's motors before the
   Vehicle Controller exists.  Button 1 (GP20, read only with the pad pull-up,
   as in Buddy 2's subsystem2_step.c) starts FOLLOW at S3_DRIVE_MMS.  The car
   stops on Button 1 again, on LINE LOST, or at the distance / time limit. */
LOCAL BOOL s3_drive_btn_down (void)
{
	return (0U == gpio_get_val(S3_DRIVE_BTN_PIN)) ? TRUE : FALSE;
}

LOCAL BOOL s3_drive_btn2_down (void)
{
	return (0U == gpio_get_val(S3_DRIVE_BTN2_PIN)) ? TRUE : FALSE;
}

/* Steering gain presets for tuning on the floor without re-flashing:
   Button 2 moves to the next one while the car is waiting. */
LOCAL const H g_drive_gains[][2] = {	/* {Kp, Kd} x100 */
	{150, 400}, {100, 300}, {75, 200}, {50, 100}, {100, 0}
};
#define S3_DRIVE_N_GAINS    (sizeof(g_drive_gains) / sizeof(g_drive_gains[0]))

LOCAL UW g_drive_preset = 1U;		/* start at {100, 300}: the default weaved (26 Oct) */

/* Wait for Button 1; every Button 2 press selects the next gain preset. */
LOCAL void s3_drive_wait_press (void)
{
	UW   held = 0U;
	BOOL b2_was = FALSE;
	BOOL b2;

	while (s3_drive_btn_down()) {		/* let go of the last press first */
		tk_dly_tsk(20);
	}
	while (held < 60U) {			/* 60 ms held = a real press */
		tk_dly_tsk(20);
		held = s3_drive_btn_down() ? (held + 20U) : 0U;
		b2 = s3_drive_btn2_down();
		if (b2 && !b2_was) {
			g_drive_preset = (g_drive_preset + 1U) % S3_DRIVE_N_GAINS;
			s3_set_line_gains(g_drive_gains[g_drive_preset][0], g_drive_gains[g_drive_preset][1]);
			tm_printf((UB *)"S3 DRIVE GAINS preset %d: kp=%d kd=%d (x100)\n", (INT)g_drive_preset,
				  (INT)g_drive_gains[g_drive_preset][0], (INT)g_drive_gains[g_drive_preset][1]);
		}
		b2_was = b2;
	}
	while (s3_drive_btn_down()) {
		tk_dly_tsk(20);
	}
}

/* One on-the-spot turn with Buddy 2's TURN command; waits until it ends. */
LOCAL BOOL s3_drive_turn (INT deg)
{
	subsystem2_status_t ss;
	UW ms = 0U;

	if (E_OK != subsystem2_turn((int16_t)deg, (uint16_t)S3_SWEEP_MMS)) {
		return FALSE;
	}
	tk_dly_tsk(60);				/* motion_task picks it up in its next 20 ms cycle */
	do {
		tk_dly_tsk(20);
		ms += 20U;
		subsystem2_status_get(&ss);
	} while ((S2_STATE_TURNING == ss.state) && (ms < 4000U));
	return (S2_STATE_TURNING != ss.state) ? TRUE : FALSE;
}

/* Self-calibration (Week 6 CALIBRATING "IR sweep"): swing the sensors across
   the line, left then right, and back.  s3_cal_learn() widens the white/black
   range from whatever the sensors pass over. */
LOCAL void s3_drive_sweep (void)
{
	S3_STATUS st = {0};
	BOOL ok;

	tm_printf((UB *)"S3 DRIVE SWEEP: turning +-%d deg on the spot to calibrate. Car must be on the line.\n", (INT)S3_SWEEP_DEG);
	ok = s3_drive_turn(S3_SWEEP_DEG);
	ok = ok && s3_drive_turn(-2 * S3_SWEEP_DEG);
	ok = ok && s3_drive_turn(S3_SWEEP_DEG);
	(void)subsystem2_stop();
	(void)s3_get_line_status(&st);
	tm_printf((UB *)"S3 DRIVE SWEEP %s: line state %s\n", ok ? "done" : "TURN FAILED",
		  s3_state_txt((UW)st.state, 0U));
}

LOCAL void s3_drive_task (INT stacd, void *exinf)
{
	S3_STATUS	st = {0};
	UW		start_mm, moved_mm, ms, i;
	char const	*why;
	ER		er;

	(void)stacd; (void)exinf;
	(void)gpio_set_pin(S3_DRIVE_BTN_PIN, GPIO_MODE_IN);
	clr_w(GPIO(S3_DRIVE_BTN_PIN), GPIO_PDE);
	set_w(GPIO(S3_DRIVE_BTN_PIN), GPIO_PUE);
	(void)gpio_set_pin(S3_DRIVE_BTN2_PIN, GPIO_MODE_IN);
	clr_w(GPIO(S3_DRIVE_BTN2_PIN), GPIO_PDE);
	set_w(GPIO(S3_DRIVE_BTN2_PIN), GPIO_PUE);
	tk_dly_tsk(100);
	s3_set_line_gains(g_drive_gains[g_drive_preset][0], g_drive_gains[g_drive_preset][1]);
	tm_printf((UB *)"S3 DRIVE GAINS preset %d: kp=%d kd=%d (x100)\n", (INT)g_drive_preset,
		  (INT)g_drive_gains[g_drive_preset][0], (INT)g_drive_gains[g_drive_preset][1]);

	while (1) {
		tm_printf((UB *)"S3 DRIVE: BUTTON 1 = %s. Centre on the %s edge (C=EDGE outer=WHITE) to drive. BUTTON 2 = next gain preset.\n",
			  (E_OK == s3_get_line_status(&st) && (S3_LINE_UNCAL == st.state)) ? "self-calibrate (car on the line)" : "drive",
			  S3_EDGE_TXT);
		s3_drive_wait_press();
		if ((E_OK == s3_get_line_status(&st)) && (S3_LINE_UNCAL == st.state)) {
			s3_drive_sweep();		/* first press after power-up: calibrate, then wait again */
			continue;
		}
		for (i = 3U; i > 0U; i--) {
			tm_printf((UB *)"S3 DRIVE %d\n", (INT)i);
			tk_dly_tsk(1000);
		}
		/* Only start from a calibrated sensor that is on the line: an
		   UNCALIBRATED car gets no steering (drives blind), and a LOST car
		   keeps its last correction (spins on the spot). */
		if ((E_OK != s3_get_line_status(&st)) || (S3_LINE_FOLLOW != st.state)) {
			tm_printf((UB *)"S3 DRIVE NOT STARTED: line state is %s. Sweep the car sideways across the line until it is calibrated, then put the Centre on the %s edge.\n",
				  s3_state_txt((UW)st.state, 0U), S3_EDGE_TXT);
			continue;
		}
		/* n_centre is 0..255: EDGE band 25..75 % = 64..191 */
		if ((st.n_centre < 64U) || (st.n_centre > 191U)) {
			tm_printf((UB *)"S3 DRIVE NOT STARTED: Centre is not on the edge (C=%d of 1000). Put the Centre on the %s edge so the log shows C=EDGE.\n",
				  (INT)((st.n_centre * 1000U) / 255U), S3_EDGE_TXT);
			continue;
		}
		start_mm = s3_odom_mm();
		moved_mm = 0U;
		ms       = 0U;
		why      = "?";
		er = subsystem2_follow((uint16_t)S3_DRIVE_MMS);
		tm_printf((UB *)"S3 DRIVE GO %d mm/s (er=%d). Press BUTTON 1 to stop.\n", (INT)S3_DRIVE_MMS, (INT)er);
		while (1) {
			tk_dly_tsk(20);
			ms += 20U;
			moved_mm = s3_odom_mm() - start_mm;	/* reversing wraps huge -> distance stop */
			if (s3_drive_btn_down()) {
				why = "button";
				break;
			}
			if ((E_OK == s3_get_line_status(&st)) && (S3_LINE_LOST == st.state)) {
				why = "LINE LOST";
				break;
			}
			if (moved_mm >= S3_DRIVE_MAX_MM) {
				why = "distance limit";
				break;
			}
			if (ms >= S3_DRIVE_MAX_MS) {
				why = "time limit";
				break;
			}
		}
		(void)subsystem2_stop();
		tm_printf((UB *)"S3 DRIVE STOP (%s) after %d mm, %d ms\n", why, (INT)moved_mm, (INT)ms);
		tk_dly_tsk(500);
	}
}
#endif

LOCAL void s3_bringup_task (INT stacd, void *exinf)
{
	ER   er;
	UW   tick = 0U;
	UW   now;
	BOOL dig_busy;
	BOOL ana_busy;

	(void)stacd; (void)exinf;

	s3_pins_init();
	er = s3_irq_init();				/* runs on core 1 */
	CHECK(er, "s3_irq_init");

	(void)s3_print_line;
#if (1 == S3_TEST)
	tm_printf((UB *)"S3 TEST BARCODE: DOUT -> GP%d (digital), AOUT -> GP27 (analog)\n", (INT)S3_PIN_BC);
#elif (2 == S3_TEST)
	tm_printf((UB *)"S3 TEST CENTRE: AOUT -> GP27 (ADC1)\n");
#elif (3 == S3_TEST)
	tm_printf((UB *)"S3 TEST LEFT: AOUT -> GP28 (ADC2)\n");
#elif S3_RUN
	tm_printf((UB *)"S3 RUN: barcode DOUT=GP%d AOUT=ADC%d, Left=ADC2 (GP28), Centre=ADC1 (GP27)\n",
		  (INT)S3_PIN_BC, (INT)S3_ADC_BC_CH);
	{
		T_CCYC ccyc = {0};
		g_tid_line = app_start_task(s3_line_task, S3_PRI_LINE, TP_PRC2, "s3_line_task");
		CHECK(g_tid_line, "s3_line_task");
		ccyc.cycatr = TA_HLNG | TA_STA;
		ccyc.cychdr = (FP)s3_line_cyclic;
		ccyc.cyctim = S3_LINE_PERIOD_MS;
		ccyc.cycphs = S3_LINE_PERIOD_MS;
		g_cyc_line  = tk_cre_cyc(&ccyc);
		CHECK(g_cyc_line, "cyc_line");
#if S3_DRIVE_TEST
		CHECK(app_start_task(s3_drive_task, S3_PRI_BRINGUP, TP_PRC1, "s3_drive_task"), "s3_drive_task");
#endif
	}
#else
	tm_printf((UB *)"S3 ALL (robot wiring): barcode DOUT=GP%d AOUT=ADC%d, Left=ADC2, Centre=ADC1\n",
		  (INT)S3_PIN_BC, (INT)S3_ADC_BC_CH);
#endif

	while (1) {
		tk_dly_tsk(50);
		tick++;

#if (1 == S3_TEST)
		if (0U == (tick % 4U)) {		/* every 200 ms */
			INT dv = (INT)gpio_get_val(S3_PIN_BC);
			UW  q  = (0U != (g_act & 1U)) ? (g_idx - g_act_last) : 0U;
			tm_printf((UB *)"BARCODE analog=%4d  digital=%d (%s)  | pass: edges=%d open=%d quiet=%dms lost_chars=%d\n",
				  (INT)g_ring[(g_idx - 1U) & S3_RING_MASK] << 4, dv, dv ? "BLACK" : "WHITE",
				  (INT)((UB)(g_gap_wr - g_gap_rd) & (S3_DIG_RING - 1U)), (INT)(g_act & 1U),
				  (INT)((q * S3_SAMPLE_US) / 1000U),
				  S3_CONSOLE_DROPPED());
		}
#elif (2 == S3_TEST)
		if (0U == (tick % 2U)) {		/* every 100 ms */
			static UH lo = 0xFFFFU, hi = 0U;
			s3_print_line("CENTRE", S3_LINE_C(), &lo, &hi);
		}
#elif (3 == S3_TEST)
		if (0U == (tick % 2U)) {
			static UH lo = 0xFFFFU, hi = 0U;
			s3_print_line("LEFT", S3_LINE_L(), &lo, &hi);
		}
#elif (4 == S3_TEST)
		if (0U == (tick % 4U)) {
			tm_printf((UB *)"S3 IR left=%4d centre=%4d bc_dout=%d\n",
				  (INT)S3_LINE_L(), (INT)S3_LINE_C(), (INT)gpio_get_val(S3_PIN_BC));
		}
#endif
		if (!S3_USE_BARCODE) {
			continue;			/* line-only test: no barcode passes */
		}

		now      = in_w(TIMER_TIMERAWL);
		{
			BOOL dig_open = (g_gap_wr != g_gap_rd);
			BOOL ana_open = (0U != (g_act & 1U));
			dig_busy = dig_open && ((now - g_last_us) <= S3_QUIET_US);
			ana_busy = ana_open && ((g_idx - g_act_last) <= (S3_QUIET_US / S3_SAMPLE_US));
			if (!((dig_open || ana_open) && !dig_busy && !ana_busy)) {
				continue;
			}
		}
		{
			char ch = '?';
			UB   rev = 0U, src = 0U;
			if (s3_process_pass(&ch, &rev, &src)) {
#if S3_RUN
				s3_publish_barcode(ch, rev, src);
#else
				(void)rev; (void)src;
#endif
			}
		}
	}
}

EXPORT ER s3_bringup_start (void)
{
	ID tid = app_start_task(s3_bringup_task, S3_PRI_BRINGUP, TP_PRC2,
				"s3_bringup_task");
	return (tid < E_OK) ? tid : E_OK;
}
