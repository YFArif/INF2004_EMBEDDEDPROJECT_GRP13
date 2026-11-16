/** @file s5_profile.h
 *
 * @brief Obstacle profile: turns ultrasonic scan points into the
 *        obstacle's closest point, width, side clearances and the
 *        sideways shift the robot needs to pass it.
 *
 * @par Robot frame
 * The origin is the servo pivot. x is sideways, positive to the robot's
 * right; y is straight ahead. A servo angle of 90 degrees looks ahead,
 * 0 degrees looks right and 180 degrees looks left, so a point at angle
 * a and range d is x = d * cos(a), y = d * sin(a) (Week 6 report 6.4).
 *
 * @par
 * Pure computation: no kernel or hardware calls, so the module can be
 * tested on a PC (S5-U06) as well as on the robot.
 */

#ifndef S5_PROFILE_H
#define S5_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/* Chassis geometry, from the team's measurements (2026-11-05):
 * - Width: S2's track, tyre centre to centre, 112 mm (subsystem2.h),
 *   plus one tyre width (~26 mm, not measured): about 140 mm.
 * - Axle to front: drive axle (rear) to the front ball caster 77.5 mm,
 *   ruler (S4, imu_task.c WHEELBASE_MM), plus the ball and the bumper.
 * - Setback: from log 2026-11-04_s5_stop_2 (sensor read 121 mm with the
 *   front 30 mm from the box).
 * The margin was raised to 100 mm while the width was guessed at 180 mm
 * (log 2026-11-04_s5_avoid_3); with the measured width it is 50 mm
 * again, and the side gap check (s5_avoid.c) eases out if too close.
 * TODO: ruler check of the outside width and the setback (S5-U04).
 */
#define S5_ROBOT_WIDTH_MM       (140)   /* Widest point, wheels included.*/
#define S5_ROBOT_LENGTH_MM      (200)   /* Front bumper to rear.         */
#define S5_AXLE_TO_FRONT_MM     (160)   /* Rear wheel centre to the front*/
                                        /* tip, measured 15-17 cm        */
                                        /* (2026-11-06).                 */
#define S5_SENSOR_SETBACK_MM    (75)    /* Sensor face behind the front, */
                                        /* measured 7-8 cm (2026-11-05). */
#define S5_PATH_MARGIN_MM       (70)    /* Gap kept from the obstacle.   */
                                        /* 50 hit a box after a 5 degree */
                                        /* drift (S4 heading, log        */
                                        /* 2026-11-11_1732_track_2); 80  */
                                        /* cleared but went wide (logs   */
                                        /* 2026-11-11_1744/1746); 60     */
                                        /* touched (log 1752).           */

/* S3's sensors (Anna, 2026-11-13, measured from the front of the rear
 * wheel; the wheel's radius is 33 mm, S2's 66.5 mm rolling diameter):
 * - The barcode sensor sticks out 55 mm beyond the body on the robot's
 *   left, low enough to hit a box, 40 mm ahead of the wheel's front.
 * - The line sensors are about 100 mm ahead of the wheel's front.
 */
#define S5_SIDE_EXTRA_LEFT_MM   (55)    /* Beyond the body, left side.   */
#define S5_SIDE_EXTRA_RIGHT_MM  (0)     /* Nothing sticks out, right.    */
#define S5_SIDE_EXTRA_AHEAD_MM  (73)    /* Its tip ahead of the rear     */
                                        /* axle: 33 + 40.                */
#define S5_AXLE_TO_LINE_IR_MM   (133)   /* Line sensors ahead of the rear*/
                                        /* axle: 33 + 100.               */

/* The look while passing (s5_avoid.c, Anna 2026-11-13): a flat side
 * slanting more than about 20 degrees from square sends no echo back, so
 * the sensor at its 40 degree servo limit never hears a box's side from
 * a parallel robot (logs 2026-11-12 and 2026-11-13_1417). The robot
 * turns S5_LOOK_DEG towards the obstacle to look (beam 75 degrees off
 * the track). It passes far enough out (s5_profile_pass_mm()) that the
 * part swinging nearest to the obstacle in that turn still stays
 * S5_LOOK_SPARE_MM from its side.
 */
#define S5_LOOK_DEG             (35)
#define S5_LOOK_SPARE_MM        (15)

/* Scan thresholds (Week 6 report 6.4, preliminary until S5-U04). */
#define S5_HIT_RANGE_MM         (400)   /* Closer than this is a hit.    */
#define S5_EDGE_JUMP_MM         (100)   /* Larger range step is an edge. */
#define S5_EDGE_CORR_DEG        (6)     /* Edge correction, A16.         */
#define S5_CLEAR_DEPTH_MM       (300)   /* Look this far past the edge.  */
#define S5_CLEAR_OPEN_MM        (1000)  /* Clearance with nothing seen.  */
#define S5_EDGE_UNSEEN_MM       (150)   /* Edge beyond the servo: assumed*/
                                        /* this far past the last echo   */
                                        /* (80 was 5 cm short, log       */
                                        /* 2026-11-13_1549).             */

/* Range value of a scan point at which nothing answered. */
#define S5_PROFILE_NO_ECHO      (0xFFFFu)

/* Packed profile fields (minimum-bit requirement, Tribunal 1).
 * - Distances: 10 bits, whole mm, 0 to 1023. Every value is clamped
 *   before it is stored; 1023 means "1023 mm or more" (nothing near).
 *   Sensor accuracy is +/-1 cm (NFR7), so whole mm loses nothing.
 * - Angle: 7 bits, the servo angle minus S5_PROFILE_ANGLE_MIN_DEG, in
 *   1 degree steps. The servo only reaches 50 ... 130 degrees (a box on
 *   the robot stops it, drv_servo.h), so 81 values. Not the Tribunal's
 *   5-bit 7.5 degree step: the fine sweep finds edges to 1 degree and
 *   squaring up acts on an 8 degree offset. The 5-bit step is kept on
 *   the MQTT wire only (S1, angle_idx).
 * - seq: 5 bits, wrapping. Readers only check whether it changed, and a
 *   profile takes seconds to build, so 32 values never alias.
 * Each assignment masks the clamped value (& S5_PROFILE_xxx_MASK): the
 * mask changes nothing, but it shows the compiler the value fits the
 * field, so -Wconversion stays quiet.
 */
#define S5_PROFILE_MM_BITS      (10u)
#define S5_PROFILE_MM_MAX       (1023)
#define S5_PROFILE_MM_MASK      (0x3FFu)
#define S5_PROFILE_STEP_BITS    (7u)
#define S5_PROFILE_STEP_MASK    (0x7Fu)
#define S5_PROFILE_ANGLE_MIN_DEG (50)   /* DRV_SERVO_RIGHT_LIMIT_DEG.    */
#define S5_PROFILE_ANGLE_MAX_DEG (130)  /* DRV_SERVO_LEFT_LIMIT_DEG.     */
#define S5_PROFILE_SEQ_BITS     (5u)
#define S5_PROFILE_SEQ_MASK     (0x1Fu)
#define S5_PROFILE_BYTES        (12u)   /* Was 26 B before packing.      */

/** One ultrasonic reading at one servo angle. */
typedef struct
{
    int16_t     angle_deg;      /* Servo angle, 0 (right) to 180 (left). */
    uint16_t    dist_mm;        /* Range, or S5_PROFILE_NO_ECHO.         */
} s5_profile_point_t;

/** One side edge of the obstacle, as found by a fine sweep. */
typedef struct
{
    bool                b_found;    /* False: sweep reached servo limit. */
    s5_profile_point_t  last_hit;   /* Outermost point on the obstacle.  */
} s5_profile_edge_t;

/** The obstacle profile (interface IF-09), packed into three 32-bit
 *  words: 96 bits, 12 B. Distances are in mm (see "Packed profile
 *  fields" above). Read the angle with s5_profile_angle(); never take
 *  the address of a field. The field order fills each word exactly, so
 *  the compiler adds no padding (checked in s5_profile.c).
 */
typedef struct
{
    /* Word 1. */
    unsigned int closest_mm     : 10;   /* Nearest point of the obstacle.*/
    unsigned int width_mm       : 10;   /* Edge to edge, beam corrected. */
    unsigned int clear_left_mm  : 10;   /* Free gap beside the left edge.*/
    unsigned int b_valid        : 1;    /* False until the first scan.   */
    unsigned int b_on_path      : 1;    /* Something blocks the path.    */
    /* Word 2. */
    unsigned int clear_right_mm : 10;   /* Free gap beside right edge.   */
    unsigned int shift_left_mm  : 10;   /* Sideways move to pass left;   */
                                        /* without the edge, assumed.    */
    unsigned int shift_right_mm : 10;   /* Sideways move to pass right.  */
    unsigned int b_left_edge    : 1;    /* Left edge inside servo range. */
    unsigned int b_right_edge   : 1;    /* Right edge inside servo range.*/
    /* Word 3. */
    unsigned int depth_mm       : 10;   /* Length along track; 0 unknown.*/
    unsigned int pass_ahead_mm  : 10;   /* Side watch: obstacle's end    */
                                        /* still this far ahead of the   */
                                        /* sensor when passed.           */
    unsigned int closest_step   : 7;    /* Angle of the nearest point,   */
                                        /* minus ANGLE_MIN_DEG.          */
    unsigned int seq            : 5;    /* Profiles built, wrapping.     */
} s5_profile_t;

bool s5_profile_is_hit(s5_profile_point_t const * p_point);
bool s5_profile_same_object(uint16_t prev_mm,
                            s5_profile_point_t const * p_next);
void s5_profile_to_xy(s5_profile_point_t const * p_point,
                      int32_t * p_x_mm, int32_t * p_y_mm);
void s5_profile_build(s5_profile_point_t const * p_points, uint32_t count,
                      s5_profile_edge_t const * p_left,
                      s5_profile_edge_t const * p_right, int32_t body_deg,
                      s5_profile_t * p_out);
void s5_profile_set_clear(s5_profile_t * p_out);

uint32_t s5_profile_mm(int32_t value_mm);
void     s5_profile_set_angle(s5_profile_t * p_profile, int32_t angle_deg);
int32_t  s5_profile_angle(s5_profile_t const * p_profile);
uint32_t s5_profile_next_seq(uint32_t seq);
int32_t  s5_profile_pass_mm(bool b_box_left);
int32_t  s5_profile_look_reach_mm(bool b_box_left);
int32_t  s5_profile_half_mm(bool b_left);

#endif /* S5_PROFILE_H */

/*** end of file ***/
