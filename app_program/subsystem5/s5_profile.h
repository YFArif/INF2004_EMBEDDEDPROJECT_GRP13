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

/* Chassis geometry. TODO: measure the robot and update (S5-U04). */
#define S5_ROBOT_WIDTH_MM       (150)   /* Outside of tyre to tyre.      */
#define S5_ROBOT_LENGTH_MM      (200)   /* Front bumper to rear.         */
#define S5_PATH_MARGIN_MM       (50)    /* Gap kept from the obstacle.   */

/* Scan thresholds (Week 6 report 6.4, preliminary until S5-U04). */
#define S5_HIT_RANGE_MM         (400)   /* Closer than this is a hit.    */
#define S5_EDGE_JUMP_MM         (100)   /* Larger range step is an edge. */
#define S5_EDGE_CORR_DEG        (6)     /* Edge correction, A16.         */
#define S5_CLEAR_DEPTH_MM       (300)   /* Look this far past the edge.  */
#define S5_CLEAR_OPEN_MM        (1000)  /* Clearance with nothing seen.  */

/* Range value of a point at which nothing answered. */
#define S5_PROFILE_NO_ECHO      (0xFFFFu)

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

/** The obstacle profile (interface IF-09). Distances are in mm. */
typedef struct
{
    uint32_t    seq;            /* Profiles built since power-up.        */
    bool        b_valid;        /* False until the first scan.           */
    bool        b_on_path;      /* Something blocks the robot's path.    */
    bool        b_left_edge;    /* Left edge found inside servo range.   */
    bool        b_right_edge;   /* Right edge found inside servo range.  */
    uint16_t    closest_mm;     /* Nearest point of the obstacle.        */
    int16_t     closest_deg;    /* Servo angle of that point.            */
    uint16_t    width_mm;       /* Edge to edge, after beam correction.  */
    uint16_t    clear_left_mm;  /* Free gap beside the left edge.        */
    uint16_t    clear_right_mm; /* Free gap beside the right edge.       */
    uint16_t    shift_left_mm;  /* Sideways move to pass on the left.    */
    uint16_t    shift_right_mm; /* Sideways move to pass on the right.   */
    uint16_t    depth_mm;       /* Length along the track; 0 = unknown.  */
} s5_profile_t;

bool s5_profile_is_hit(s5_profile_point_t const * p_point);
bool s5_profile_same_object(uint16_t prev_mm,
                            s5_profile_point_t const * p_next);
void s5_profile_to_xy(s5_profile_point_t const * p_point,
                      int32_t * p_x_mm, int32_t * p_y_mm);
void s5_profile_build(s5_profile_point_t const * p_points, uint32_t count,
                      s5_profile_edge_t const * p_left,
                      s5_profile_edge_t const * p_right,
                      s5_profile_t * p_out);
void s5_profile_set_clear(s5_profile_t * p_out);

#endif /* S5_PROFILE_H */

/*** end of file ***/
