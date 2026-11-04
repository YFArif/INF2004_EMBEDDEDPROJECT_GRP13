/** @file s3_experiment.h
 * @brief Isolated, opt-in floor experiments; normal S2/S3/S5 is unchanged.
 */
#ifndef S3_EXPERIMENT_H
#define S3_EXPERIMENT_H

/* 0: normal; 1: GP27 follow/recover; 2: armed left; 3: stationary IR view.
 * Experiments own motion and buttons instead of S5. No ultrasonic guard.
 */
#define S3_EXPERIMENT_MODE (0)

#if ((S3_EXPERIMENT_MODE < 0) || (S3_EXPERIMENT_MODE > 3))
#error "S3_EXPERIMENT_MODE must be 0, 1, 2 or 3"
#endif
#if (S3_EXPERIMENT_MODE != 0) && APP_TEST
#error "GP27 experiments must not run alongside subsystem self-tests"
#endif

#endif
