/*
 * Ball motion control based on IMU measurements.
 */

#ifndef MOTION_H
#define MOTION_H

#include "common.h"

void motion_init();
void motion_shutdown();
void motion_update_ball_pos(ball_t *ball, const maze_t *maze);

#endif /* MOTION_H */
