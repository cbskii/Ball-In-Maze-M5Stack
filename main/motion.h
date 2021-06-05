/*
 * Ball motion control based on IMU measurements.
 */

#ifndef MOTION_H
#define MOTION_H

#include "common.h"

/**
 * @brief initializes motion control.
 */
void motion_init();

/**
 * @brief Shuts down motion control.
 */
void motion_shutdown();

/**
 * @brief Takes a ball and maze and updates the ball's position based on accelerometer datasheet.
 * Note: assumes the maze is composed of only horizontal and vertical walls.
 *
 * @param[out]  ball: pointer to the ball
 * @param[in]  maze: pointer to the maze
 */
void motion_update_ball_pos(ball_t *ball, const maze_t *maze);

#endif /* MOTION_H */
