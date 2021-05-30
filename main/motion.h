/*
 * Ball motion control based on IMU measurements.
 */

#ifndef MOTION_H
#define MOTION_H

void motion_init();
void motion_shutdown();
void motion_get_euler_angles();

#endif /* MOTION_H */
