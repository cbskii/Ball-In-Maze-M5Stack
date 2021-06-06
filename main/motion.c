#include "driver/timer.h"

#include "motion.h"
#include "mpu9250.h"
#include "display.h"

/*
 * These values help determine how fast the ball will move in response to accelerations and what
 * happens when the ball rebounds off of a wall. They should be tuned based on main loop delay
 * length and wall detection algorithm to find a value that feels responsive and still challenging.
 */
#define MAX_VELOCITY (5)
#define MIN_VELOCITY (0.5)
#define FRICTION (2)
#define BALL_SPEED_FACTOR (0.3)

void motion_init()
{
    mpu9250_init();
}

void motion_shutdown()
{
    mpu9250_shutdown();
}

static bool point_is_below_wall(pos_t point, wall_t wall)
{
    /* Y direction is reversed, smaller numbers near top of display */
    if (wall.start.y <= wall.end.y) {
        return point.y > (wall.end.y + BALL_RADIUS);
    } else {
        return point.y > (wall.start.y + BALL_RADIUS);
    }
}

static bool point_is_above_wall(pos_t point, wall_t wall)
{
    /* Y direction is reversed, smaller numbers near top of display */
    if (wall.start.y <= wall.end.y) {
        return point.y < (wall.start.y - BALL_RADIUS);
    } else {
        return point.y < (wall.end.y - BALL_RADIUS);
    }
}

static bool point_is_left_of_wall(pos_t point, wall_t wall)
{
    if (wall.start.x <= wall.end.x) {
        return point.x < (wall.start.x - BALL_RADIUS);
    } else {
        return point.x < (wall.end.x - BALL_RADIUS);
    }
}

static bool point_is_right_of_wall(pos_t point, wall_t wall)
{
    if (wall.start.x <= wall.end.x) {
        return point.x > (wall.end.x + BALL_RADIUS);
    } else {
        return point.x > (wall.start.x + BALL_RADIUS);
    }
}

static bool point_is_in_wall(pos_t point, wall_t wall)
{
    return (!point_is_above_wall(point, wall) &&
            !point_is_below_wall(point, wall) &&
            !point_is_left_of_wall(point, wall) &&
            !point_is_right_of_wall(point, wall));
}

static bool is_ball_moving_vertically(ball_t ball)
{
    if (ball.velocity.y < -MIN_VELOCITY || ball.velocity.y > MIN_VELOCITY) {
        return true;
    }

    return false;
}

static bool is_ball_moving_horizontally(ball_t ball)
{
    if (ball.velocity.x < -MIN_VELOCITY || ball.velocity.x > MIN_VELOCITY) {
        return true;
    }

    return false;
}

void motion_update_ball_pos(ball_t *ball, const maze_t *maze)
{
    bool hit_vert_wall = false;
    bool hit_horiz_wall = false;

    if (!ball || !maze) {
        printf("%s: ball or maze is NULL.\n", __func__);
        return;
    }

    /* Save previous position before modifying */
    ball->prev_pos = ball->pos;

    /* Collect latest accelerometer data */
    vector_float_t accel;
    mpu9250_get_accel_data(&accel);

    /* Multiplying acceleration by this number seems to work pretty well */
    ball->velocity.x += -accel.x * BALL_SPEED_FACTOR;
    ball->velocity.y += accel.y * BALL_SPEED_FACTOR;

    /*
     * Just update position directly based on "velocity" value; works fine. Also handle
     * unwanted integer wrapping due to negative velocities (unlikely).
     */
    int temp_pos;
    temp_pos = ball->pos.x + ball->velocity.x;
    if (temp_pos < 0) {
        ball->pos.x = 0;
    } else {
        ball->pos.x = temp_pos;
    }

    temp_pos = ball->pos.y + ball->velocity.y;
    if (temp_pos < 0) {
        ball->pos.y = 0;
    } else {
        ball->pos.y = temp_pos;
    }

    /* Clamp the max velocity the ball can reach */
    if (ball->velocity.x > MAX_VELOCITY) {
        ball->velocity.x = MAX_VELOCITY;
    } else if (ball->velocity.x < -MAX_VELOCITY) {
        ball->velocity.x = -MAX_VELOCITY;
    }

    if (ball->velocity.y > MAX_VELOCITY) {
        ball->velocity.y = MAX_VELOCITY;
    } else if (ball->velocity.y < -MAX_VELOCITY) {
        ball->velocity.y = -MAX_VELOCITY;
    }

    /* Check for ball hitting display edges */
    if (ball->pos.x > display_get_width() - BALL_RADIUS) {
        ball->pos.x = display_get_width() - BALL_RADIUS;
        hit_vert_wall = true;
    } else if (ball->pos.x < BALL_RADIUS) {
        ball->pos.x = BALL_RADIUS;
        hit_vert_wall = true;
    }

    if (ball->pos.y > display_get_height() - BALL_RADIUS) {
        ball->pos.y = display_get_height() - BALL_RADIUS;
        hit_horiz_wall = true;
    } else if (ball->pos.y < BALL_RADIUS) {
        ball->pos.y = BALL_RADIUS;
        hit_horiz_wall = true;
    }

    /*
     * Only check one wall at a time. This has disadvantages for being able to detect if a ball
     * should be moved around a corner and instead the effect is that the ball's speed appears
     * slower. Not an issue with ball speed tuning values used above.
     */
    for (int i = 0; i < maze->num_walls; i++) {
        /*
         * If the new ball position is inside a maze wall, then use the ball's previous position
         * to figure out where the ball needs to be and move it just outside the wall.
         */
        if (point_is_in_wall(ball->pos, maze->walls[i])) {
            if (point_is_below_wall(ball->prev_pos, maze->walls[i])) {
                hit_horiz_wall = true;
                if (maze->walls[i].start.y <= maze->walls[i].end.y) {
                    ball->pos.y = maze->walls[i].end.y + BALL_RADIUS + 1;
                } else {
                    ball->pos.y = maze->walls[i].start.y + BALL_RADIUS + 1;
                }
            } else if (point_is_above_wall(ball->prev_pos, maze->walls[i])) {
                hit_horiz_wall = true;
                if (maze->walls[i].start.y <= maze->walls[i].end.y) {
                    ball->pos.y = maze->walls[i].start.y - BALL_RADIUS - 1;
                } else {
                    ball->pos.y = maze->walls[i].end.y - BALL_RADIUS - 1;
                }
            } else if (point_is_left_of_wall(ball->prev_pos, maze->walls[i])) {
                hit_vert_wall = true;
                if (maze->walls[i].start.x <= maze->walls[i].end.x) {
                    ball->pos.x = maze->walls[i].start.x - BALL_RADIUS - 1;
                } else {
                    ball->pos.x = maze->walls[i].end.x - BALL_RADIUS - 1;
                }
            } else if (point_is_right_of_wall(ball->prev_pos, maze->walls[i])) {
                hit_vert_wall = true;
                if (maze->walls[i].start.x <= maze->walls[i].end.x) {
                    ball->pos.x = maze->walls[i].end.x + BALL_RADIUS + 1;
                } else {
                    ball->pos.x = maze->walls[i].start.x + BALL_RADIUS + 1;
                }
            } else {
                printf("Error: previous point was in wall.\n");
                return;
            }
        }
    }

    if (hit_horiz_wall) {
        if (is_ball_moving_vertically(*ball)) {
            ball->velocity.y = -ball->velocity.y / FRICTION;
        }
    }

    if (hit_vert_wall) {
        if (is_ball_moving_horizontally(*ball)) {
            ball->velocity.x = -ball->velocity.x / FRICTION;
        }
    }

    /* printf("ball position: x=%d, y=%d\n", ball->pos.x, ball->pos.y); */
    /* printf("ball velocity: x=%0.1f, y=%0.1f\n", ball->velocity.x, ball->velocity.y); */
    return;
}
