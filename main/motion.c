#include "driver/timer.h"

#include "motion.h"
#include "mpu9250.h"
#include "display.h"

/* How close the ball can get to a wall or edge of display */
#define WALL_BUFFER (BALL_RADIUS + 3)

/*
 * These values help determine how fast the ball will move in response to accelerations and what
 * happens when the ball rebounds off of a wall. They should be tuned based on main loop delay
 * length and wall detection algorithm to find a value that feels responsive and still challenging.
 */
#define MAX_VELOCITY (5)
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
    if (wall.start.y >= wall.end.y) {
        return point.y > (wall.start.y + WALL_BUFFER);
    } else {
        return point.y > (wall.end.y + WALL_BUFFER);
    }
}

static bool point_is_left_of_wall(pos_t point, wall_t wall)
{
    if (wall.start.x <= wall.end.x) {
        return point.x < (wall.start.x - WALL_BUFFER);
    } else {
        return point.x < (wall.end.x - WALL_BUFFER);
    }
}

/* Note: assumes only vertical and horizontal walls for now */
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

    /* Just update position directly based on "velocity" value; works fine */
    ball->pos.x += ball->velocity.x;
    ball->pos.y += ball->velocity.y;

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
    if (ball->pos.x > display_get_width() - WALL_BUFFER) {
        ball->pos.x = display_get_width() - WALL_BUFFER;
        hit_vert_wall = true;
    } else if (ball->pos.x < WALL_BUFFER) {
        ball->pos.x = WALL_BUFFER;
        hit_vert_wall = true;
    }

    if (ball->pos.y > display_get_height() - WALL_BUFFER) {
        ball->pos.y = display_get_height() - WALL_BUFFER;
        hit_horiz_wall = true;
    } else if (ball->pos.y < WALL_BUFFER) {
        ball->pos.y = WALL_BUFFER;
        hit_horiz_wall = true;
    }

    // TODO need special checking for hitting corner/wall intersection?

    /*
     * Only check one wall at a time. This has disadvantages for being able to detect if a ball
     * should be moved around a corner and instead the effect is that the ball's speed appears
     * slower. Not an issue with ball speed tuning values defined above.
     */
    for (int i = 0; i < maze->num_walls; i++) {
        // TODO remove debug logs
        /* printf("prev ball: (%d, %d), ball: (%d, %d), wall: (%d, %d) -> (%d, %d)\n", */
        /*         ball->prev_pos.x, ball->prev_pos.y, ball->pos.x, ball->pos.y, */
        /*         maze->walls[i].start.x, maze->walls[i].start.y, */
        /*         maze->walls[i].end.x, maze->walls[i].end.y); */
        /* printf("prev ball under wall? %s | cur ball under wall? %s \n", point_is_below_wall(ball->prev_pos, maze->walls[i]) ? "yes": "no", point_is_below_wall(ball->pos, maze->walls[i]) ? "yes" : "no"); */
        /* printf("prev ball above wall? %s | cur ball above wall? %s \n", point_is_above_wall(ball->prev_pos, maze->walls[i]) ? "yes": "no", point_is_above_wall(ball->pos, maze->walls[i]) ? "yes" : "no"); */

        /* Use previous ball position to determine where new ball position should be */
        if (point_is_below_wall(ball->prev_pos, maze->walls[i])
                && !point_is_below_wall(ball->pos, maze->walls[i])) {
            ball->pos.y = maze->walls[i].start.y + WALL_BUFFER;
            hit_horiz_wall = true;
        }

        if (!point_is_below_wall(ball->prev_pos, maze->walls[i])
                && point_is_below_wall(ball->pos, maze->walls[i])) {
            ball->pos.y = maze->walls[i].start.y - WALL_BUFFER;
            hit_horiz_wall = true;
        }

        if (point_is_left_of_wall(ball->prev_pos, maze->walls[i])
                && !point_is_left_of_wall(ball->pos, maze->walls[i])) {
            ball->pos.x = maze->walls[i].start.x - WALL_BUFFER;
            hit_vert_wall = true;
        }

        if (!point_is_left_of_wall(ball->prev_pos, maze->walls[i])
                && point_is_left_of_wall(ball->pos, maze->walls[i])) {
            ball->pos.x = maze->walls[i].start.x + WALL_BUFFER;
            hit_vert_wall = true;
        }
    }

    if (hit_horiz_wall) {
        printf("Hit horizontal wall!\n");
        ball->velocity.y = -ball->velocity.y / FRICTION;
    }

    if (hit_vert_wall) {
        printf("Hit vertical wall!\n");
        ball->velocity.x = -ball->velocity.x / FRICTION;
    }

    /* /1* Just update position directly based on "velocity" value; works fine *1/ */
    /* ball->pos.x += ball->velocity.x; */
    /* ball->pos.y += ball->velocity.y; */

    /* printf("accel: x=%0.1f, y=%0.1f, z=%0.1f\n", accel.x, accel.y, accel.z); */
    /* printf("velocity: x=%0.1f, y=%0.1f\n", ball->velocity.x, ball->velocity.y); */
    return;
}
