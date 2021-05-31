#include "driver/timer.h"

#include "tft.h" // TODO when can rely on maze wall detection
#include "motion.h"
#include "mpu9250.h"

void motion_init()
{
    mpu9250_init();

#ifdef USE_TIMER
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = 16,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = false,
    }; /* default clock source is APB */

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
#endif
}

void motion_shutdown()
{
    mpu9250_shutdown();
}

// TODO needs to take a pointer to maze layout (const) so it can handle bouncing off walls
// If new position of ball is going to hit wall, modify new position to right before wall
// Then reverse direction of velocity (make negative) and divide by some factor (e.g. 3).
void motion_update_ball_pos(ball_t *ball)
{
    if (!ball) {
        printf("%s: ball is NULL.\n", __func__);
        return;
    }

#ifdef USE_TIMER
    double elapsed_time_sec;
    static double time_delta_sec;
    static double last_update_time_sec;
    static bool first_update = true;

    if (first_update) {
        /* Start timer and skip first update */
        timer_start(TIMER_GROUP_0, TIMER_0);
        timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &last_update_time_sec);
        first_update = false;
        return;
    }

    /* Calculate time delta from last measurement */
    timer_get_counter_time_sec(TIMER_GROUP_0, TIMER_0, &elapsed_time_sec);
    time_delta_sec = elapsed_time_sec - last_update_time_sec;
    last_update_time_sec = elapsed_time_sec;
#endif

    /* Save previous position before modifying */
    ball->prev_pos = ball->pos;

    /* Collect latest accelerometer data */
    vector_float_t accel;
    mpu9250_get_accel_data(&accel);

#ifdef USE_TIMER
    ball->velocity.x += -accel.x * time_delta_sec;
    ball->velocity.y += accel.y * time_delta_sec;
#else
    /* Multiplying acceleration by this number seems to work pretty well */
    ball->velocity.x += -accel.x * 0.3;
    ball->velocity.y += accel.y * 0.3;
#endif

    /* Clamp max velocity ball can reach */
    #define MAX_VELOCITY (5)
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

    // TODO temporary bounds checking, replace with check for maze wall
    #define FRICTION (3)
    #define WALL_BUFFER (BALL_RADIUS + 3)
    if (ball->pos.x > DEFAULT_TFT_DISPLAY_WIDTH - WALL_BUFFER) {
        ball->pos.x = DEFAULT_TFT_DISPLAY_WIDTH - WALL_BUFFER;
        ball->velocity.x = -ball->velocity.x / FRICTION;
    } else if (ball->pos.x < WALL_BUFFER) {
        ball->pos.x = WALL_BUFFER;
        ball->velocity.x = -ball->velocity.x / FRICTION;
    }

    if (ball->pos.y > DEFAULT_TFT_DISPLAY_HEIGHT - WALL_BUFFER) {
        ball->pos.y = DEFAULT_TFT_DISPLAY_HEIGHT - WALL_BUFFER;
        ball->velocity.y = -ball->velocity.y / FRICTION;
    } else if (ball->pos.y < WALL_BUFFER) {
        ball->pos.y = WALL_BUFFER;
        ball->velocity.y = -ball->velocity.y / FRICTION;
    }

    ball->pos.x += ball->velocity.x;
    ball->pos.y += ball->velocity.y;

    printf("accel: x=%0.1f, y=%0.1f, z=%0.1f\n", accel.x, accel.y, accel.z);
    printf("velocity: x=%0.1f, y=%0.1f\n", ball->velocity.x, ball->velocity.y);
    return;
}
