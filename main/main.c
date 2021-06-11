#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "common.h"
#include "buttons.h"
#include "display.h"
#include "motion.h"
#include "maze_generated.h" /* Only include once as this declares generated arrays */

void init_ball_and_maze(ball_t *ball, maze_t *maze)
{
    /* Assumes ball and maze were generated correctly */
    size_t ball_size = sizeof(g_start_ball);
    size_t maze_size = sizeof(g_generated_maze);
    /* size_t ball_size = sizeof(ball_t); */
    /* size_t maze_size = sizeof(maze_t); */

    if (sizeof(g_start_ball) > sizeof(ball_t)) {
        ball_size = sizeof(ball_t);
    }

    if (sizeof(g_generated_maze) > sizeof(maze_t)) {
        maze_size = sizeof(maze_t);
    }

    memcpy(ball, g_start_ball, ball_size);
    memcpy(maze, g_generated_maze, maze_size);

    printf("ball->prev_pos: x=%d, y=%d\n", ball->prev_pos.x, ball->prev_pos.y);
    printf("ball->pos: x=%d, y=%d\n", ball->pos.x, ball->pos.y);
    printf("ball->velocity: x=%f, y=%f, z=%f\n", ball->velocity.x, ball->velocity.y, ball->velocity.z);


    /* /1* Ball starting position *1/ */
    /* ball->prev_pos.x = 20; */
    /* ball->prev_pos.y = display_get_height() / 2; */
    /* ball->pos.x = 20; */
    /* ball->pos.y = display_get_height() / 2; */

    /* /1* Create maze *1/ */
    /* // TODO build maze based off of a config file and verify num_walls is < MAX_NUM_WALLS */
    /* printf("display dimensions: height = %d, width = %d\n", display_get_height(), display_get_width()); */
    /* maze->walls[0].start.x = 0; */
    /* maze->walls[0].start.y = 150; */
    /* maze->walls[0].end.x = 90; */
    /* maze->walls[0].end.y = 150; */

    /* maze->walls[1].start.x = 160; */
    /* maze->walls[1].start.y = 0; */
    /* maze->walls[1].end.x = 160; */
    /* maze->walls[1].end.y = 240; */

    /* maze->num_walls = 2; */
}

void app_main(void)
{
    ball_t ball = {};
    maze_t maze = {};

    buttons_init();
    motion_init();
    display_init();

    /* Verify generated maze is based off correct values */
    if ((GEN_DISPLAY_WIDTH != display_get_width()) || (GEN_DISPLAY_HEIGHT != display_get_height())) {
        printf("Maze was generated with incorrect display width/height settings!. "
               "Supported dimensions: width=%d, height=%d\n",
               display_get_width(), display_get_height());
        return;
    }

    /* Build maze */
    init_ball_and_maze(&ball, &maze);
    display_draw_maze(&ball, &maze);

    while (true) {
        /* Note: changing this delay may require changing ball speed as well */
        vTaskDelay(20 / portTICK_RATE_MS); 
        motion_update_ball_pos(&ball, &maze);
        display_move_ball(&ball);
    }

    display_shutdown();
    motion_shutdown();
    buttons_exit();
}
