#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "common.h"
#include "buttons.h"
#include "display.h"
#include "motion.h"
#include "maze_generated.h" /* Only include once as this declares generated arrays */

static void init_ball_and_maze(ball_t *ball, maze_t *maze)
{
    /* Assumes ball and maze were generated correctly */
    memcpy(ball, g_start_ball, sizeof(ball_t));
    memcpy(maze, g_generated_maze, sizeof(maze_t));
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
