#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common.h"
#include "buttons.h"
#include "display.h"
#include "motion.h"

void init_ball_and_maze(ball_t *ball, maze_t *maze)
{
    /* Ball starting position */
    ball->prev_pos.x = 20;
    ball->prev_pos.y = display_get_height() / 2;
    ball->pos.x = 20;
    ball->pos.y = display_get_height() / 2;

    /* Create maze */
    // TODO build maze based off of a config file and verify num_walls is < MAX_NUM_WALLS
    printf("display dimensions: height = %d, width = %d\n", display_get_height(), display_get_width());
    maze->walls[0].start.x = 0;
    maze->walls[0].start.y = 150;
    maze->walls[0].end.x = 90;
    maze->walls[0].end.y = 150;

    maze->walls[1].start.x = 160;
    maze->walls[1].start.y = 0;
    maze->walls[1].end.x = 160;
    maze->walls[1].end.y = 240;

    maze->num_walls = 2;
}

void app_main(void)
{
    ball_t ball = {};
    maze_t maze = {};

    buttons_init();
    motion_init();
    display_init();

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
