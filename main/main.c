#include <stdio.h>
#include "freertos/FreeRTOS.h"

#include "buttons.h"
#include "display.h"
#include "motion.h"

// TODO test movement in horizontal line; eventually use acceleromter to move position
static display_coord_t get_new_ball_pos(display_coord_t prev_pos)
{
    display_coord_t new_pos;
    if (prev_pos.x + 5 > DEFAULT_TFT_DISPLAY_WIDTH - 20) {
        new_pos.x = 20;
    } else {
        new_pos.x = prev_pos.x + 5;
    }
    new_pos.y = prev_pos.y + 0;
    return new_pos;
}

void app_main(void)
{
    // TODO get working power on/off button
    // TODO do stuff with buttons
    // TODO generate different mazes
    display_coord_t prev_ball_pos;
    display_coord_t new_ball_pos;

    buttons_init();
    motion_init();
    display_init();
    display_draw_maze(&prev_ball_pos);

    while (true) {
        motion_get_euler_angles();
        new_ball_pos = get_new_ball_pos(prev_ball_pos);
        display_move_ball(prev_ball_pos, new_ball_pos);
        prev_ball_pos = new_ball_pos;
        vTaskDelay(50 / portTICK_RATE_MS); // TODO adjust as necessary
    }

    display_shutdown();
    motion_shutdown();
    buttons_exit();
}
