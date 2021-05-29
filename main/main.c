#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "buttons.h"
#include "display.h"

// TODO test movement in horizontal line; eventually use acceleromter to move position
void get_new_ball_pos(display_coord_t *new_pos)
{
    static display_coord_t prev_pos = {
        .x = 20,
        .y =  DEFAULT_TFT_DISPLAY_HEIGHT / 2,
    };

    if (!new_pos) {
        printf("%s: new_pos is NULL.", __func__);
        return;
    }

    if (prev_pos.x + 5 >= DEFAULT_TFT_DISPLAY_WIDTH - 20) {
        new_pos->x = 20;
    } else {
        new_pos->x = prev_pos.x + 5;
    }

    new_pos->y = prev_pos.y + 0;
    prev_pos = *new_pos;
}

void app_main(void)
{
    // TODO for now just test different components; eventually separate into unist test apps
    // TODO get working power on/off button
    // TODO do stuff with buttons
    // TODO generate different mazes
    display_coord_t prev_ball_pos;
    display_coord_t new_ball_pos;

    buttons_init();
    display_init();
    display_draw_maze(&prev_ball_pos);

    while (true) {
        get_new_ball_pos(&new_ball_pos);
        display_move_ball(prev_ball_pos, new_ball_pos);
        prev_ball_pos = new_ball_pos;
        vTaskDelay(20 / portTICK_RATE_MS); // TODO adjust as necessary
    }

    buttons_exit();
    display_shutdown();
}
