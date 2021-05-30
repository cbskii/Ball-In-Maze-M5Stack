#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common.h"
#include "buttons.h"
#include "display.h"
#include "motion.h"

void app_main(void)
{
    // TODO get working power on/off button
    // TODO do stuff with buttons
    // TODO generate different mazes
    /* display_coord_t prev_ball_pos; */
    /* display_coord_t new_ball_pos; */
    ball_t ball;

    buttons_init();
    motion_init();
    display_init();
    display_draw_maze(&ball);

    while (true) {
        motion_update_ball_pos(&ball);
        display_move_ball(&ball);
        vTaskDelay(50 / portTICK_RATE_MS); // TODO adjust as necessary
    }

    display_shutdown();
    motion_shutdown();
    buttons_exit();
}
