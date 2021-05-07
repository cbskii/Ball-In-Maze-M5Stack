#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "buttons.h"
#include "display.h"

void app_main(void)
{
    // TODO for now just test different components; eventually separate into unist test apps
    // TODO get working power on/off button
    esp_err_t ret;

    /* Install ISR service */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); /* low priority interrupt */

    buttons_init();
    display_init();

    // TODO test display w/ simplest maze
    display_test();
    while (true) {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    buttons_exit();
    display_shutdown();

    /* Uninstall ISR service */
    gpio_uninstall_isr_service();
}
