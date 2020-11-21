#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "buttons.h"
#include "unity.h"

void app_main(void)
{
    // TODO for now just test different components

    /* Install ISR service */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); /* low priority interrupt */

    /* Initialize buttons */
    buttons_init();

    /* Loop forever */
    for (;;) {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    /* Cleanup buttons */
    buttons_exit();

    /* Uninstall ISR service */
    gpio_uninstall_isr_service();
}
