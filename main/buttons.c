
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "buttons.h"

/* Button GPIOs */
#define BUTTON_A GPIO_NUM_37
#define BUTTON_B GPIO_NUM_38
#define BUTTON_C GPIO_NUM_39
#define NUM_BUTTONS (sizeof(buttons_list) / sizeof(uint32_t))
#define get_button_string(button) #button

/* Bit that represents button press/release state */
#define STATE_BIT 8

TaskHandle_t button_task_handle = NULL;

static const uint32_t buttons_list[] = {BUTTON_A, BUTTON_B, BUTTON_C};

static void button_isr_handler(void *arg) {
    uint32_t pressed_button = (uint32_t) arg;
    uint32_t button_state = gpio_get_level(pressed_button);

    /* Notify the button task of which button was pressed
     * where the last two bytes contain the button state
     * and the button pressed */
    xTaskNotifyFromISR(button_task_handle,
                       (button_state << STATE_BIT) | pressed_button,
                       eSetValueWithoutOverwrite,
                       NULL);
}

static void button_task(void *pvParameters) {
    uint32_t button_info, pressed_button;
    bool is_button_released;
    char button_string[9];

    while (true) {
        /* Wait for a button press notification and clear
         * the notification afterwards. Blocks indefinitely until
         * a notification is received. */
        xTaskNotifyWait(0x00, ULONG_MAX, &button_info, portMAX_DELAY);
        is_button_released = !!(button_info >> STATE_BIT);
        pressed_button = (button_info & 0xff);

        if (pressed_button == BUTTON_A) {
            strcpy(button_string, get_button_string(BUTTON_A));
        } else if (pressed_button == BUTTON_B) {
            strcpy(button_string, get_button_string(BUTTON_B));
        } else if (pressed_button == BUTTON_C) {
            strcpy(button_string, get_button_string(BUTTON_C));
        } else {
            strcpy(button_string, "UNKNOWN!");
        }

        // TODO for now just print button action, eventually use for something
        printf("%s - %s\n",
               button_string,
               is_button_released ? "RELEASED\n" : "PRESSED");
    }
}

void buttons_init(void) {
    esp_err_t err;

    /* Install ISR service */
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); /* low priority interrupt */

    /* Configure button GPIOs */
    for (int i = 0; i < NUM_BUTTONS; i++) {
        gpio_pad_select_gpio(buttons_list[i]);
        gpio_set_direction(buttons_list[i], GPIO_MODE_INPUT);
        gpio_set_intr_type(buttons_list[i], GPIO_INTR_ANYEDGE);

        /* Add ISR handler for the button GPIO */
        err = gpio_isr_handler_add(buttons_list[i], button_isr_handler,
                                   (void*) buttons_list[i]);
        if (err != ESP_OK) {
            printf("Error occurred adding isr handler for GPIO %d. "
                   "Error: %d\n", buttons_list[i], err);
        }
    }

    /* Start button task */
    xTaskCreate(button_task, "button_task", 2048, NULL, 10,
                &button_task_handle);
}

void buttons_exit(void) {
    esp_err_t err;

    /* Delete button task */
    if (button_task_handle != NULL) {
        vTaskDelete(button_task_handle);
    }

    /* Remove each GPIO ISR handler */
    for (int i = 0; i < NUM_BUTTONS; i++) {
        err = gpio_isr_handler_remove(buttons_list[i]);
        if (err != ESP_OK) {
            printf("Error occurred adding isr handler for GPIO %d. "
                   "Error: %d\n", buttons_list[i], err);
        }
    }

    /* Uninstall ISR service */
    gpio_uninstall_isr_service();
}
