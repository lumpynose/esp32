/*
 * GPIO Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your
 * option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 *  CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

# define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
# include "esp_log.h"

/**
 * Brief: This test code shows how to configure gpio and how to use
 * gpio interrupt.
 *
 * GPIO status:
 * GPIO02: output
 * GPIO13: input, pulled up, interrupt from rising edge and falling edge
 */

# define GPIO_OUTPUT_IO          GPIO_NUM_2
# define GPIO_OUTPUT_PIN_SEL     (1ULL<<GPIO_OUTPUT_IO)

# define GPIO_INPUT_IO           GPIO_NUM_13
# define GPIO_INPUT_PIN_SEL      (1ULL<<GPIO_INPUT_IO)

# define ESP_INTR_FLAG_DEFAULT   0

static const char *log_tag =	"main";

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    const uint32_t gpio_num = (uint32_t) arg;

    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg) {
    TickType_t previous = 0;
    int level = 0;
    uint32_t io_num;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI(log_tag, "GPIO[%d] intr, val: %d\n",
                io_num, gpio_get_level(io_num));
     
            // button released sets level to 1
            if (gpio_get_level(io_num) == 0) {
                TickType_t now = xTaskGetTickCount();
                TickType_t diff = now - previous;
                printf("time diff: %u\n", diff);

                if (diff > 30) {
                    level = !level;
                    gpio_set_level(GPIO_OUTPUT_IO, level);

                    ESP_LOGI(log_tag, "level: %d\n", level);
                }

                previous = now;
            }
        }
    }
}

void app_main(void) {
    const gpio_config_t io_conf_output = (gpio_config_t) {
        // disable interrupt
        .intr_type = GPIO_PIN_INTR_DISABLE,
        // set as output mode
        .mode = GPIO_MODE_OUTPUT,
        // bit mask of the pins that you want to set, e.g. OUTPUT_IO
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        // disable pull-down mode
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // disable pull-up mode
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    // configure GPIO with the given settings
    gpio_config(&io_conf_output);

    const gpio_config_t io_conf_input = (gpio_config_t) {
        // interrupt of rising edge
        .intr_type = GPIO_PIN_INTR_NEGEDGE,
        // set as input mode    
        .mode = GPIO_MODE_INPUT,
        // bit mask of the pins, use INPUT_IO here
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        // disable pull-down mode
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // enable pull-up mode
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    // configure GPIO with the given settings
    gpio_config(&io_conf_input);

    // change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO, GPIO_INTR_NEGEDGE);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example",
                2048, NULL, 10, NULL);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler,
                         (void*) GPIO_INPUT_IO);

    gpio_set_level(GPIO_OUTPUT_IO, 0);

    for (int cnt = 0; /**/; cnt++) {
        ESP_LOGI(log_tag, "cnt: %d\n", cnt);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
