/* RMT example -- RGB LED Strip

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"

static const char *TAG = "example";

#define RMT_TX_CHANNEL RMT_CHANNEL_0

#define EXAMPLE_CHASE_SPEED_MS (40)
#define STRIP_LED_NUMBER 10
#define RMT_TX_GPIO 25

extern void led_strip_hsv2rgb(uint32_t, uint32_t, uint32_t, uint32_t *, uint32_t *, uint32_t *);

void app_main(void) {
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_TX_CHANNEL);

    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(STRIP_LED_NUMBER,
            (led_strip_dev_t) config.channel);

    led_strip_t *strip = led_strip_new_rmt_ws2812(&strip_config);

    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }

    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");

    for (;;) {
	for (int j = 0; j < CONFIG_EXAMPLE_STRIP_LED_NUMBER; j += 1) {
	    // Build RGB values
	    hue = j * 360 / CONFIG_EXAMPLE_STRIP_LED_NUMBER + start_rgb;

	    // a value of 100 is very bright; 30 is better
	    led_strip_hsv2rgb(hue, 100, 10, &red, &green, &blue);
           
	    // Write RGB values to strip driver
	    ESP_ERROR_CHECK(strip->set_pixel(strip, j, red, green, blue));
            
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(strip->refresh(strip, 100));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }

	vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

        start_rgb += 20;
    }
}
