/*
 * LEDC (LED Controller) fade example
 *
 * This example code is in the Public Domain (or CC0 licensed, at
 * your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_err.h"

# define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
# include "esp_log.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 */
# define LEDC_HS_TIMER          LEDC_TIMER_0
# define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE

# define LEDC_HS_CH0_GPIO       18
# define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
# define LEDC_HS_CH1_GPIO       19
# define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

# define LEDC_LS_TIMER          LEDC_TIMER_1
# define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

# define LEDC_LS_CH2_GPIO       4
# define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
# define LEDC_LS_CH3_GPIO       5
# define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

# define LEDC_TEST_CH_NUM       (4)

/* final brightness */
# define LEDC_TEST_DUTY         (4000)

# define DUTY_RESOLUTION	LEDC_TIMER_13_BIT

# define LEDC_TEST_FADE_TIME    (6000)

# define DELAY			(LEDC_TEST_FADE_TIME + 3000)
# define DELAY_ERROR		10000

/*
 * resolution of pulses; bigger = more finely divided.
 * around 30 and below produces flickering.
 */
# define PWM_FREQ_HZ		5000

static const char		*log_tag = "ledc02";

/*
 * Prepare and set configuration of timers that will be used by
 * LED Controller
 */
void config_timers() {
    // Set configuration of timer0 for high speed channels
    const ledc_timer_config_t ledc_timer_hs = {
        // resolution of PWM duty
        .duty_resolution = DUTY_RESOLUTION,
	// frequency of PWM signal
        .freq_hz = PWM_FREQ_HZ,
	// timer mode
        .speed_mode = LEDC_HS_MODE,
	// timer index
        .timer_num = LEDC_HS_TIMER,
	// Auto select the source clock
        .clk_cfg = LEDC_AUTO_CLK
    };

    if (ledc_timer_config(&ledc_timer_hs) != ESP_OK) {
	ESP_LOGE(log_tag, "high speed timer config failed\n");
	vTaskDelay(DELAY_ERROR / portTICK_PERIOD_MS);
    }
}

static xQueueHandle ledc_evt_queue = NULL;

static void IRAM_ATTR ledc_isr_handler(void *arg) {
    const uint32_t chan_num = (uint32_t) arg;

    xQueueSend(ledc_evt_queue, &chan_num, NULL);
}

static void ledc_task(void *arg) {
    uint32_t chan_num;
    BaseType_t xTaskWokenByReceive = pdFALSE;

    for (;;) {
	if (xQueueReceive(ledc_evt_queue, (void *) &chan_num, &xTaskWokenByReceive)) {
	    /*ESP_LOGI(log_tag, "waiting: %d\n", uxQueueMessagesWaiting(ledc_evt_queue));*/
	    printf("LEDC[%d] intr\n", chan_num);
	}
    }
}

void app_main(void) {
    config_timers();

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel = {
	 .channel    = LEDC_HS_CH0_CHANNEL,
	 .duty       = 0,
	 .gpio_num   = LEDC_HS_CH0_GPIO,
	 .speed_mode = LEDC_HS_MODE,
	 .hpoint     = 0,
	 .timer_sel  = LEDC_HS_TIMER,
	 .intr_type  = LEDC_INTR_FADE_END,
    };

    // Set LED Controller with previously prepared configuration
    ESP_ERROR_CHECK_WITHOUT_ABORT (ledc_channel_config(&ledc_channel));

    // create a queue to handle gpio event from isr
    if ((ledc_evt_queue = xQueueCreate(10, sizeof(uint32_t))) == 0) {
	    ESP_LOGE(log_tag, "ledc queue create failed\n");
	    vTaskDelay(DELAY_ERROR / portTICK_PERIOD_MS);
    }

    // start ledc task
    xTaskCreate(ledc_task, "ledc_task", 2048, NULL, 10, NULL);

    // Initialize fade service.
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_SHARED);

    for (;;) {
        ESP_LOGI(log_tag, "1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);

	ESP_ERROR_CHECK_WITHOUT_ABORT (ledc_set_fade_with_time(ledc_channel.speed_mode,
                ledc_channel.channel,
                LEDC_TEST_DUTY,
                LEDC_TEST_FADE_TIME));
	    
	ESP_ERROR_CHECK_WITHOUT_ABORT (ledc_fade_start(ledc_channel.speed_mode,
                ledc_channel.channel,
                LEDC_FADE_NO_WAIT) );

	ESP_ERROR_CHECK_WITHOUT_ABORT (ledc_isr_register(&ledc_isr_handler, (void *) ledc_channel.channel,
                ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_SHARED /*ESP_INTR_FLAG_LEVELMASK*/, NULL) );

        vTaskDelay(DELAY / portTICK_PERIOD_MS);

        ESP_LOGI(log_tag, "2. LEDC fade down to duty = 0\n");

	ESP_ERROR_CHECK_WITHOUT_ABORT (ledc_set_fade_with_time(ledc_channel.speed_mode,
                ledc_channel.channel,
		0,
		LEDC_TEST_FADE_TIME) );
	    
	ESP_ERROR_CHECK_WITHOUT_ABORT (ledc_fade_start(ledc_channel.speed_mode,
                ledc_channel.channel,
                LEDC_FADE_NO_WAIT) );

        vTaskDelay(DELAY /  portTICK_PERIOD_MS);

        ESP_LOGI(log_tag, "3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);

	if (ledc_set_duty(ledc_channel.speed_mode,
                ledc_channel.channel,
		LEDC_TEST_DUTY) != ESP_OK) {
	    ESP_LOGE(log_tag, "ledc set duty failed\n");
	    vTaskDelay(DELAY_ERROR / portTICK_PERIOD_MS);
	}
	    
	if (ledc_update_duty(ledc_channel.speed_mode,
		ledc_channel.channel) != ESP_OK) {
	    ESP_LOGE(log_tag, "ledc update duty failed\n");
	    vTaskDelay(DELAY_ERROR / portTICK_PERIOD_MS);
	}

        vTaskDelay(DELAY / portTICK_PERIOD_MS);

        ESP_LOGI(log_tag, "4. LEDC set duty = 0 without fade\n");

	if (ledc_set_duty(ledc_channel.speed_mode,
                ledc_channel.channel,
		0) != ESP_OK) {
	    ESP_LOGE(log_tag, "ledc set duty failed\n");
	    vTaskDelay(DELAY_ERROR / portTICK_PERIOD_MS);
	}
	    
	if (ledc_update_duty(ledc_channel.speed_mode,
                ledc_channel.channel) != ESP_OK) {
	    ESP_LOGE(log_tag, "ledc update duty failed\n");
	    vTaskDelay(DELAY_ERROR / portTICK_PERIOD_MS);
	}

        vTaskDelay(DELAY / portTICK_PERIOD_MS);
    }
}
