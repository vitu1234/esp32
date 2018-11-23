/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
   https://github.com/espressif/esp-idf/blob/master/examples/get-started/blink/main/blink.c
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
static const char *TAG = "RELAIS";

/* on-board led gpio pin for my ESP-32 board */
#define GPIO_OUTPUT_PIN_RELAIS 22           /*!< gpio number for relais shield */

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(GPIO_OUTPUT_PIN_RELAIS);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_OUTPUT_PIN_RELAIS, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(GPIO_OUTPUT_PIN_RELAIS, 0);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "relais OFF");
        /* Blink on (output high) */
        gpio_set_level(GPIO_OUTPUT_PIN_RELAIS, 1);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "relais ON");
    }
}

void app_main() {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
