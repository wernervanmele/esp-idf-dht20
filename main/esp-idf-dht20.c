#include <stdio.h>
#include "dht20.h"

/* prototype function */
void dht20_read_task(void *param);

/* declare rtos task handle */
TaskHandle_t read_data_h;

void app_main(void)
{

    dht20_begin();
    xTaskCreate(dht20_read_task, "read_values", (256 * 11), NULL, tskIDLE_PRIORITY, &read_data_h);
}

/**
 * @brief FreeRTOS task with infinte while loop to measure data.
 * @param param
 */
void dht20_read_task(void *param)
{
    static const char *TAG = "dht20_read_task";
    static dht20_data_t measurements;
    ESP_LOGI(TAG, "Entering measerument loop");

    while (1) {         /*@ infinite loop @*/
        if (dht20_is_calibrated()) {
            ESP_LOGI(TAG, "is calibrated....");
        } else {
            ESP_LOGI(TAG, "is NOT calibrated....");
        }
        (void)dht20_read_data(&measurements);
        ESP_LOGI(TAG, "Temperature:\t%.1fC.\t Avg: %.1fC", measurements.temperature, measurements.temp_avg);
        ESP_LOGI(TAG, "Humidity:   \t%.1f%%.\t Avg: %.1f%%\n", measurements.humidity, measurements.humid_avg);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // monitor task Stack usage to fine tune StackDepth parameter of task.
        ESP_LOGD(TAG, "Stack High Water Mark: %ld Bytes free, running Core: %d", ( unsigned long int ) uxTaskGetStackHighWaterMark( NULL ), xPortGetCoreID() );
    }
}