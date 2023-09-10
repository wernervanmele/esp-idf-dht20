#pragma once
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "toolbox.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DHT20_I2C_ADDRESS      CONFIG_I2C_ADDRESS
#define DHT20_MEASURE_TIMEOUT   1000

/**
 * @brief measurements struct
 */
typedef struct dht20_data {
    float temperature;
    float humidity;
    uint32_t raw_humid;
    uint32_t raw_temp;
} dht20_data_t;

/**
 * @brief Errors enum
 * Assigned values which can't be confused with temperature or humidity.
 */
typedef enum {
    DHT20_ERROR_READ_TIMEOUT = -549,
    DHT20_ERROR_CHECKSUM,
} dht20_error_t;

/* **************************** */
void dht20_begin(void);
uint8_t dht20_status(void);
float dht20_read_data(dht20_data_t *data);
bool dht20_is_calibrated(void);
/* **************************** */

#ifdef __cplusplus
}
#endif
