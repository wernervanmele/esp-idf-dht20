#include <stdio.h>
#include "toolbox.h"

static esp_err_t _i2c_write_read(uint8_t i2c_addr, uint8_t read_reg, uint8_t *read_buffer, size_t read_size, uint16_t delayms);
//static esp_err_t _i2c_write(uint8_t i2c_addr, uint8_t *write_buffer, size_t write_size);
//static esp_err_t _i2c_read(uint8_t i2c_addr, uint8_t *read_buffer, size_t read_size);

/* ************ HELPER FUNCS ***************** */

/**
 * @brief
 * @param
 * @return
 */
esp_err_t _i2c_init(void)
{
    static esp_err_t err = ESP_OK;
    /**
     * Setup I2C Config for ADXL345 Sensor
     * If your didn't please run idf.py menuconfig
    */
    int i2c_master_port = I2C_MASTER_PORT;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_GPIO_SDA,         // select SDA GPIO specific to your project
        .sda_pullup_en = I2C_GPIO_SDA_PULLUP,
        .scl_io_num = I2C_GPIO_SCL,         // select SCL GPIO specific to your project
        .scl_pullup_en = I2C_GPIO_SCL_PULLUP,
        .master.clk_speed = I2C_BUS_SPEED,  // select frequency specific to your project
        .clk_flags = 0,                          // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };

    ESP_LOGI(__func__, "Initializing I2C Bus.......");
    err = i2c_param_config(i2c_master_port, &conf);
    err += i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    ESP_LOGI(__func__, "I2C Bus Initialized, %s", esp_err_to_name(err));

    return err;
}

/**
 * @brief
 * @param i2c_addr
 * @param read_reg
 * @param read_buffer
 * @param read_size
 * @return
 */
esp_err_t i2c_read(uint8_t i2c_addr, uint8_t *read_buffer, size_t read_size)
{
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, i2c_addr << 1 | I2C_MASTER_READ, true);
    i2c_master_read(handle, read_buffer, read_size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, handle, pdMS_TO_TICKS(I2C_CMD_TIMEOUT));

    i2c_cmd_link_delete(handle);

    return err;
}

/**
 * @brief I2C Read function with configurable delay between write and read, can be useful if you need
 * to allow some conversion time for certain sensors. (eg CCS811)
 * @param read_buffer
 * @param read_size
 * @param delayms
 * @return
 */
static esp_err_t _i2c_write_read(uint8_t i2c_addr, uint8_t read_reg, uint8_t *read_buffer, size_t read_size, uint16_t delayms)
{

    esp_err_t err = ESP_OK;

    // ESP_LOGD(__func__, "reg_buf values: %s", print_byte(read_reg));

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    i2c_master_start(handle);
    i2c_master_write_byte(handle, i2c_addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(handle, &read_reg, 1, true);
    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, handle, pdMS_TO_TICKS(I2C_CMD_TIMEOUT));
    i2c_cmd_link_delete(handle);

    if (delayms > 0) {
        portDISABLE_INTERRUPTS();
        ets_delay_us(delayms * 1000);
        portENABLE_INTERRUPTS();
    }

    handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, i2c_addr << 1 | I2C_MASTER_READ, true);
    i2c_master_read(handle, read_buffer, read_size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(handle);
    err += i2c_master_cmd_begin(I2C_MASTER_PORT, handle, pdMS_TO_TICKS(I2C_CMD_TIMEOUT));

    i2c_cmd_link_delete(handle);
    // ESP_LOGD(__func__, "read_buffer values: %s", print_byte(read_buffer));
    return err;
}

/**
 * @brief I2C Write function with dynamic adaptable write buffer
 * @param writereg
 * @param write_buffer
 * @param write_size
 * @return
 */
__unused
esp_err_t i2c_write(uint8_t i2c_addr, uint8_t *write_buffer, size_t write_size)
{
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert (handle != NULL);

    i2c_master_start(handle);
    i2c_master_write_byte(handle, i2c_addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(handle, write_buffer, write_size, true);

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, handle, pdMS_TO_TICKS(I2C_CMD_TIMEOUT));

    i2c_cmd_link_delete(handle);
    return err;
}

/**
 * @brief I2C Read from register
 * @param read_reg
 * @param read_buffer
 * @param read_size
 * @param delayms
 */
void i2c_write_read(uint8_t i2c_addr, uint8_t read_reg, uint8_t *read_buffer, size_t read_size, uint16_t delayms)
{
    _i2c_write_read(i2c_addr, read_reg, read_buffer, read_size, delayms);
}

/**
 * @brief I2C Write to register
 * @param writereg
 * @param write_buffer
 * @param write_size
 */
// void i2c_write(uint8_t i2c_addr, uint8_t *write_buffer, size_t write_size)
// {
//     _i2c_write( i2c_addr, write_buffer,  write_size);
// }

/**
 * @brief
 * @param i2c_addr
 * @param read_reg
 * @param
 * @param read_buffer
 * @param read_size
 */
// void i2c_read(uint8_t i2c_addr, uint8_t *read_buffer, size_t read_size)
// {
//     _i2c_read( i2c_addr, read_buffer,  read_size);
// }
/* ------------------------------------------------------------------------- */

/**
 * @brief
 * @param value
 */
char *print_binary(uint16_t value)
{
    static char binary_str[17];  // 16 bits + null-terminator
    binary_str[16] = '\0'; // Null-terminate the string

    for (int i = 15; i >= 0; i--) {
        binary_str[15 - i] = ((value >> i) & 1) ? '1' : '0';
    }

    return binary_str;
}

// binary values lookup table
const char *bit_rep[16] = {
    [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
    [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
    [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

/**
 * @brief
 * @param byte
 * @return
 */
__attribute__((unused))
char *print_byte(uint8_t byte)
{
    static char binbyte[11];
    sprintf(binbyte, "0b%s %s", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);

    return binbyte;
}

/**
 * @brief
 * @param val
 * @return
 */
uint8_t bcd2dec(uint8_t val)
{
    //return val - 6 * (val >> 4);
    return (val >> 4) * 10 + (val & 0x0f);
}

/**
 * @brief
 * @param val
 * @return
 */
uint8_t dec2bcd(uint8_t val)
{
    //return val + 6 * (val / 10);
    return ((val / 10) << 4) + (val % 10);
}

/**
 * @brief
 * @param x
 * @param in_min
 * @param in_max
 * @param out_min
 * @param out_max
 * @return
 */
double map(double x, double in_min, double in_max, double out_min, double out_max)
{
    const double run = in_max - in_min;
    if (run == 0) {
        ESP_LOGE(__func__, "map(): Invalid input range, min == max");
        return -1; // AVR returns -1, SAM returns 0
    }
    const double rise = out_max - out_min;
    const double delta = x - in_min;
    return (delta * rise) / run + out_min;
}