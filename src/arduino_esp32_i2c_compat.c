#include "arduino_esp32_i2c_compat.h"

#if STAMPFLY_ARDUINO_ESP32_V3

#include <esp_err.h>
#include <esp32-hal-i2c.h>

#define STAMPFLY_I2C_TIMEOUT_MS 50U

int stampfly_i2c_write(uint8_t i2c_num, uint16_t address, const uint8_t *data, size_t length)
{
    if ((data == NULL) && (length != 0U)) {
        return -1;
    }

    return i2cWrite(i2c_num, address, data, length, STAMPFLY_I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

int stampfly_i2c_read(uint8_t i2c_num, uint16_t address, uint8_t *data, size_t length)
{
    size_t read_count = 0;

    if ((data == NULL) && (length != 0U)) {
        return -1;
    }

    if (i2cRead(i2c_num, address, data, length, STAMPFLY_I2C_TIMEOUT_MS, &read_count) != ESP_OK) {
        return -1;
    }

    return read_count == length ? 0 : -1;
}

int stampfly_i2c_write_read(uint8_t i2c_num,
                            uint16_t address,
                            const uint8_t *write_data,
                            size_t write_length,
                            uint8_t *read_data,
                            size_t read_length)
{
    size_t read_count = 0;

    if (((write_data == NULL) && (write_length != 0U)) ||
        ((read_data == NULL) && (read_length != 0U))) {
        return -1;
    }

    if (i2cWriteReadNonStop(i2c_num,
                            address,
                            write_data,
                            write_length,
                            read_data,
                            read_length,
                            STAMPFLY_I2C_TIMEOUT_MS,
                            &read_count) != ESP_OK) {
        return -1;
    }

    return read_count == read_length ? 0 : -1;
}

#endif
