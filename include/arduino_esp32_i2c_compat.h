#pragma once

#include <stddef.h>
#include <stdint.h>

#include "arduino_esp32_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

#if STAMPFLY_ARDUINO_ESP32_V3

int stampfly_i2c_write(uint8_t i2c_num, uint16_t address, const uint8_t *data, size_t length);
int stampfly_i2c_read(uint8_t i2c_num, uint16_t address, uint8_t *data, size_t length);
int stampfly_i2c_write_read(uint8_t i2c_num,
                            uint16_t address,
                            const uint8_t *write_data,
                            size_t write_length,
                            uint8_t *read_data,
                            size_t read_length);

#endif

#ifdef __cplusplus
}
#endif
