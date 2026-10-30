#pragma once

#include <stdint.h>
#include <esp_arduino_version.h>
#include <esp_idf_version.h>

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#define STAMPFLY_ARDUINO_ESP32_V3 1
#define STAMPFLY_ARDUINO_ESP32_BRANCH_NAME "3.x compatibility branch"
#else
#define STAMPFLY_ARDUINO_ESP32_V3 0
#define STAMPFLY_ARDUINO_ESP32_BRANCH_NAME "legacy compatibility branch"
#endif

#define STAMPFLY_STRINGIFY_IMPL(value) #value
#define STAMPFLY_STRINGIFY(value) STAMPFLY_STRINGIFY_IMPL(value)
#define STAMPFLY_ARDUINO_ESP32_VERSION_STRING \
    STAMPFLY_STRINGIFY(ESP_ARDUINO_VERSION_MAJOR) "." \
    STAMPFLY_STRINGIFY(ESP_ARDUINO_VERSION_MINOR) "." \
    STAMPFLY_STRINGIFY(ESP_ARDUINO_VERSION_PATCH)

#if STAMPFLY_ARDUINO_ESP32_V3
#include <esp_rom_sys.h>

static inline void stampfly_delay_us(uint32_t us)
{
    esp_rom_delay_us(us);
}
#else
#include <rom/ets_sys.h>

static inline void stampfly_delay_us(uint32_t us)
{
    ets_delay_us(us);
}
#endif
#ifdef __cplusplus
#if STAMPFLY_ARDUINO_ESP32_V3

#if !defined(ARDUINO_USB_MODE) || !ARDUINO_USB_MODE
#error "StampFly HWCDC compatibility requires ARDUINO_USB_MODE=1"
#endif

#if !defined(ARDUINO_USB_CDC_ON_BOOT) || !ARDUINO_USB_CDC_ON_BOOT
#error "StampFly HWCDC compatibility requires ARDUINO_USB_CDC_ON_BOOT=1"
#endif

#include <HWCDC.h>

#define USBSerial HWCDCSerial

#endif
#endif
