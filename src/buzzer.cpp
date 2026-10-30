/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "Arduino.h"
#include <driver/ledc.h>
#include "buzzer.h"
#include "arduino_esp32_compat.h"

const int buzzerPin = 40;
const int channel   = 5;

void setup_pwm_buzzer(void) {
#if STAMPFLY_ARDUINO_ESP32_V3
    ledcAttachChannel(buzzerPin, 4000, 8, channel);
#else
    ledcSetup(channel, 4000, 8);        // 配置PWM通道：通道0，频率3000Hz，分辨率8位
    ledcAttachPin(buzzerPin, channel);  // 将PWM通道绑定到GPIO
#endif
}

void buzzer_sound(uint32_t frequency, uint32_t duration_ms) {
#if STAMPFLY_ARDUINO_ESP32_V3
    ledcWriteTone(buzzerPin, frequency);
    ledcWrite(buzzerPin, 127);
#else
    ledcWriteTone(channel, frequency);
    ledcWrite(channel, 127);
#endif

    vTaskDelay(duration_ms / portTICK_PERIOD_MS);

#if STAMPFLY_ARDUINO_ESP32_V3
    ledcWriteTone(buzzerPin, 0);
#else
    ledcWriteTone(channel, 0);
#endif
    digitalWrite(buzzerPin, 0);
}

void beep(void) {
    buzzer_sound(4000, 100);
}

void start_tone(void) {
    buzzer_sound(NOTE_D1, 200);
    buzzer_sound(NOTE_D5, 200);
    buzzer_sound(NOTE_D3, 200);
    buzzer_sound(NOTE_D4, 200);
}