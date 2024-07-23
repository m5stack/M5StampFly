/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "led.hpp"
#include "sensor.hpp"
#include "rc.hpp"
#include "flight_control.hpp"

uint32_t Led_color       = 0x000000;
uint32_t Led_color2      = 255;
uint32_t Led_color3      = 0x000000;
uint16_t LedBlinkCounter = 0;
CRGB led_esp[1];
CRGB led_onboard[2];

void led_drive(void);
void onboard_led1(CRGB p, uint8_t state);
void onboard_led2(CRGB p, uint8_t state);
void esp_led(CRGB p, uint8_t state);

void led_init(void) {
    FastLED.addLeds<WS2812, PIN_LED_ONBORD, GRB>(led_onboard, 2);
    FastLED.addLeds<WS2812, PIN_LED_ESP, GRB>(led_esp, 1);
    FastLED.setBrightness(15);
}

void led_show(void) {
    FastLED.show(32);
}

void led_drive(void) {
    if (Mode == AVERAGE_MODE) {
        onboard_led1(PERPLE, 1);
        onboard_led2(PERPLE, 1);
    } else if (Mode == AUTO_LANDING_MODE) {
        onboard_led1(GREEN, 1);
        onboard_led2(GREEN, 1);
    } else if (Mode == FLIGHT_MODE) {
        if (Control_mode == ANGLECONTROL) {
            if (Flip_flag == 0)
                Led_color = YELLOW;  // スタビライズモード・マニュアル飛行では黄色
            else
                Led_color = 0xFF9933;  // 宙返りではオレンジ？
        } else
            Led_color = 0xDC669B;  // アクロモード

        if (Throttle_control_mode == 1) Led_color = 0xc71585;  // 高度制御初期
        if (Alt_flag >= 1) Led_color = 0x331155;               // 高度制御モードではピンク
        if (Rc_err_flag == 1) Led_color = 0xff0000;

        if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {
            onboard_led1(Led_color, 1);
            onboard_led2(Led_color, 1);
        } else {
            onboard_led1(POWEROFFCOLOR, 1);
            onboard_led2(Led_color, 1);
        }
    } else if (Mode == PARKING_MODE) {
        if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {
            // イルミネーション
            if (LedBlinkCounter == 0) {  //<10
                if (Led_color2 & 0x800000)
                    Led_color2 = (Led_color2 << 1) | 1;
                else
                    Led_color2 = Led_color2 << 1;
                onboard_led1(Led_color2, 1);
                onboard_led2(Led_color2, 1);
                // if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {onboard_led1(Led_color2, 1);onboard_led2(Led_color2,
                // 1);} else onboard_led(POWEROFFCOLOR,1);
                LedBlinkCounter++;
            }
            LedBlinkCounter++;
            if (LedBlinkCounter > 20) LedBlinkCounter = 0;
        } else {
            // 水色点滅
            if (LedBlinkCounter < 10) {
                onboard_led1(POWEROFFCOLOR, 1);
                onboard_led2(POWEROFFCOLOR, 1);
            } else if (LedBlinkCounter < 200) {
                onboard_led1(POWEROFFCOLOR, 0);
                onboard_led2(POWEROFFCOLOR, 0);
            } else
                LedBlinkCounter = 0;
            LedBlinkCounter++;
        }
    }

    // LED show
    led_show();
    // FastLED.show(128);
}

void onboard_led1(CRGB p, uint8_t state) {
    if (state == 1) {
        led_onboard[0] = p;
    } else {
        led_onboard[0] = 0;
    }
    return;
}

void onboard_led2(CRGB p, uint8_t state) {
    if (state == 1) {
        led_onboard[1] = p;
    } else {
        led_onboard[1] = 0;
    }
    return;
}

void esp_led(CRGB p, uint8_t state) {
    if (state == 1)
        led_esp[0] = p;
    else
        led_esp[0] = 0;
    return;
}
