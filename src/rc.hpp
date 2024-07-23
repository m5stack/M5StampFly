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

#ifndef RC_HPP
#define RC_HPP

#include <stdio.h>
#include <stdint.h>

// #define MINIJOYC

#define CHANNEL 3

#define RUDDER         0
#define ELEVATOR       1
#define THROTTLE       2
#define AILERON        3
#define LOG            4
#define DPAD_UP        5
#define DPAD_DOWN      6
#define DPAD_LEFT      7
#define DPAD_RIGHT     8
#define BUTTON_ARM     9
#define BUTTON_FLIP    10
#define CONTROLMODE    11
#define ALTCONTROLMODE 12

#define RUDDER_MAX   511
#define RUDDER_MIN   -512
#define ELEVATOR_MAX 127
#define ELEVATOR_MIN -128
#define THROTTLE_MAX 511
#define THROTTLE_MIN -512
#define AILERON_MAX  127
#define AILERON_MIN  -128

#define LOG_MAX 1
#define LOG_MIN 0
#define CH6MAX  127
#define CH6MIN  -128

#define RUDDER_MAX_JOYC   100
#define ELEVATOR_MAX      127
#define THROTTLE_MAX_JOYC 100

void rc_init(void);
void rc_demo(void);
void rc_end(void);
uint8_t rc_isconnected(void);
uint8_t telemetry_send(uint8_t* data, uint16_t datalen);
void send_peer_info(void);

extern volatile float Stick[16];
extern volatile uint8_t Rc_err_flag;
extern volatile uint8_t MyMacAddr[6];
extern volatile uint8_t Recv_MAC[3];
extern volatile uint16_t Connect_flag;
#endif