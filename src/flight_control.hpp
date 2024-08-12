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

#ifndef CONTROL_HPP
#define CONTROL_HPP

// #include <Arduino.h>
#include <stdint.h>
#include <math.h>
#include <FastLED.h>
#include <vl53lx_platform.h>

#define BATTERY_VOLTAGE (3.7)
#define PIN_BUTTON      0
#define AVERAGENUM      800

#define INIT_MODE         0
#define AVERAGE_MODE      1
#define FLIGHT_MODE       2
#define PARKING_MODE      3
#define LOG_MODE          4
#define AUTO_LANDING_MODE 5
#define FLIP_MODE         6

#define POWER_LIMIT         3.34
#define UNDER_VOLTAGE_COUNT 100

#define ANGLECONTROL 0
#define RATECONTROL  1

#define AUTO_ALT   4
#define MANUAL_ALT 5

#define ALT_LIMIT   (2.0)
#define ALT_REF_MIN (0.05)
#define ALT_REF_MAX (1.8)

#define RNAGE0FLAG_MAX (20)

#define RATE_RATE (70.0f)
#define RATE_MAX  (1600.0f)
#define RATE_EXPO (0.5f)

// グローバル関数の宣言
void init_copter(void);
void loop_400Hz(void);
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

// グローバル変数
extern volatile uint8_t Mode;
extern volatile uint8_t Loop_flag;
extern float Control_period;
extern volatile float Elapsed_time;

// PID Gain
// Rate control PID gain
extern const float Roll_rate_kp;
extern const float Roll_rate_ti;
extern const float Roll_rate_td;
extern const float Roll_rate_eta;

extern const float Pitch_rate_kp;
extern const float Pitch_rate_ti;
extern const float Pitch_rate_td;
extern const float Pitch_rate_eta;

extern const float Yaw_rate_kp;
extern const float Yaw_rate_ti;
extern const float Yaw_rate_td;
extern const float Yaw_rate_eta;

// Angle control PID gain
extern const float Rall_angle_kp;
extern const float Rall_angle_ti;
extern const float Rall_angle_td;
extern const float Rall_angle_eta;

extern const float Pitch_angle_kp;
extern const float Pitch_angle_ti;
extern const float Pitch_angle_td;
extern const float Pitch_angle_eta;

// Altitude control PID gain
extern const float alt_kp;
extern const float alt_ti;
extern const float alt_td;
extern const float alt_eta;
extern const float alt_period;

extern volatile float Interval_time;

// Offset
extern volatile float Roll_angle_offset, Pitch_angle_offset, Yaw_angle_offset;
extern volatile float Elevator_center, Aileron_center, Rudder_center;

// 制御目標
// PID Control reference
// 角速度目標値
// Rate reference
extern volatile float Roll_rate_reference, Pitch_rate_reference, Yaw_rate_reference;
// 角度目標値
// Angle reference
extern volatile float Roll_angle_reference, Pitch_angle_reference, Yaw_angle_reference;
// 舵角指令値
// Commanad
// スロットル指令値
// Throttle
extern volatile float Thrust_command;
// 角速度指令値
// Rate command
extern volatile float Roll_rate_command, Pitch_rate_command, Yaw_rate_command;
// 角度指令値
// Angle comannd
extern volatile float Roll_angle_command, Pitch_angle_command, Yaw_angle_command;
// 高度目標
extern volatile float Alt_ref;
// Motor Duty
extern volatile float FrontRight_motor_duty;
extern volatile float FrontLeft_motor_duty;
extern volatile float RearRight_motor_duty;
extern volatile float RearLeft_motor_duty;
// 速度目標Z
extern float Z_dot_ref;

extern uint8_t Control_mode;
extern uint8_t Flip_flag;
extern uint8_t Alt_flag;
extern uint8_t Throttle_control_mode;

extern uint8_t ahrs_reset_flag;
extern uint8_t last_ahrs_reset_flag;
#endif
