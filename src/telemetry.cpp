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

#include "telemetry.hpp"

#include "rc.hpp"
#include "led.hpp"
#include "sensor.hpp"
#include "flight_control.hpp"

uint8_t Telem_mode     = 0;
uint8_t Telem_cnt      = 0;
const uint8_t MAXINDEX = 120;
const uint8_t MININDEX = 30;

void telemetry_sequence(void);
void telemetry_sequence_fast(void);
void make_telemetry_header_data(uint8_t* senddata);
void make_telemetry_data(uint8_t* senddata);
void make_telemetry_data_fast(uint8_t* senddata);
void data2log(uint8_t* data_list, float add_data, uint8_t index);
void float2byte(float x, uint8_t* dst);
void append_data(uint8_t* data, uint8_t* newdata, uint8_t index, uint8_t len);
void data_set(uint8_t* datalist, float value, uint8_t* index);
void data_set_uint32(uint8_t* datalist, uint32_t value, uint8_t* index);
void data_set_uint16(uint8_t* datalist, uint16_t value, uint8_t* index);
void data_set_uint8(uint8_t* datalist, uint8_t value, uint8_t* index);

void telemetry(void) {
    uint8_t senddata[MAXINDEX];

    if (Telem_mode == 0) {
        // Send header data
        Telem_mode = 1;
        make_telemetry_header_data(senddata);

        // Send !
        telemetry_send(senddata, sizeof(senddata));
    } else if (Mode > AVERAGE_MODE) {
        const uint8_t N = 10;
        // N回に一度送信
        if (Telem_cnt == 0) telemetry_sequence();
        Telem_cnt++;
        if (Telem_cnt > N - 1) Telem_cnt = 0;
        // telemetry_sequence();
    }
}

void telemetry_sequence(void) {
    uint8_t senddata[MAXINDEX];

    switch (Telem_mode) {
        case 1:
            make_telemetry_data(senddata);
            // Send !
            if (telemetry_send(senddata, sizeof(senddata)) == 1)
                esp_led(0x110000, 1);  // Telemetory Reciver OFF
            else
                esp_led(0x001100, 1);  // Telemetory Reciver ON

            // Telem_mode = 2;
            break;
    }
}

void make_telemetry_header_data(uint8_t* senddata) {
    float d_float;
    uint8_t d_int[4];
    uint8_t index = 0;

    index = 2;
    for (uint8_t i = 0; i < (MAXINDEX - 2) / 4; i++) {
        data2log(senddata, 0.0f, index);
        index = index + 4;
    }
    // Telemetry Header
    senddata[0] = 99;
    senddata[1] = 99;
    index       = 2;
    data_set(senddata, Roll_rate_kp, &index);
    data_set(senddata, Roll_rate_ti, &index);
    data_set(senddata, Roll_rate_td, &index);
    data_set(senddata, Roll_rate_eta, &index);
    data_set(senddata, Pitch_rate_kp, &index);
    data_set(senddata, Pitch_rate_ti, &index);
    data_set(senddata, Pitch_rate_td, &index);
    data_set(senddata, Pitch_rate_eta, &index);
    data_set(senddata, Yaw_rate_kp, &index);
    data_set(senddata, Yaw_rate_ti, &index);
    data_set(senddata, Yaw_rate_td, &index);
    data_set(senddata, Yaw_rate_eta, &index);
    data_set(senddata, Rall_angle_kp, &index);
    data_set(senddata, Rall_angle_ti, &index);
    data_set(senddata, Rall_angle_td, &index);
    data_set(senddata, Rall_angle_eta, &index);
    data_set(senddata, Pitch_angle_kp, &index);
    data_set(senddata, Pitch_angle_ti, &index);
    data_set(senddata, Pitch_angle_td, &index);
    data_set(senddata, Pitch_angle_eta, &index);
}

void make_telemetry_data(uint8_t* senddata) {
    float d_float;
    uint8_t d_int[4];
    uint8_t index = 0;

    // Telemetry Header
    senddata[0] = 88;
    senddata[1] = 88;
    index       = 2;
    data_set(senddata, Elapsed_time, &index);                                   // 1 Time
    data_set(senddata, Interval_time, &index);                                  // 2 delta Time
    data_set(senddata, (Roll_angle - Roll_angle_offset) * 180 / PI, &index);    // 3 Roll_angle
    data_set(senddata, (Pitch_angle - Pitch_angle_offset) * 180 / PI, &index);  // 4 Pitch_angle
    data_set(senddata, (Yaw_angle - Yaw_angle_offset) * 180 / PI, &index);      // 5 Yaw_angle
    data_set(senddata, (Roll_rate) * 180 / PI, &index);                         // 6 P
    data_set(senddata, (Pitch_rate) * 180 / PI, &index);                        // 7 Q
    data_set(senddata, (Yaw_rate) * 180 / PI, &index);                          // 8 R
    data_set(senddata, Roll_angle_reference * 180 / PI, &index);                // 9 Roll_angle_reference
    // data_set(senddata, 0.5f * 180.0f *Roll_angle_command, index);
    data_set(senddata, Pitch_angle_reference * 180 / PI, &index);  // 10 Pitch_angle_reference
    // data_set(senddata, 0.5 * 189.0f* Pitch_angle_command, index);
    data_set(senddata, Roll_rate_reference * 180 / PI, &index);    // 11 P ref
    data_set(senddata, Pitch_rate_reference * 180 / PI, &index);   // 12 Q ref
    data_set(senddata, Yaw_rate_reference * 180 / PI, &index);     // 13 R ref
    data_set(senddata, Thrust_command / BATTERY_VOLTAGE, &index);  // 14 T ref
    data_set(senddata, Voltage, &index);                           // 15 Voltage
    data_set(senddata, Accel_x_raw, &index);                       // 16 Accel_x_raw
    data_set(senddata, Accel_y_raw, &index);                       // 17 Accel_y_raw
    data_set(senddata, Accel_z_raw, &index);                       // 18 Accel_z_raw
    data_set(senddata, Alt_velocity, &index);                      // 19 Alt Velocity
    data_set(senddata, Z_dot_ref, &index);                         // 20 Z_dot_ref
    // data_set(senddata, FrontRight_motor_duty, index);
    data_set(senddata, FrontLeft_motor_duty, &index);  // 21 FrontLeft_motor_duty
    data_set(senddata, RearRight_motor_duty, &index);  // 22 RearRight_motor_duty
    // data_set(senddata, RearLeft_motor_duty, index);
    data_set(senddata, Alt_ref, &index);            // 23 Alt_ref
    data_set(senddata, Altitude2, &index);          // 24 Altitude2
    data_set(senddata, Altitude, &index);           // 25 Sense_Alt
    data_set(senddata, Az, &index);                 // 26 Az
    data_set(senddata, Az_bias, &index);            // 27 Az_bias
    data_set_uint8(senddata, Alt_flag, &index);     // 28.1 Alt_flag(1 byte)
    data_set_uint8(senddata, Mode, &index);         // 28.2 fly mode(1 byte)
    data_set_uint16(senddata, RangeFront, &index);  // 28.3-4 tof front
}

void telemetry_fast(void) {
    uint8_t senddata[MAXINDEX];

    if (Telem_mode == 0) {
        // Send header data
        Telem_mode = 1;
        make_telemetry_header_data(senddata);

        // Send !
        telemetry_send(senddata, sizeof(senddata));
    }
    // else if(Mode > AVERAGE_MODE)
    //{
    //   telemetry_sequence400();
    // }
    else if (Mode > AVERAGE_MODE) {
        const uint8_t N = 8;
        // N回に一度送信
        if (Telem_cnt == 0) telemetry_sequence_fast();
        Telem_cnt++;
        if (Telem_cnt > N - 1) Telem_cnt = 0;
        // telemetry_sequence();
    }
}

void telemetry_sequence_fast(void) {
    uint8_t senddata[MAXINDEX];

    make_telemetry_data_fast(senddata);
    // Send !
    if (telemetry_send(senddata, MININDEX) == 1)
        esp_led(0x110000, 1);  // Telemetory Reciver OFF
    else
        esp_led(0x001100, 1);  // Telemetory Reciver ON
}

void make_telemetry_data_fast(uint8_t* senddata) {
    float d_float;
    uint8_t d_int[4];
    uint8_t index = 0;

    // Telemetry Header
    senddata[0] = 88;
    senddata[1] = 88;
    index       = 2;

    data_set(senddata, Elapsed_time, &index);  // 1 Time
    data_set(senddata, Mode, &index);          // 3 Accel_z
    data_set(senddata, Alt_flag, &index);      // 2 Accel_z_raw
    data_set(senddata, RawRange / 1000.0, &index);
    data_set(senddata, Altitude, &index);
    data_set(senddata, Altitude2, &index);
    data_set(senddata, Alt_ref, &index);
}

void data_set(uint8_t* datalist, float value, uint8_t* index) {
    data2log(datalist, value, *index);
    *index = *index + 4;
}

void data_set_uint32(uint8_t* datalist, uint32_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 4);
    *index = *index + 4;
}

void data_set_uint16(uint8_t* datalist, uint16_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 2);
    *index = *index + 2;
}

void data_set_uint8(uint8_t* datalist, uint8_t value, uint8_t* index) {
    append_data(datalist, (uint8_t*)&value, *index, 1);
    *index = *index + 1;
}

void data2log(uint8_t* data_list, float add_data, uint8_t index) {
    uint8_t d_int[4];
    float d_float = add_data;
    float2byte(d_float, d_int);
    append_data(data_list, d_int, index, 4);
}

void float2byte(float x, uint8_t* dst) {
    uint8_t* dummy;
    dummy  = (uint8_t*)&x;
    dst[0] = dummy[0];
    dst[1] = dummy[1];
    dst[2] = dummy[2];
    dst[3] = dummy[3];
}

void append_data(uint8_t* data, uint8_t* newdata, uint8_t index, uint8_t len) {
    for (uint8_t i = index; i < index + len; i++) {
        data[i] = newdata[i - index];
    }
}
