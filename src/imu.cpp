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

#include <Arduino.h>
#include "common.h"
#include "bmi2.h"
#include "imu.hpp"
#include <bmi270.h>

float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
struct bmi2_sens_data imu_data;

void imu_init(void) {
    int8_t st;
    uint8_t data = 0;

    USBSerial.printf("Start IMU Initialize!\n\r");

    pinMode(46, OUTPUT);  // CSを設定
    digitalWrite(46, 1);  // CSをHIGH
    pinMode(12, OUTPUT);  // CSを設定
    digitalWrite(12, 1);  // CSをHIGH
    delay(5);
    USBSerial.printf("SPI Initilize status:%d\n\r", spi_init());

    // BMI270 Init
    bmi270_dev_init();
    st = bmi270_init(pBmi270);
    USBSerial.printf("#INIT Status:%d\n\r", st);
    if (st != 0) {
        USBSerial.printf("BMI270 INIT Fail!\n\r");
        while (1);
    }
    USBSerial.printf("#Chip ID DEV:%02X\n\r", Bmi270.chip_id);
    USBSerial.printf("#APP_STATUS:%02X\n\r", Bmi270.aps_status);

    USBSerial.printf("#INIT_STATUS Read:%d\n\r", bmi2_get_regs(0x21, &data, 1, pBmi270));
    USBSerial.printf("#INIT_STATUS:%02X\n\r", data);
    // IMU Config
    USBSerial.printf("#Config Status:%d\n\r", set_accel_gyro_config(pBmi270));
    uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    USBSerial.printf("#Sensor enable Status:%d\n\r", bmi2_sensor_enable(sensor_list, 2, pBmi270));
}

void imu_update(void) {
    bmi2_get_sensor_data(&imu_data, pBmi270);
}

float imu_get_acc_x(void) {
    return lsb_to_mps2(imu_data.acc.x, 8.0, 16) / GRAVITY_EARTH;
}

float imu_get_acc_y(void) {
    return lsb_to_mps2(imu_data.acc.y, 8.0, 16) / GRAVITY_EARTH;
}

float imu_get_acc_z(void) {
    return lsb_to_mps2(imu_data.acc.z, 8.0, 16) / GRAVITY_EARTH;
}

float imu_get_gyro_x(void) {
    return lsb_to_rps(imu_data.gyr.x, DPS20002RAD, 16);
}

float imu_get_gyro_y(void) {
    return lsb_to_rps(imu_data.gyr.y, DPS20002RAD, 16);
}

float imu_get_gyro_z(void) {
    return lsb_to_rps(imu_data.gyr.z, DPS20002RAD, 16);
}

void imu_test(void) {
    u_long st, now, old, end;
    uint16_t count;
    uint8_t ret;
    st  = micros();
    now = st;
    old = st;
    struct bmi2_sens_data imu_data;
    usleep(1000 * 5000);

    while (1) {
        old = now;
        now = micros();
        ret = bmi2_get_sensor_data(&imu_data, pBmi270);
        // USBSerial.printf("%d\n\r", ret);
        acc_x  = lsb_to_mps2(imu_data.acc.x, 8.0, 16);
        acc_y  = lsb_to_mps2(imu_data.acc.y, 8.0, 16);
        acc_z  = lsb_to_mps2(imu_data.acc.z, 8.0, 16);
        gyro_x = lsb_to_rps(imu_data.gyr.x, DPS10002RAD, 16);
        gyro_y = lsb_to_rps(imu_data.gyr.y, DPS10002RAD, 16);
        gyro_z = lsb_to_rps(imu_data.gyr.z, DPS10002RAD, 16);
#if 1
        USBSerial.printf("%8.4f %7.5f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %d\n\r", (float)(now - st) * 1.0e-6,
                         (float)(now - old) * 1.0e-6, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, ret);
#endif
    }
}
