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

#ifndef ALT_KALMAN_HPP
#define ALT_KALMAN_HPP

class Mat {};

class Alt_kalman {
    // accel
    float accel = 0.0;

    // state
    float velocity = 0.0, altitude = 0.0, bias = 0.0;
    float velocity_, altitude_, bias_;
    float true_v;

    // Sensor
    float z_sens;

    // Bias beta
    float beta = -0.01;

    // Kalman Gain
    float k1, k2, k3;

    // F
    float f11 = 1.0, f12 = 0.0, f13 = -step;
    float f21 = step, f22 = 1.0, f23 = 0.0;
    float f31 = 0.0, f32 = 0.0, f33 = 1 + beta * step;

    // B
    float b11 = step, b12 = 0.0;
    float b21 = 0.0, b22 = 0.0;
    float b31 = 0.0, b32 = step;

    // H
    float h1 = 0.0, h2 = 1.0, h3 = 0.0;
    ;

    // P
    float p11 = 100.0, p12 = 0.0, p13 = 0.0;
    float p21 = 0.0, p22 = 100.0, p23 = 0.0;
    float p31 = 0.0, p32 = 0.0, p33 = 100.0;

    float p11_, p12_, p13_;
    float p21_, p22_, p23_;
    float p31_, p32_, p33_;

    // Q
    float q1 = 0.1 * 0.1, q2 = (1.0) * (1.0);  // q1=1.0*1.0 q2=1.0*1.0

    // R
    // float R = 0.004*0.004;
    float R = 0.004 * 0.004;

   public:
    // step
    float step = 1.0 / 400.0;
    // state
    float Velocity = 0.0, Altitude = 0.0, Bias = 0.0;

    // Method
    Alt_kalman();
    void update(float z_sens, float accel, float h);
    void set_vel(float v);
    void reset(void);
};

#endif