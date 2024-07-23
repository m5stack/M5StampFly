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
#include "pid.hpp"

PID::PID() {
    m_kp           = 1.0e-8f;
    m_ti           = 1.0e8f;
    m_td           = 0.0f;
    m_eta          = 0.01;
    m_integral     = 0.0f;
    m_differential = 0.0f;
    m_err          = 0.0f;
    m_h            = 0.01f;
}

void PID::set_parameter(float kp, float ti, float td, float eta, float h) {
    m_kp  = kp;
    m_ti  = ti;
    m_td  = td;
    m_eta = eta;
    m_h   = h;
}

void PID::reset(void) {
    m_integral     = 0.0f;
    m_differential = 0.0f;
    m_err          = 0.0f;
    m_err2         = 0.0f;
    m_err3         = 0.0f;
}

void PID::i_reset(void) {
    m_integral = 0.0f;
}
void PID::printGain(void) {
    Serial.printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Eta:%8.4f h:%8.4f\r\n", m_kp, m_ti, m_td, m_eta, m_h);
}

void PID::set_error(float err) {
    m_err = err;
}

float PID::update(float err, float h) {
    float d;
    m_h = h;

    // 積分
    m_integral = m_integral + m_h * (err + m_err) / 2 / m_ti;
    if (m_integral > 30000.0f) m_integral = 30000.0f;
    if (m_integral < -30000.0f) m_integral = -30000.0f;
    // 不完全微分
    m_differential = (2 * m_eta * m_td - m_h) * m_differential / (2 * m_eta * m_td + m_h) +
                     2 * m_td * (err - m_err) / (2 * m_eta * m_td + m_h);
    m_err = err;
    return m_kp * (err + m_integral + m_differential);
}

Filter::Filter() {
    m_state = 0.0f;
    m_T     = 0.0025f;
    m_h     = 0.0025f;
}

void Filter::reset(void) {
    m_state = 0.0f;
}

void Filter::set_parameter(float T, float h) {
    m_T = T;
    m_h = h;
}

float Filter::update(float u, float h) {
    m_h     = h;
    m_state = m_state * m_T / (m_T + m_h) + u * m_h / (m_T + m_h);
    m_out   = m_state;
    return m_out;
}
