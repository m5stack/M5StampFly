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

#ifndef PID_HPP
#define PID_HPP

class PID {
   private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_eta;
    float m_err, m_err2, m_err3;
    float m_h;

   public:
    float m_differential;
    float m_integral;
    PID();
    void set_parameter(float kp, float ti, float td, float eta, float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    void set_error(float err);
    float update(float err, float h);
};

class Filter {
   private:
    float m_state;
    float m_T;
    float m_h;

   public:
    float m_out;
    Filter();
    void set_parameter(float T, float h);
    void reset(void);
    float update(float u, float h);
};

#endif