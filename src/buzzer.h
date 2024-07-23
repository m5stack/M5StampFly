/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef BUZZER_H
#define BUZZER_H

#define NOTE_D1 294
#define NOTE_D2 330
#define NOTE_D3 350
#define NOTE_D4 393
#define NOTE_D5 441
#define NOTE_D6 495
#define NOTE_D7 556

void setup_pwm_buzzer(void);
void beep(void);
void start_tone(void);
void buzzer_sound(uint32_t frequency, uint32_t duration_ms);

#endif
