/**
 * @file    buzzer.h
 * @brief   Buzzer driver — TIM4 PWM CH1 tone generator
 *
 * Two ways to use it:
 *   1. Buzzer_SetPattern(tone) — fire-and-forget, sets the *sustained* tone.
 *      Call this once when alarm_priority changes; call it again with
 *      BUZZER_OFF to silence it. Non-blocking — safe to call from the
 *      10 ms EV loop or the ADAS/fault update path.
 *   2. Buzzer_Tick() — must be called periodically (every main loop pass,
 *      or from a timer ISR) so DOUBLE/RAPID patterns can toggle on/off
 *      over time. SINGLE and OFF don't need ticking, but it's harmless
 *      to call it unconditionally.
 *
 * uart_shell.c's "alarm test" command calls Buzzer_SetPattern() with
 * HAL_Delay() between each tone — that still works fine since SetPattern
 * itself is non-blocking; the delay is just giving you time to hear it.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include "common.h"

/* ─── PWM config ─────────────────────────────────────────────────────────── */
#define BUZZER_TIM_CHANNEL      TIM_CHANNEL_1
#define BUZZER_PWM_DUTY_PCT     50U      /* 50% duty — audible square wave */

/* Tone timing (ms) — used by Buzzer_Tick() to drive on/off toggling */
#define BUZZER_SINGLE_ON_MS     200U     /* one beep, then stays off       */
#define BUZZER_DOUBLE_ON_MS     150U
#define BUZZER_DOUBLE_OFF_MS    150U
#define BUZZER_RAPID_ON_MS       80U
#define BUZZER_RAPID_OFF_MS      80U

/* ─── API ────────────────────────────────────────────────────────────────── */
void Buzzer_Init(void);
void Buzzer_SetPattern(BuzzerTone_t tone);
void Buzzer_Tick(void);   /* call periodically (e.g. every 10ms main loop pass) */

#endif /* BUZZER_H */
