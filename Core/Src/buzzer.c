/**
 * @file    buzzer.c
 * @brief   Buzzer driver — TIM4 PWM CH1 tone generator
 *
 * Requires TIM4 configured for PWM (CubeMX): any audible frequency works,
 * e.g. 2 kHz. ARR/PSC are set in TIM4's own MX_TIM4_Init() — this driver
 * only touches the duty cycle (CCR) and start/stop, it doesn't reconfigure
 * the timer's frequency.
 */

#include "buzzer.h"

extern TIM_HandleTypeDef htim4;

/* ─── Internal state ─────────────────────────────────────────────────────── */
static BuzzerTone_t _tone        = BUZZER_OFF;
static uint8_t      _pwm_running = 0;
static uint8_t      _on_phase    = 0;   /* 1 = currently sounding, 0 = silent */
static uint32_t      _phase_ms    = 0;   /* ms elapsed in current on/off phase */

/* ─── Internal: PWM on/off helpers ───────────────────────────────────────── */
static void pwm_on(void)
{
    if (!_pwm_running) {
        uint32_t arr  = __HAL_TIM_GET_AUTORELOAD(&htim4);
        uint32_t ccr  = (arr * BUZZER_PWM_DUTY_PCT) / 100U;
        __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL, ccr);
        HAL_TIM_PWM_Start(&htim4, BUZZER_TIM_CHANNEL);
        _pwm_running = 1;
    }
}

static void pwm_off(void)
{
    if (_pwm_running) {
        HAL_TIM_PWM_Stop(&htim4, BUZZER_TIM_CHANNEL);
        _pwm_running = 0;
    }
}

/* ─── Buzzer_Init ─────────────────────────────────────────────────────────── */
void Buzzer_Init(void)
{
    _tone        = BUZZER_OFF;
    _pwm_running = 0;
    _on_phase    = 0;
    _phase_ms    = 0;
    pwm_off();
}

/* ─── Buzzer_SetPattern ───────────────────────────────────────────────────── *
 * Non-blocking. Sets the sustained tone; Buzzer_Tick() does the toggling
 * for DOUBLE/RAPID. Changing tone always restarts the phase timer cleanly
 * so patterns don't start mid-cycle. */
void Buzzer_SetPattern(BuzzerTone_t tone)
{
    if (tone == _tone)
        return;   /* already in this pattern, nothing to reset */

    _tone     = tone;
    _phase_ms = 0;

    switch (_tone)
    {
        case BUZZER_OFF:
            _on_phase = 0;
            pwm_off();
            break;

        case BUZZER_SINGLE:
        case BUZZER_DOUBLE:
        case BUZZER_RAPID:
            _on_phase = 1;
            pwm_on();
            break;

        default:
            _tone     = BUZZER_OFF;
            _on_phase = 0;
            pwm_off();
            break;
    }
}

/* ─── Buzzer_Tick ─────────────────────────────────────────────────────────── *
 * Call once per main-loop pass at a known cadence (matches the EV 10ms tick
 * is fine, or hook it to TIM3's 100ms scheduler tick — just be consistent
 * about what "1 tick" means here). This implementation assumes it's called
 * every 10ms; adjust BUZZER_*_MS constants in buzzer.h if your call cadence
 * differs. */
void Buzzer_Tick(void)
{
    const uint32_t TICK_MS = 10U;

    switch (_tone)
    {
        case BUZZER_OFF:
            return;   /* nothing to do */

        case BUZZER_SINGLE:
            /* One beep, then silence until SetPattern is called again */
            if (_on_phase) {
                _phase_ms += TICK_MS;
                if (_phase_ms >= BUZZER_SINGLE_ON_MS) {
                    _on_phase = 0;
                    pwm_off();
                }
            }
            return;

        case BUZZER_DOUBLE:
            _phase_ms += TICK_MS;
            if (_on_phase && _phase_ms >= BUZZER_DOUBLE_ON_MS) {
                _on_phase = 0;
                _phase_ms = 0;
                pwm_off();
            } else if (!_on_phase && _phase_ms >= BUZZER_DOUBLE_OFF_MS) {
                _on_phase = 1;
                _phase_ms = 0;
                pwm_on();
            }
            return;

        case BUZZER_RAPID:
            _phase_ms += TICK_MS;
            if (_on_phase && _phase_ms >= BUZZER_RAPID_ON_MS) {
                _on_phase = 0;
                _phase_ms = 0;
                pwm_off();
            } else if (!_on_phase && _phase_ms >= BUZZER_RAPID_OFF_MS) {
                _on_phase = 1;
                _phase_ms = 0;
                pwm_on();
            }
            return;

        default:
            return;
    }
}
