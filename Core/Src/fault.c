#include "fault.h"
#include "ultrasonic.h"
#include "uart_shell.h"

extern TIM_HandleTypeDef htim1;

void Fault_Init(Fault_HandleTypeDef *flt)
{
    flt->flags  = FAULT_NONE;
    flt->active = 0;
}

void Fault_Check(Fault_HandleTypeDef *flt,
                 EV_HandleTypeDef *ev,
                 ADAS_HandleTypeDef *adas)
{
    uint8_t new_flags = FAULT_NONE;

    /* ── Critical faults — force FAULT state, cut PWM ─────────────────── */
    if (ev->motor_temp >= EV_MAX_MOTOR_TEMP_C)
        new_flags |= FAULT_OT;

    if (ev->soc <= EV_FAULT_SOC_PCT)
        new_flags |= FAULT_SOC;

    if (adas->collision_warn == 2 && HCSR04_IsValid(HCSR04_FRONT))
        new_flags |= FAULT_COL;

    /* ── Warning-only faults ───────────────────────────────────────────
     * Reported in flt->flags / the telemetry FLT: byte, but do NOT force
     * a FAULT state transition or cut motor power. Per the Requirements
     * & Design Doc section 7.5:
     *   "Sensor timeout all 3 (FAULT_SEN): Warning log, degrade gracefully."
     *   "UART comm timeout > 5 s (FAULT_COM): Warning log only."
     *
     * NOTE on FAULT_COM: see the comment on Shell_CommTimeout() in
     * uart_shell.c — since telemetry is currently one-directional
     * (MCU -> host), this bit will be set almost continuously during
     * normal use unless the host periodically sends something back. This
     * is expected given the doc's "warning only" framing (it never trips
     * a hard fault or cuts power), just don't mistake FLT:10 in the
     * telemetry stream for an actual comms problem unless you've wired
     * up a host-side heartbeat/ack. */
    if (!HCSR04_IsValid(HCSR04_FRONT) &&
        !HCSR04_IsValid(HCSR04_LEFT)  &&
        !HCSR04_IsValid(HCSR04_RIGHT))
        new_flags |= FAULT_SEN;

    if (Shell_CommTimeout())
        new_flags |= FAULT_COM;

    flt->flags = new_flags;

    /* Only OT / SOC / COL are safety-critical enough to force a hard
       FAULT state + cut PWM. SEN / COM are advisory-only bits and must
       not by themselves trip the vehicle into FAULT. */
    uint8_t critical = new_flags & (FAULT_OT | FAULT_SOC | FAULT_COL);

    if (critical != FAULT_NONE)
    {
        flt->active = 1;
        ev->state    = STATE_FAULT;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
    }
    else
    {
        flt->active = 0;

        /* Only step OUT of FAULT here — do NOT force DRIVING every tick.
         * Forcing DRIVING unconditionally would fight the vehicle FSM in
         * EV_Update() (e.g. yanking the car out of PARKED/READY/REGEN back
         * into DRIVING on every no-fault tick, since "no fault" is the
         * common case). Once faults clear, drop back to READY and let the
         * normal FSM (EV_Update) decide DRIVING/REGEN from pedal input. */
        if (ev->state == STATE_FAULT)
            ev->state = STATE_READY;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
    }
}

void Fault_Clear(Fault_HandleTypeDef *flt, EV_HandleTypeDef *ev)
{
    flt->flags  = FAULT_NONE;
    flt->active = 0;

    ev->state = STATE_PARKED;

    /* Reset EV to safe state */
    ev->speed_kmh   = 0.0f;
    ev->motor_torque = 0.0f;
    ev->motor_temp  = 25.0f;
    ev->soc         = 80.0f;

    /* Turn off fault LED */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}
