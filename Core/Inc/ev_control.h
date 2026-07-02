/**
 * @file    ev_control.h
 * @brief   Electric Vehicle dynamics model — torque, speed, SOC, range
 */

#ifndef EV_CONTROL_H
#define EV_CONTROL_H


/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

#include "common.h"

/* ─── EV Handle ──────────────────────────────────────────────────────────── */
typedef struct {
    float   accel_pedal;    /* 0–100 %                */
    float   brake_pedal;    /* 0–100 %                */
    float   speed_kmh;      /* 0–200 km/h             */
    float   motor_torque;   /* Nm  (neg = regen)      */
    float   motor_temp;     /* °C  (ADC simulated)    */
    float   soc;            /* 0–100 %                */
    float   regen_level;    /* 0–100 %                */
    float   power_kw;       /* + drive, − regen       */
    float   range_km;       /* estimated range        */
    uint8_t drive_mode;     /* ECO / NORMAL / SPORT   */
    VehicleState_t state;   /* PARKED/READY/DRIVING/REGEN/FAULT (common.h) */
} EV_HandleTypeDef;

/* ─── Constants ──────────────────────────────────────────────────────────── */
#define EV_MAX_TORQUE_NM        150.0f
#define EV_REGEN_TORQUE_MAX_NM   80.0f
#define EV_BATTERY_CAPACITY_KWH  60.0f
#define EV_MASS_FACTOR         1500.0f
#define EV_MAX_SPEED_KMH        200.0f
#define EV_MAX_MOTOR_TEMP_C      90.0f
#define EV_FAULT_SOC_PCT          2.0f
#define EV_DRAG_COEFF             2.0f

#define EV_SIM_SCALE        50.0f

/* Efficiency Wh/km per mode */
#define EV_EFFICIENCY_ECO    14.0f
#define EV_EFFICIENCY_OTHER  18.0f

/* ─── API ────────────────────────────────────────────────────────────────── */
void EV_Init(EV_HandleTypeDef *ev);
void EV_Update(EV_HandleTypeDef *ev, float dt);
void EV_ReadADC(EV_HandleTypeDef *ev);
void EV_SetDriveMode(EV_HandleTypeDef *ev, uint8_t mode);
void EV_InjectSpeed(EV_HandleTypeDef *ev, float speed_kmh);
void EV_InjectSOC(EV_HandleTypeDef *ev, float soc_pct);
void EV_InjectMotorTemp(EV_HandleTypeDef *ev, float temp_c);
uint16_t Read_ADC_Channel(uint32_t channel);

/* ─── Vehicle FSM control ───────────────────────────────────────────────────
 * Thresholds below match the Requirements & Design Doc v1.0 state table
 * exactly (section 5.3) — note PARKED→READY and READY→DRIVING use
 * *different* pedal thresholds, and DRIVING↔REGEN is intentionally
 * asymmetric (enter at a higher brake % than exit) to prevent chatter
 * right at the boundary:
 *
 *   PARKED → READY    : accel_pedal > EV_READY_PEDAL_PCT   (2%)
 *   READY  → DRIVING  : accel_pedal > EV_DRIVING_PEDAL_PCT (5%)
 *   DRIVING → REGEN   : brake_pedal > EV_REGEN_ENTER_PCT   (10%)
 *   REGEN  → DRIVING  : brake_pedal < EV_REGEN_EXIT_PCT    (5%)
 *   any    → FAULT    : driven externally by Fault_Check() in fault.c
 *   FAULT  → PARKED   : only via Fault_Clear() in fault.c
 *
 * "Accel pedal > 2%" key/start interlock is a placeholder for a real
 * button/key GPIO read — swap it in here if/when hardware has one.
 * ────────────────────────────────────────────────────────────────────────── */
VehicleState_t EV_GetState(const EV_HandleTypeDef *ev);

#define EV_READY_PEDAL_PCT     2.0f   /* accel threshold: PARKED → READY   */
#define EV_DRIVING_PEDAL_PCT   5.0f   /* accel threshold: READY  → DRIVING */
#define EV_REGEN_ENTER_PCT    10.0f   /* brake threshold: DRIVING → REGEN  */
#define EV_REGEN_EXIT_PCT      5.0f   /* brake threshold: REGEN  → DRIVING */

#endif /* EV_CONTROL_H */
