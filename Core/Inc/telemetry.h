/**
 * @file    telemetry.h
 * @brief   Binary UART telemetry protocol — Packets 0x01–0x04
 *
 * Frame format (matches the Day-11 schedule's "frame sync 0xAA 0x55,
 * CRC16, payload decode" note — adjust SYNC bytes / CRC poly here AND in
 * the matching Python parser if you change either side):
 *
 *   [0xAA][0x55][TYPE][LEN][PAYLOAD...][CRC16_LO][CRC16_HI]
 *     |     |     |     |      |            \________________/
 *   sync1 sync2  1B    1B   LEN bytes        CRC16-CCITT (poly 0x1021,
 *                                            init 0xFFFF) over
 *                                            TYPE+LEN+PAYLOAD only
 *                                            (not the sync bytes)
 *
 * All multi-byte payload fields are little-endian (native STM32/x86 byte
 * order — no byte-swapping needed on either end since both the Blue Pill
 * and a typical PC are little-endian).
 *
 * Packet types (7.2 / 7.3 / 9.1 of the Requirements & Design Doc):
 *   0x01  EV Metrics     — sent every 100 ms (10 Hz)
 *   0x02  ADAS Alerts    — sent every 100 ms (10 Hz)
 *   0x03  Vehicle State  — sent on state change only
 *   0x04  ACK / Status   — sent on request (shell "status"-style query)
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "common.h"
#include "ev_control.h"
#include "adas.h"
#include "fault.h"

/* ─── Frame constants ────────────────────────────────────────────────────── */
#define TLM_SYNC1           0xAAU
#define TLM_SYNC2           0x55U
#define TLM_CRC_INIT        0xFFFFU
#define TLM_OVERHEAD_BYTES  6U      /* sync1+sync2+type+len+crc_lo+crc_hi */

/* ─── Packet type IDs ────────────────────────────────────────────────────── */
#define TLM_PKT_EV_METRICS   0x01U
#define TLM_PKT_ADAS_ALERTS  0x02U
#define TLM_PKT_VEHICLE_STATE 0x03U
#define TLM_PKT_ACK          0x04U

/* ─── Payload sizes (bytes) — must match the packed struct layouts below ─── */
#define TLM_LEN_EV_METRICS     29U   /* 6×float32 + 1×uint8 + 1×uint32      */
#define TLM_LEN_ADAS_ALERTS    19U   /* 4×float32 + 3×uint8                */
#define TLM_LEN_VEHICLE_STATE   3U   /* state, drive_mode, fault_flags     */
#define TLM_LEN_ACK             9U   /* uptime_ms (4B) + version (5 chars) */

#define TLM_VERSION_STR      "v1.0"  /* 4 chars + implicit pad to 5 bytes  */

/* ─── Payload structs — packed, field order matches doc tables 7.2/7.3 ───── *
 * NOTE: these are convenience views for building/parsing payloads; the
 * actual bytes on the wire are written/read field-by-field in telemetry.c
 * (memcpy from these structs would assume no compiler padding — using
 * __attribute__((packed)) here to guarantee that on GCC/arm-none-eabi-gcc,
 * which is what STM32CubeIDE uses). */
#pragma pack(push, 1)

typedef struct {
    float    speed_kmh;
    float    soc_pct;
    float    torque_nm;
    float    power_kw;
    float    range_km;
    float    motor_temp_c;
    uint8_t  drive_mode;
    uint32_t uptime_ms;
} TLM_EVMetrics_t;

typedef struct {
    float   front_cm;
    float   left_cm;
    float   right_cm;
    uint8_t collision_lvl;
    uint8_t blindspot_l;
    uint8_t blindspot_r;
    float   ttc_sec;
} TLM_AdasAlerts_t;

typedef struct {
    uint8_t state;
    uint8_t drive_mode;
    uint8_t fault_flags;
} TLM_VehicleState_t;

typedef struct {
    uint32_t uptime_ms;
    char     version[5];   /* "v1.0" + NUL */
} TLM_Ack_t;

#pragma pack(pop)

/* ─── API ────────────────────────────────────────────────────────────────── *
 * All Build_* functions write a complete framed packet (sync..crc) into
 * `out` and return the total frame length in bytes, or 0 if `out_cap` is
 * too small. Caller is responsible for transmitting `out[0..return value)`
 * (e.g. via HAL_UART_Transmit_DMA in main.c / uart_shell.c — not done here,
 * this module only builds bytes, it doesn't touch the UART peripheral). */
uint16_t TLM_BuildEVMetrics(uint8_t *out, uint16_t out_cap,
                             const EV_HandleTypeDef *ev, uint32_t uptime_ms);

uint16_t TLM_BuildAdasAlerts(uint8_t *out, uint16_t out_cap,
                              const ADAS_HandleTypeDef *adas);

uint16_t TLM_BuildVehicleState(uint8_t *out, uint16_t out_cap,
                                const EV_HandleTypeDef *ev,
                                const Fault_HandleTypeDef *flt);

uint16_t TLM_BuildAck(uint8_t *out, uint16_t out_cap, uint32_t uptime_ms);

/* CRC16-CCITT (poly 0x1021, init 0xFFFF) — exposed in case the Python side
 * or a unit test wants to verify against the same implementation. */
uint16_t TLM_CRC16(const uint8_t *data, uint16_t len);

#endif /* TELEMETRY_H */
