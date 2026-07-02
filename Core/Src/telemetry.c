/**
 * @file    telemetry.c
 * @brief   Binary UART telemetry protocol — packet builders + CRC16
 *
 * Bytes are written field-by-field with memcpy of each primitive, not by
 * memcpy-ing a whole struct over the wire. This sidesteps any possible
 * struct-padding mismatch between this firmware build and whatever decodes
 * it on the PC side (Python's `struct` module, a different compiler, etc.)
 * — the on-wire format is defined entirely by the order of calls below,
 * not by the in-memory layout of TLM_*_t. Those structs in telemetry.h
 * exist purely so calling code has named fields to fill in; they are not
 * memcpy'd directly anywhere in this file.
 */

#include "telemetry.h"

/* ─── CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) ─────────────── *
 * This is the common "CRC-CCITT FALSE" variant. If your Python parser uses
 * a different CRC16 (e.g. CRC16-IBM/Modbus, poly 0x8005), swap this table
 * or use a software bit-loop with the matching polynomial — just make sure
 * both ends agree, or every packet will look corrupted. */
uint16_t TLM_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = TLM_CRC_INIT;

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U)
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ─── Internal: little-endian field writers ──────────────────────────────── *
 * STM32 (Cortex-M3) and a typical PC are both little-endian, so these are
 * just byte-order-explicit memcpy's — written this way so the wire format
 * is correct even if this code ever runs on a big-endian target. */
static uint16_t put_u8(uint8_t *buf, uint16_t off, uint8_t v)
{
    buf[off] = v;
    return off + 1U;
}

static uint16_t put_u32(uint8_t *buf, uint16_t off, uint32_t v)
{
    buf[off + 0] = (uint8_t)( v        & 0xFFU);
    buf[off + 1] = (uint8_t)((v >> 8)  & 0xFFU);
    buf[off + 2] = (uint8_t)((v >> 16) & 0xFFU);
    buf[off + 3] = (uint8_t)((v >> 24) & 0xFFU);
    return off + 4U;
}

static uint16_t put_f32(uint8_t *buf, uint16_t off, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));   /* type-pun via memcpy — no UB,
                                           no aliasing warnings */
    return put_u32(buf, off, bits);
}

/* ─── Internal: common frame wrapper ─────────────────────────────────────── *
 * Writes sync+type+len+payload+crc into `out`, given a payload that's
 * already been written starting at out[4] (right after sync1/sync2/type/
 * len). Returns total frame length, or 0 if it wouldn't fit in out_cap. */
static uint16_t frame_finish(uint8_t *out, uint16_t out_cap,
                              uint8_t type, uint16_t payload_len)
{
    uint16_t frame_len = (uint16_t)(payload_len + TLM_OVERHEAD_BYTES);
    if (frame_len > out_cap)
        return 0U;

    out[0] = TLM_SYNC1;
    out[1] = TLM_SYNC2;
    out[2] = type;
    out[3] = (uint8_t)payload_len;   /* LEN is payload-only, per the frame
                                         diagram in telemetry.h */

    /* CRC covers TYPE + LEN + PAYLOAD — i.e. out[2 .. 4+payload_len) */
    uint16_t crc = TLM_CRC16(&out[2], (uint16_t)(2U + payload_len));
    out[4 + payload_len]     = (uint8_t)(crc & 0xFFU);        /* CRC_LO */
    out[4 + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFFU); /* CRC_HI */

    return frame_len;
}

/* ─── Packet 0x01 — EV Metrics ───────────────────────────────────────────── */
uint16_t TLM_BuildEVMetrics(uint8_t *out, uint16_t out_cap,
                             const EV_HandleTypeDef *ev, uint32_t uptime_ms)
{
    uint16_t frame_len = (uint16_t)(TLM_LEN_EV_METRICS + TLM_OVERHEAD_BYTES);
    if (frame_len > out_cap)
        return 0U;

    uint16_t off = 4U;   /* payload starts after sync1/sync2/type/len */
    off = put_f32(out, off, ev->speed_kmh);
    off = put_f32(out, off, ev->soc);
    off = put_f32(out, off, ev->motor_torque);
    off = put_f32(out, off, ev->power_kw);
    off = put_f32(out, off, ev->range_km);
    off = put_f32(out, off, ev->motor_temp);
    off = put_u8 (out, off, ev->drive_mode);
    off = put_u32(out, off, uptime_ms);

    return frame_finish(out, out_cap, TLM_PKT_EV_METRICS, TLM_LEN_EV_METRICS);
}

/* ─── Packet 0x02 — ADAS Alerts ──────────────────────────────────────────── */
uint16_t TLM_BuildAdasAlerts(uint8_t *out, uint16_t out_cap,
                              const ADAS_HandleTypeDef *adas)
{
    uint16_t frame_len = (uint16_t)(TLM_LEN_ADAS_ALERTS + TLM_OVERHEAD_BYTES);
    if (frame_len > out_cap)
        return 0U;

    uint16_t off = 4U;
    off = put_f32(out, off, adas->front_cm);
    off = put_f32(out, off, adas->left_cm);
    off = put_f32(out, off, adas->right_cm);
    off = put_u8 (out, off, adas->collision_warn);   /* 0/1/2 = NONE/WARN/CRIT */
    off = put_u8 (out, off, adas->blindspot_left);
    off = put_u8 (out, off, adas->blindspot_right);
    off = put_f32(out, off, adas->ttc_sec);

    return frame_finish(out, out_cap, TLM_PKT_ADAS_ALERTS, TLM_LEN_ADAS_ALERTS);
}

/* ─── Packet 0x03 — Vehicle State (send only on change — caller decides) ── */
uint16_t TLM_BuildVehicleState(uint8_t *out, uint16_t out_cap,
                                const EV_HandleTypeDef *ev,
                                const Fault_HandleTypeDef *flt)
{
    uint16_t frame_len = (uint16_t)(TLM_LEN_VEHICLE_STATE + TLM_OVERHEAD_BYTES);
    if (frame_len > out_cap)
        return 0U;

    uint16_t off = 4U;
    off = put_u8(out, off, (uint8_t)ev->state);
    off = put_u8(out, off, ev->drive_mode);
    off = put_u8(out, off, flt->flags);

    return frame_finish(out, out_cap, TLM_PKT_VEHICLE_STATE,
                         TLM_LEN_VEHICLE_STATE);
}

/* ─── Packet 0x04 — ACK / Status (send on request, e.g. shell command) ──── */
uint16_t TLM_BuildAck(uint8_t *out, uint16_t out_cap, uint32_t uptime_ms)
{
    uint16_t frame_len = (uint16_t)(TLM_LEN_ACK + TLM_OVERHEAD_BYTES);
    if (frame_len > out_cap)
        return 0U;

    uint16_t off = 4U;
    off = put_u32(out, off, uptime_ms);

    const char *ver = TLM_VERSION_STR;
    uint8_t i;
    for (i = 0; i < 4 && ver[i] != '\0'; i++)
        off = put_u8(out, off, (uint8_t)ver[i]);
    for (; i < 5; i++)
        off = put_u8(out, off, 0U);   /* NUL-pad to fixed 5-byte field */

    return frame_finish(out, out_cap, TLM_PKT_ACK, TLM_LEN_ACK);
}
