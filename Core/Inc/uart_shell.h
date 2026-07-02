/* Core/Inc/uart_shell.h */
#ifndef UART_SHELL_H
#define UART_SHELL_H

#include "common.h"
#include "ev_control.h"
#include "adas.h"
#include "fault.h"

#define SHELL_BUF_SIZE   128   /* ring buffer size       */
#define SHELL_CMD_SIZE    64   /* max command length     */
#define SHELL_COMM_TIMEOUT_MS  5000U  /* FAULT_COM threshold (doc 7.5: >5s) */

/* ── Ring buffer ─────────────────────────────────────────────── */
typedef struct {
    uint8_t  buf[SHELL_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} RingBuf_t;

/* ── API ─────────────────────────────────────────────────────── */
void Shell_Init    (UART_HandleTypeDef  *huart,
                    EV_HandleTypeDef    *ev,
                    ADAS_HandleTypeDef  *adas,
                    Fault_HandleTypeDef *flt);
void Shell_PushByte(uint8_t byte);   /* call from UART RX callback */
void Shell_Process (void);           /* call in main loop          */

/* Gates the periodic 0x01/0x02 telemetry stream — "stream on"/"stream off"
 * shell commands toggle this. main.c's send loop should check it before
 * calling TLM_BuildEVMetrics()/TLM_BuildAdasAlerts() each cycle. Defaults
 * to enabled (1) so telemetry flows without requiring a command first. */
uint8_t Shell_StreamEnabled(void);

/* "blindspot <on/off>" forces a simulated side-vehicle presence for testing,
 * independent of actual sensor readings. main.c / adas.c should OR this
 * into the real blindspot_left/right result if it wants the override to
 * actually light the LEDs and raise the alarm, rather than just being a
 * readback value. */
uint8_t Shell_BlindspotOverride(void);

/* Returns 1 if more than SHELL_COMM_TIMEOUT_MS has elapsed since the last
 * byte was received on USART1 RX, 0 otherwise. Used by fault.c to set the
 * FAULT_COM warning bit (Requirements & Design Doc section 7.5). NOTE:
 * since telemetry currently only flows MCU->host, this will read as
 * "timed out" almost continuously during normal streaming unless the host
 * periodically sends something back — see the comment in fault.c. */
uint8_t Shell_CommTimeout(void);

#endif
