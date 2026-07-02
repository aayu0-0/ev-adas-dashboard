/* Core/Src/uart_shell.c */
#include "uart_shell.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── Private handles ─────────────────────────────────────────── */
static UART_HandleTypeDef  *_huart = NULL;
static EV_HandleTypeDef    *_ev    = NULL;
static ADAS_HandleTypeDef  *_adas  = NULL;
static Fault_HandleTypeDef *_flt   = NULL;

/* ── Ring buffer ─────────────────────────────────────────────── */
static RingBuf_t  _rb;
static char       _cmd[SHELL_CMD_SIZE];
static uint8_t    _cmd_idx = 0;

/* ── Stream / test-override flags ─────────────────────────────── */
static uint8_t _stream_enabled    = 1;   /* "stream on/off" — default ON */
static uint8_t _blindspot_override = 0;  /* "blindspot on/off" — default OFF */

/* ── Last-RX-byte timestamp, used for FAULT_COM detection ────── */
static uint32_t _last_rx_tick = 0;

/* ── Internal: ring buffer helpers ──────────────────────────── */
static void rb_push(uint8_t b)
{
    if (_rb.count < SHELL_BUF_SIZE) {
        _rb.buf[_rb.head] = b;
        _rb.head = (_rb.head + 1) % SHELL_BUF_SIZE;
        _rb.count++;
    }
}

static uint8_t rb_pop(uint8_t *b)
{
    if (_rb.count == 0) return 0;
    *b = _rb.buf[_rb.tail];
    _rb.tail = (_rb.tail + 1) % SHELL_BUF_SIZE;
    _rb.count--;
    return 1;
}

/* ── Internal: transmit string ───────────────────────────────── */
static void shell_tx(const char *s)
{
    HAL_UART_Transmit(_huart, (uint8_t*)s, strlen(s), 200);
}

/* ── Internal: print full system status ─────────────────────── */
static void print_status(void)
{
    char buf[200];
    int  spd  = (int)_ev->speed_kmh;
    int  spdd = (int)(_ev->speed_kmh * 10) % 10;
    int  soc  = (int)_ev->soc;
    int  socd = (int)(_ev->soc * 10) % 10;
    int  tmp  = (int)_ev->motor_temp;
    int  tmpd = (int)(_ev->motor_temp * 10) % 10;
    int  trq  = (int)_ev->motor_torque;
    int  rng  = (int)_ev->range_km;
    const char *mode_str[] = {"ECO","NORMAL","SPORT"};

    shell_tx("\r\n===== EV ADAS STATUS =====\r\n");

    sprintf(buf, " Speed     : %d.%d km/h\r\n", spd, spdd);
    shell_tx(buf);
    sprintf(buf, " SOC       : %d.%d %%\r\n", soc, socd);
    shell_tx(buf);
    sprintf(buf, " Torque    : %d Nm\r\n", trq);
    shell_tx(buf);
    sprintf(buf, " Motor Temp: %d.%d degC\r\n", tmp, tmpd);
    shell_tx(buf);
    sprintf(buf, " Range     : %d km\r\n", rng);
    shell_tx(buf);
    sprintf(buf, " Drive Mode: %s\r\n", mode_str[_ev->drive_mode]);
    shell_tx(buf);
    sprintf(buf, " Accel     : %d %%\r\n", (int)_ev->accel_pedal);
    shell_tx(buf);
    sprintf(buf, " Brake     : %d %%\r\n", (int)_ev->brake_pedal);
    shell_tx(buf);

    shell_tx("--- ADAS ---\r\n");
    sprintf(buf, " Front     : %d cm\r\n", (int)_adas->front_cm);
    shell_tx(buf);
    sprintf(buf, " Left      : %d cm\r\n", (int)_adas->left_cm);
    shell_tx(buf);
    sprintf(buf, " Right     : %d cm\r\n", (int)_adas->right_cm);
    shell_tx(buf);
    sprintf(buf, " TTC       : %d.%ds\r\n",
            (int)_adas->ttc_sec, (int)(_adas->ttc_sec*10)%10);
    shell_tx(buf);
    sprintf(buf, " Collision : %d\r\n", _adas->collision_warn);
    shell_tx(buf);

    const char *alm_str[] = {"NONE","ADVISORY","WARNING","CRITICAL"};
    sprintf(buf, " Alarm     : %s\r\n",
            alm_str[_adas->alarm_priority]);
    shell_tx(buf);

    shell_tx("--- FAULT ---\r\n");
    sprintf(buf, " Flags     : 0x%02X  Active:%d\r\n",
            _flt->flags, _flt->active);
    shell_tx(buf);
    if (_flt->flags & FAULT_OT)  shell_tx(" [FAULT_OT]  Motor overheat\r\n");
    if (_flt->flags & FAULT_SOC) shell_tx(" [FAULT_SOC] Low battery\r\n");
    if (_flt->flags & FAULT_COL) shell_tx(" [FAULT_COL] Collision critical\r\n");

    shell_tx("==========================\r\n> ");
}

/* ── Internal: parse and execute one command ─────────────────── */
static void process_cmd(char *cmd)
{
    char arg1[32] = {0};
    char arg2[32] = {0};
    float fval = 0.0f;

    /* ── mode <eco|normal|sport> ─────────────────────────────── */
    if (sscanf(cmd, "mode %31s", arg1) == 1) {
        if      (!strcmp(arg1,"eco"))    EV_SetDriveMode(_ev, DRIVE_MODE_ECO);
        else if (!strcmp(arg1,"normal")) EV_SetDriveMode(_ev, DRIVE_MODE_NORMAL);
        else if (!strcmp(arg1,"sport"))  EV_SetDriveMode(_ev, DRIVE_MODE_SPORT);
        else { shell_tx("Unknown mode. Use: eco normal sport\r\n> "); return; }
        shell_tx("OK\r\n> ");
        return;
    }

    /* ── speed set <kmh> ─────────────────────────────────────── */
    if (sscanf(cmd, "speed set %f", &fval) == 1) {
        EV_InjectSpeed(_ev, fval);
        shell_tx("OK\r\n> ");
        return;
    }

    /* ── soc set <pct> ───────────────────────────────────────── */
    if (sscanf(cmd, "soc set %f", &fval) == 1) {
        EV_InjectSOC(_ev, fval);
        shell_tx("OK\r\n> ");
        return;
    }

    /* ── temp set <degC> ─────────────────────────────────────── */
    if (sscanf(cmd, "temp set %f", &fval) == 1) {
        EV_InjectMotorTemp(_ev, fval);
        shell_tx("OK\r\n> ");
        return;
    }

    /* ── obstacle <cm> ───────────────────────────────────────── */
    if (sscanf(cmd, "obstacle %f", &fval) == 1) {
        /* Override front distance by injecting directly */
        _adas->front_cm = CLAMP(fval, 2.0f, 400.0f);
        shell_tx("OK\r\n> ");
        return;
    }

    /* ── obstacle clear ──────────────────────────────────────── */
    if (!strcmp(cmd, "obstacle clear")) {
        _adas->front_cm = 400.0f;
        shell_tx("OK\r\n> ");
        return;
    }

    /* ── fault inject motor ──────────────────────────────────── */
    if (sscanf(cmd, "fault inject %31s", arg1) == 1) {
        if (!strcmp(arg1,"motor")) {
            EV_InjectMotorTemp(_ev, 95.0f);
            shell_tx("Injected motor overheat (95 degC)\r\n> ");
        } else if (!strcmp(arg1,"soc")) {
            EV_InjectSOC(_ev, 1.0f);
            shell_tx("Injected low SOC (1%)\r\n> ");
        } else if (!strcmp(arg1,"col")) {
            _adas->collision_warn = 2;
            shell_tx("Injected collision critical\r\n> ");
        } else {
            shell_tx("Use: fault inject motor|soc|col\r\n> ");
        }
        return;
    }

    /* ── fault clear ─────────────────────────────────────────── */
    if (!strcmp(cmd, "fault clear")) {
        Fault_Clear(_flt, _ev);
        shell_tx("Faults cleared. State: PARKED\r\n> ");
        return;
    }

    /* ── blindspot <on|off> ───────────────────────────────────── */
    if (sscanf(cmd, "blindspot %31s", arg1) == 1) {
        if (!strcmp(arg1, "on")) {
            _blindspot_override = 1;
            shell_tx("Blind spot override: ON\r\n> ");
        } else if (!strcmp(arg1, "off")) {
            _blindspot_override = 0;
            shell_tx("Blind spot override: OFF\r\n> ");
        } else {
            shell_tx("Use: blindspot on|off\r\n> ");
        }
        return;
    }

    /* ── stream <on|off> ──────────────────────────────────────── */
    if (sscanf(cmd, "stream %31s", arg1) == 1) {
        if (!strcmp(arg1, "on")) {
            _stream_enabled = 1;
            shell_tx("Telemetry stream: ON\r\n> ");
        } else if (!strcmp(arg1, "off")) {
            _stream_enabled = 0;
            shell_tx("Telemetry stream: OFF\r\n> ");
        } else {
            shell_tx("Use: stream on|off\r\n> ");
        }
        return;
    }

    /* ── alarm test ──────────────────────────────────────────── */
    if (!strcmp(cmd, "alarm test")) {
        shell_tx("Testing ADVISORY...\r\n");
        Buzzer_SetPattern(BUZZER_SINGLE);
        HAL_Delay(2000);
        HAL_IWDG_Refresh(&hiwdg);   /* this command blocks ~6s total —
                                       refresh here or the 1s IWDG would
                                       reset the MCU mid-demo            */
        shell_tx("Testing WARNING...\r\n");
        Buzzer_SetPattern(BUZZER_DOUBLE);
        HAL_Delay(2000);
        HAL_IWDG_Refresh(&hiwdg);
        shell_tx("Testing CRITICAL...\r\n");
        Buzzer_SetPattern(BUZZER_RAPID);
        HAL_Delay(2000);
        HAL_IWDG_Refresh(&hiwdg);
        Buzzer_SetPattern(BUZZER_OFF);
        shell_tx("Done\r\n> ");
        return;
    }

    /* ── status ──────────────────────────────────────────────── */
    if (!strcmp(cmd, "status")) {
        print_status();
        return;
    }

    /* ── reset ───────────────────────────────────────────────── */
    if (!strcmp(cmd, "reset")) {
        shell_tx("Resetting...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
        return;
    }

    /* ── help ────────────────────────────────────────────────── */
    if (!strcmp(cmd,"help") || !strcmp(cmd,"?")) {
        shell_tx("\r\nCommands:\r\n"
            "  mode <eco|normal|sport>\r\n"
            "  speed set <kmh>\r\n"
            "  soc set <pct>\r\n"
            "  temp set <degC>\r\n"
            "  obstacle <cm>\r\n"
            "  obstacle clear\r\n"
            "  blindspot <on|off>\r\n"
            "  fault inject <motor|soc|col>\r\n"
            "  fault clear\r\n"
            "  alarm test\r\n"
            "  stream <on|off>\r\n"
            "  status\r\n"
            "  reset\r\n"
            "  help\r\n> ");
        return;
    }

    /* unknown */
    shell_tx("Unknown command. Type 'help'\r\n> ");
}

/* ── Shell_Init ──────────────────────────────────────────────── */
void Shell_Init(UART_HandleTypeDef  *huart,
                EV_HandleTypeDef    *ev,
                ADAS_HandleTypeDef  *adas,
                Fault_HandleTypeDef *flt)
{
    _huart = huart;
    _ev    = ev;
    _adas  = adas;
    _flt   = flt;
    memset(&_rb, 0, sizeof(_rb));
    _cmd_idx = 0;
    _stream_enabled     = 1;
    _blindspot_override = 0;
    _last_rx_tick        = HAL_GetTick();
    shell_tx("\r\n=== EV ADAS System Ready ===\r\n");
    shell_tx("Type 'help' for commands\r\n> ");
}

/* ── Shell_PushByte — called from UART RX IT callback ───────── */
void Shell_PushByte(uint8_t byte)
{
    _last_rx_tick = HAL_GetTick();
    rb_push(byte);
}

/* ── Shell_Process — call every main loop iteration ─────────── */
void Shell_Process(void)
{
    uint8_t byte;
    while (rb_pop(&byte)) {

        /* Echo character back */
        HAL_UART_Transmit(_huart, &byte, 1, 10);

        if (byte == '\r' || byte == '\n') {
            if (_cmd_idx > 0) {
                _cmd[_cmd_idx] = '\0';
                shell_tx("\r\n");
                process_cmd(_cmd);
                _cmd_idx = 0;
            } else {
                shell_tx("\r\n> ");
            }
        } else if (byte == 0x08 || byte == 0x7F) {
            /* Backspace */
            if (_cmd_idx > 0) _cmd_idx--;
        } else if (_cmd_idx < SHELL_CMD_SIZE - 1) {
            _cmd[_cmd_idx++] = (char)byte;
        }
    }
}

/* ── Accessors — used by main.c / adas.c, not by this file ──── */
uint8_t Shell_StreamEnabled(void)
{
    return _stream_enabled;
}

uint8_t Shell_BlindspotOverride(void)
{
    return _blindspot_override;
}

/* ── Shell_CommTimeout — used by fault.c for FAULT_COM ────────
 * NOTE: this is one-directional-stream-aware only in the sense that it
 * measures RX inactivity, not TX. Since nothing currently sends bytes
 * back to the MCU unless a human (or a script) is actively typing shell
 * commands, this will read "timed out" almost continuously during normal
 * telemetry streaming. That's consistent with the doc's "warning log
 * only" intent (it doesn't cut power or force FAULT state on its own —
 * see fault.c), but don't be surprised to see FLT: bit 0x10 set most of
 * the time once the dashboard is just listening and not talking back. */
uint8_t Shell_CommTimeout(void)
{
    return ((HAL_GetTick() - _last_rx_tick) > SHELL_COMM_TIMEOUT_MS) ? 1U : 0U;
}
