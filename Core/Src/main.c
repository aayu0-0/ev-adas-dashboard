/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "ev_control.h"
#include "adas.h"
#include "fault.h"
#include "buzzer.h"
#include "ultrasonic.h"
#include "uart_shell.h"
/* telemetry.h intentionally NOT included here anymore — see note in the
   100ms tick block below for why the binary CRC16 protocol was removed
   from main.c's send loop. telemetry.c/h are left in the project untouched
   in case a binary stream is wanted again later (e.g. once the Python
   dashboard is built and you switch off the ASCII protocol). */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* ── Module instances ─────────────────────────────────────────────────── */
static EV_HandleTypeDef    ev;
static ADAS_HandleTypeDef  adas;
static Fault_HandleTypeDef flt;

/* ── Scheduler flags — set in HAL_TIM_PeriodElapsedCallback (TIM3, 100ms) */
static volatile uint8_t flag_100ms = 0;

/* ── Tick counters for sub-dividing the 100ms base tick ──────────────── */
static uint32_t uptime_ms   = 0;   /* free-running uptime, incremented every 100ms tick by 100 */
static uint8_t  div_ultrasonic = 0;/* ultrasonic read is slow (3 * ~30ms worst case) — every 2 ticks (200ms) */
static uint8_t  div_telemetry  = 0;/* ASCII telemetry send — every 10 ticks (1000ms / 1Hz) */

/* ── USART1 RX single-byte interrupt buffer ──────────────────────────── */
static uint8_t uart_rx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  TIM period-elapsed callback — shared by all TIMs using HAL_TIM_Base_Start_IT.
  *         Only TIM3 (100ms base tick) is started this way, so no need to check
  *         which timer fired, but we check anyway in case TIM1 (PWM) interrupt
  *         is ever started here too, to avoid silently misfiring our flag.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        flag_100ms = 1;
    }
}

/**
  * @brief  USART1 RX-complete callback — fires once per received byte since
  *         we re-arm HAL_UART_Receive_IT(huart1, &uart_rx_byte, 1) each time.
  *         Feeds the shell's ring buffer; Shell_Process() (called from the
  *         main loop) parses completed lines.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        Shell_PushByte(uart_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);   /* re-arm for next byte */
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* MX_IWDG_Init() intentionally NOT called here — CubeMX auto-generates
     this call in the standard peripheral init sequence, but we want the
     watchdog armed AFTER module init (EV/ADAS/Shell/sensors), not before.
     See the real call in USER CODE 2, right before the main loop.
     NOTE: regenerating code via CubeMX will re-insert a call here — if
     that happens, delete it again and keep only the one in USER CODE 2. */
  /* USER CODE BEGIN 2 */

  /* ── Module init ──────────────────────────────────────────────────── */
  EV_Init(&ev);
  ADAS_Init(&adas);
  Fault_Init(&flt);
  Buzzer_Init();
  HCSR04_Init();
  Shell_Init(&huart1, &ev, &adas, &flt);

  /* ── Start free-running timers ────────────────────────────────────── */
  HAL_TIM_Base_Start(&htim2);          /* µs tick for ultrasonic timing, no IRQ needed */
  HAL_TIM_Base_Start_IT(&htim3);       /* 100ms scheduler tick -> flag_100ms           */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  /* motor PWM output                       */
  /* NOTE: TIM4 PWM (buzzer) is started/stopped on demand by buzzer.c itself,
     not unconditionally here — see Buzzer_SetPattern()/pwm_on()/pwm_off(). */

  /* ── Arm USART1 RX interrupt (shell input) ────────────────────────── */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  /* ── Start the independent watchdog LAST, right before the main loop.
   *    All init above is fast (<<1s total), so starting it here avoids
   *    any risk of tripping mid-boot. From this point on, the main loop
   *    must call HAL_IWDG_Refresh() at least once every ~1s or the MCU
   *    will reset (see MX_IWDG_Init() for the exact timeout). ─────── */
  MX_IWDG_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ── Watchdog refresh — must happen at least once per ~1s or the
       *    MCU resets. Placed first so it's reached on every pass,
       *    including the many fast passes between 100ms ticks. The one
       *    place in this codebase that blocks longer than 1s in a single
       *    call — uart_shell.c's "alarm test" command (~6s total via
       *    HAL_Delay) — refreshes the watchdog itself between delays,
       *    since it doesn't return here until it's done. ─────────── */
      HAL_IWDG_Refresh(&hiwdg);

      /* ── Shell command processing — every loop pass, cheap & non-blocking */
      Shell_Process();

      /* ── Buzzer pattern toggling — must be ticked regularly for
       *    DOUBLE/RAPID patterns to actually beep on/off. Called every
       *    loop pass; buzzer.c's own internal ms accumulator assumes a
       *    ~10ms-ish cadence (see buzzer.h comment) — Shell_Process() +
       *    the rest of this loop body keep us in roughly that range. */
      Buzzer_Tick();

      /* ── 100ms scheduler tick (set by TIM3 ISR) ───────────────────── */
      if (flag_100ms) {
          flag_100ms = 0;
          uptime_ms += 100;

          /* -- EV physics + ADAS + fault check, every 100ms -- */
          EV_ReadADC(&ev);
          EV_Update(&ev, 0.1f);   /* dt = 100ms = 0.1s */

          /* -- Ultrasonic sensors are slow (TRIG/ECHO round trip can take
           *    up to ~30ms each on timeout x3 sensors = up to ~90ms worst
           *    case) — read every 2nd tick (200ms) so a slow read doesn't
           *    starve the 100ms EV/fault loop on the ticks in between. */
          if (++div_ultrasonic >= 2) {
              div_ultrasonic = 0;
              HCSR04_ReadAll();

              adas.front_cm = HCSR04_GetDistance(HCSR04_FRONT);
              adas.left_cm  = HCSR04_GetDistance(HCSR04_LEFT);
              adas.right_cm = HCSR04_GetDistance(HCSR04_RIGHT);
          }

          ADAS_Update(&adas, &ev);

          /* -- Blind-spot shell test override (forces both sides active,
           *    independent of real sensor readings) -- */
          if (Shell_BlindspotOverride()) {
              adas.blindspot_left  = 1;
              adas.blindspot_right = 1;
          }

          Fault_Check(&flt, &ev, &adas);

          /* -- Buzzer pattern driven by ADAS alarm priority --
           *    collision_warn: 0=none 1=warning 2=critical (adas.h)
           *    Critical -> RAPID, Warning -> DOUBLE, blind-spot-only -> SINGLE,
           *    nothing -> OFF. Fault state overrides everything with RAPID. */
          if (flt.active) {
              Buzzer_SetPattern(BUZZER_RAPID);
          } else if (adas.collision_warn == 2) {
              Buzzer_SetPattern(BUZZER_RAPID);
          } else if (adas.collision_warn == 1) {
              Buzzer_SetPattern(BUZZER_DOUBLE);
          } else if (adas.blindspot_left || adas.blindspot_right) {
              Buzzer_SetPattern(BUZZER_SINGLE);
          } else {
              Buzzer_SetPattern(BUZZER_OFF);
          }

          /* -- Telemetry: plain-ASCII EV + ADAS frames at 1Hz (once per
           *    second — every 10th 100ms tick), gated by "stream on/off"
           *    shell command.
           *
           *    FIX (was the cause of garbled VTerm output): this used to
           *    ALSO build and send the binary CRC16 telemetry packets
           *    (TLM_BuildEVMetrics / TLM_BuildAdasAlerts / TLM_BuildVehicleState
           *    from telemetry.c) on the same UART as this ASCII line. Both
           *    streams interleaved on the wire, so VTerm rendered the raw
           *    binary bytes (sync markers, floats, CRC) as garbage text
           *    mixed in with the readable ASCII lines. Sending ONLY ASCII
           *    here fixes it. If a binary protocol is needed later (e.g.
           *    for the Python dashboard), re-add the TLM_Build* calls but
           *    remove this ASCII block first — don't run both at once. --
           */
          if (Shell_StreamEnabled() && (++div_telemetry >= 10)) {
              div_telemetry = 0;

              char line1[100];
              char line2[100];

              /* Line 1 — EV telemetry (matches presentation slide format) */
              int spd  = (int)ev.speed_kmh;
              int spdd = (int)(ev.speed_kmh  * 10.0f) % 10;
              int soc  = (int)ev.soc;
              int socd = (int)(ev.soc        * 10.0f) % 10;
              int trq  = (int)ev.motor_torque;
              int tmp  = (int)ev.motor_temp;
              int tmpd = (int)(ev.motor_temp * 10.0f) % 10;
              int rng  = (int)ev.range_km;
              int acc  = (int)ev.accel_pedal;
              int brk  = (int)ev.brake_pedal;

              snprintf(line1, sizeof(line1),
                  "SPD:%d.%d SOC:%d.%d TRQ:%d TMP:%d.%d RNG:%d ACC:%d BRK:%d STATE:%d\r\n",
                  spd, spdd, soc, socd, trq, tmp, tmpd, rng, acc, brk, (int)ev.state);
              HAL_UART_Transmit(&huart1, (uint8_t*)line1, strlen(line1), 100);

              /* Line 2 — ADAS telemetry (was previously missing entirely —
                 only the binary 0x02 packet carried this data before). */
              int front = (int)adas.front_cm;
              int left  = (int)adas.left_cm;
              int right = (int)adas.right_cm;
              int ttc   = (int)adas.ttc_sec;
              int ttcd  = (int)(adas.ttc_sec * 10.0f) % 10;
              /* BSD encoded as two digits, left then right (e.g. "10" =
                 left active/right clear), matching the slide's BSD:10 example */
              int bsd   = (adas.blindspot_left ? 10 : 0) + (adas.blindspot_right ? 1 : 0);

              snprintf(line2, sizeof(line2),
                  "F:%d L:%d R:%d TTC:%d.%ds COL:%d BSD:%02d ALM:%d FLT:%02X\r\n",
                  front, left, right, ttc, ttcd, adas.collision_warn, bsd,
                  (int)adas.alarm_priority, flt.flags);
              HAL_UART_Transmit(&huart1, (uint8_t*)line2, strlen(line2), 100);
          }
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 1250;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, US_FRONT_TRIG_Pin|US_LEFT_TRIG_Pin|GPIO_PIN_10|FAULT_RELAY_Pin
                          |US_RIGHT_TRIG_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : US_FRONT_TRIG_Pin US_LEFT_TRIG_Pin PB10 FAULT_RELAY_Pin
                           US_RIGHT_TRIG_Pin PB8 PB9 */
  GPIO_InitStruct.Pin = US_FRONT_TRIG_Pin|US_LEFT_TRIG_Pin|GPIO_PIN_10|FAULT_RELAY_Pin
                          |US_RIGHT_TRIG_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : US_FRONT_ECHO_Pin US_LEFT_ECHO_Pin US_RIGHT_ECHO_Pin */
  GPIO_InitStruct.Pin = US_FRONT_ECHO_Pin|US_LEFT_ECHO_Pin|US_RIGHT_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
