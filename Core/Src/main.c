/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Platform, TIM2 10 ms ISR & Scheduler)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "scheduler.h"
#include "software_timer.h"
#include "global.h"
#include "app_control.h"

#if !defined(PLATFORM_FULL_INTEGRATION)
#  if defined(__has_include)
#    if __has_include("peripheral_stm32.h") && __has_include("button.h") \
        && __has_include("traffic_light.h") && __has_include("dht20.h") \
        && __has_include("lcd.h") && __has_include("display.h") \
        && __has_include("fsm_tuning.h")
#      define PLATFORM_FULL_INTEGRATION 1
#    else
#      define PLATFORM_FULL_INTEGRATION 0
#    endif
#  else
#    define PLATFORM_FULL_INTEGRATION 0
#  endif
#endif

#if PLATFORM_FULL_INTEGRATION
#include "peripheral_stm32.h"
#include "button.h"
#include "traffic_light.h"
#include "dht20.h"
#include "lcd.h"
#include "display.h"
#include "fsm_tuning.h"
void app_control_run(void);
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Define BOARD_DIAG_STEP3 (e.g. via compiler -DBOARD_DIAG_STEP3) ONLY during
 * isolated Step 3 hardware bring-up (H01-H04). Disabled in normal builds so
 * no HAL_Delay exists in the scheduled firmware.
 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Live-Expression measurement variables for acceptance tests H04, S01, S02, S05 */
volatile uint32_t g_last_isr_cycles = 0U;
volatile uint32_t g_max_isr_cycles = 0U;
volatile uint16_t g_last_isr_us = 0U;
volatile uint16_t g_max_isr_us = 0U;
volatile uint32_t g_oneshot_fired_count = 0U;
volatile uint8_t g_i2c_found_addrs[8] = {0};
volatile uint8_t g_i2c_found_count = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void platform_enable_cycle_counter(void);
static bool platform_register_tasks(void);
#if !PLATFORM_FULL_INTEGRATION
static void toggle_PA5(void);
static void toggle_PC9(void);
static void one_shot_test(void);
#endif
#if defined(BOARD_DIAG_STEP3)
static void platform_run_step3_hardware_diag(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void platform_enable_cycle_counter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

#if PLATFORM_FULL_INTEGRATION
/*
 * Explicit enum translation between Member 3 (global.h) and Member 2 (button.h,
 * traffic_light.h, display.h) so differing enumerator orders cannot mismatch.
 */
static app_event_t platform_button_take_event(uint8_t id)
{
    switch (button_take_event(id)) {
    case BUTTON_PRESS: return APP_EVENT_PRESS;
    case BUTTON_LONG:  return APP_EVENT_LONG;
    case BUTTON_NONE:
    default:           return APP_EVENT_NONE;
    }
}

static light_t platform_map_light(app_light_t light)
{
    switch (light) {
    case APP_GREEN: return LIGHT_GREEN;
    case APP_AMBER: return LIGHT_AMBER;
    case APP_RED:
    default:        return LIGHT_RED;
    }
}

static bool platform_traffic_apply_pair(app_light_t road1, app_light_t road2)
{
    return traffic_apply_pair(platform_map_light(road1), platform_map_light(road2));
}

static display_view_t platform_read_display_view(void)
{
    app_snapshot_t snap = app_get_snapshot();
    display_view_t view = {0};

    switch (snap.mode) {
    case APP_MANUAL: view.mode = DISPLAY_MANUAL; break;
    case APP_TUNING: view.mode = DISPLAY_TUNING; break;
    case APP_AUTO:
    default:         view.mode = DISPLAY_AUTO;   break;
    }

    switch (snap.phase) {
    case PHASE_P0: view.phase = DISPLAY_ROAD1_GREEN; break;
    case PHASE_P1: view.phase = DISPLAY_ROAD1_AMBER; break;
    case PHASE_P2: view.phase = DISPLAY_ROAD2_GREEN; break;
    case PHASE_P3: view.phase = DISPLAY_ROAD2_AMBER; break;
    case PHASE_ALL_RED:
    default:       view.phase = DISPLAY_ALL_RED;     break;
    }

    view.remaining_s = (uint16_t)((snap.phase_remaining_ms + 999U) / 1000U);

    switch (snap.selected) {
    case PARAM_GREEN:
        view.tuning_field = 'G';
        view.tuning_value_s = snap.draft.green_s;
        break;
    case PARAM_AMBER:
        view.tuning_field = 'Y';
        view.tuning_value_s = snap.draft.amber_s;
        break;
    case PARAM_RED:
    default:
        view.tuning_field = 'R';
        view.tuning_value_s = snap.draft.red_s;
        break;
    }

    switch (snap.tuning_status) {
    case TUNING_SAVED:   view.tuning_status = DISPLAY_SAVED;   break;
    case TUNING_INVALID: view.tuning_status = DISPLAY_INVALID; break;
    case TUNING_EDITING:
    default:             view.tuning_status = DISPLAY_EDITING; break;
    }

    return view;
}

static const app_services_t g_app_services = {
    .button_take_event = platform_button_take_event,
    .timer_start = timer_start,
    .timer_cancel = timer_cancel,
    .timer_take_expired = timer_take_expired,
    .traffic_apply_pair = platform_traffic_apply_pair,
    .traffic_all_red = traffic_all_red,
    .timer_remaining_ms = timer_remaining_ms
};

static bool platform_register_tasks(void)
{
    static const peripheral_pin_t button_pins[BUTTON_COUNT] = {
        {BTN_1_GPIO_Port, BTN_1_Pin},
        {BTN_2_GPIO_Port, BTN_2_Pin},
        {BTN_3_GPIO_Port, BTN_3_Pin},
        {BTN_4_GPIO_Port, BTN_4_Pin}
    };
    /* Active-LOW buttons with internal GPIO_PULLUP (confirm in H03) */
    static const bool button_active_high[BUTTON_COUNT] = {false, false, false, false};
    static const peripheral_pin_t light_pins[4] = {
        {TL1_A_GPIO_Port, TL1_A_Pin},
        {TL1_B_GPIO_Port, TL1_B_Pin},
        {TL2_A_GPIO_Port, TL2_A_Pin},
        {TL2_B_GPIO_Port, TL2_B_Pin}
    };
    /* Default 2-bit light encoding table pending physical H02 measurement */
    static const traffic_codes_t light_codes[2] = {
        {.red = 3U, .amber = 1U, .green = 2U},
        {.red = 3U, .amber = 1U, .green = 2U}
    };
    peripheral_i2c_t i2c_bus = peripheral_stm32_i2c(&hi2c1);

    if (!peripheral_stm32_bind_buttons(button_pins, button_active_high)) return false;
    if (!peripheral_stm32_bind_lights(light_pins, light_codes)) return false;
    traffic_all_red();

    (void)dht20_init(&i2c_bus);
    (void)lcd_configure(&i2c_bus, LCD_GROVE_ADDRESS);
    if (!display_init(platform_read_display_view)) return false;

    if (!app_control_init(&g_app_services)) {
        traffic_all_red();
        return false;
    }

    /*
     * Mandatory registration order for simultaneous deadlines (Team Plan p. 5):
     * 1. Software timer (10 ms) - first tick after 10 ms so 1000 ms = 100 real ticks
     * 2. Button sampling (10 ms)
     * 3. Control / active FSM (10 ms)
     * 4. Sensor service (10 ms)
     * 5. Display renderer (200 ms, initial render at boot delay = 0)
     */
    if (SCH_Add_Task(timer_tick, 1U, 1U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(button_sample, 1U, 1U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(app_control_run, 1U, 1U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(dht20_task, 1U, 1U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(display_task, 0U, 20U) == SCH_NO_TASK_ID) return false;

    return true;
}
#else
static void toggle_PA5(void)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

static void toggle_PC9(void)
{
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

static void one_shot_test(void)
{
    ++g_oneshot_fired_count;
}

static bool platform_register_tasks(void)
{
    /*
     * Standalone platform & scheduler verification profile (Step 4.2 / S01-S02):
     * - Software timer service every 10 ms (1 tick)
     * - LD2 (PA5) toggles every 500 ms (50 ticks)
     * - LD3 (PC9) toggles every 1000 ms (100 ticks)
     * - One-shot verification fires once after 3000 ms (300 ticks)
     */
    if (SCH_Add_Task(timer_tick, 1U, 1U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(toggle_PA5, 0U, 50U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(toggle_PC9, 0U, 100U) == SCH_NO_TASK_ID) return false;
    if (SCH_Add_Task(one_shot_test, 300U, 0U) == SCH_NO_TASK_ID) return false;

    return true;
}
#endif

#if defined(BOARD_DIAG_STEP3)
static void platform_run_step3_hardware_diag(void)
{
    uint8_t addr;
    g_i2c_found_count = 0U;
    for (addr = 1U; addr < 128U; ++addr) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1U), 1U, 10U) == HAL_OK) {
            if (g_i2c_found_count < 8U) {
                g_i2c_found_addrs[g_i2c_found_count++] = addr;
            }
        }
    }

    for (;;) {
        uint8_t combo;
        for (combo = 0U; combo < 4U; ++combo) {
            GPIO_PinState a = ((combo >> 1U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            GPIO_PinState b = (combo & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
            HAL_GPIO_WritePin(TL1_A_GPIO_Port, TL1_A_Pin, a);
            HAL_GPIO_WritePin(TL1_B_GPIO_Port, TL1_B_Pin, b);
            HAL_GPIO_WritePin(TL2_A_GPIO_Port, TL2_A_Pin, a);
            HAL_GPIO_WritePin(TL2_B_GPIO_Port, TL2_B_Pin, b);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(2000U);
        }
    }
}
#endif
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
  platform_enable_cycle_counter();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if defined(BOARD_DIAG_STEP3)
  platform_run_step3_hardware_diag();
#endif

  SCH_Init();
  timer_init();

  /* Check task registration errors before starting automatic control (Team Plan p. 4) */
  if (!platform_register_tasks()) {
#if PLATFORM_FULL_INTEGRATION
    traffic_all_red();
#endif
    Error_Handler();
  }

  /* Clear update flag generated by UG during TIM2 init so tick #1 occurs at exact 10 ms */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
#if PLATFORM_FULL_INTEGRATION
    traffic_all_red();
#endif
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    SCH_Dispatch_Tasks();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TL1_A_Pin | TL1_B_Pin | TL2_A_Pin | TL2_B_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TL1_A_Pin TL1_B_Pin TL2_A_Pin TL2_B_Pin */
  GPIO_InitStruct.Pin = TL1_A_Pin | TL1_B_Pin | TL2_A_Pin | TL2_B_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_1_Pin BTN_2_Pin BTN_3_Pin BTN_4_Pin */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = BTN_1_Pin | BTN_2_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BTN_3_Pin;
  HAL_GPIO_Init(BTN_3_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = BTN_4_Pin;
  HAL_GPIO_Init(BTN_4_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};

  htim2.Instance = TIM2;
  /* APB1 timer clock is 64 MHz. (6399 + 1) * (99 + 1) / 64 MHz = 10 ms. */
  htim2.Init.Prescaler = 6399;
  htim2.Init.Period = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIM_ConfigClockSource(&htim2, &clock) != HAL_OK ||
      HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 4 */
/**
  * @brief  TIM period elapsed callback in non-blocking mode.
  *         TIM2 10 ms ISR strictly calls SCH_Update() in O(1) time.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    uint32_t c0 = DWT->CYCCNT;

    SCH_Update();

    uint32_t cycles = DWT->CYCCNT - c0;
    /* SYSCLK = 64 MHz -> 64 cycles = 1 us */
    uint16_t elapsed_us = (uint16_t)((cycles + 32U) / 64U);
    g_last_isr_cycles = cycles;
    if (cycles > g_max_isr_cycles)
    {
      g_max_isr_cycles = cycles;
    }
    g_last_isr_us = elapsed_us;
    if (elapsed_us > g_max_isr_us)
    {
      g_max_isr_us = elapsed_us;
    }
  }
}
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
