/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "peripheral_stm32.h"
#include "button.h"
#include "traffic_light.h"
#include "dht20.h"
#include "lcd.h"
#include "display.h"
#include "global.h"
#include "app_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Quick hardware mapping constants (adjust after Step 3 kit check if needed):
 * - BUTTON_ACTIVE_HIGH: false = active-LOW buttons with internal GPIO_PULLUP
 * - LIGHT_CODE_*: 2-bit truth table ((Signal_A << 1) | Signal_B) for each color
 * - LCD_FALLBACK_ADDR: used if auto-scan does not detect an LCD at startup
 */
#define BUTTON_ACTIVE_HIGH  false
#define LIGHT_CODE_RED      3U
#define LIGHT_CODE_GREEN    2U
#define LIGHT_CODE_AMBER    1U
#define LCD_FALLBACK_ADDR   LCD_OHSTEM_ADDRESS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static app_event_t app_read_button_event(uint8_t id)
{
    switch (button_take_event(id)) {
    case BUTTON_PRESS: return APP_EVENT_PRESS;
    case BUTTON_LONG:  return APP_EVENT_LONG;
    case BUTTON_NONE:
    default:           return APP_EVENT_NONE;
    }
}

static light_t app_to_driver_light(app_light_t light)
{
    switch (light) {
    case APP_GREEN: return LIGHT_GREEN;
    case APP_AMBER: return LIGHT_AMBER;
    case APP_RED:
    default:        return LIGHT_RED;
    }
}

static bool app_apply_lights(app_light_t road1, app_light_t road2)
{
    return traffic_apply_pair(app_to_driver_light(road1), app_to_driver_light(road2));
}

static display_view_t app_read_display_view(void)
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

static void heartbeat_task(void)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

static void system_setup_and_schedule(void)
{
    static const peripheral_pin_t button_pins[BUTTON_COUNT] = {
        {BTN_1_GPIO_Port, BTN_1_Pin},
        {BTN_2_GPIO_Port, BTN_2_Pin},
        {BTN_3_GPIO_Port, BTN_3_Pin},
        {BTN_4_GPIO_Port, BTN_4_Pin}
    };
    static const bool button_polarity[BUTTON_COUNT] = {
        BUTTON_ACTIVE_HIGH, BUTTON_ACTIVE_HIGH, BUTTON_ACTIVE_HIGH, BUTTON_ACTIVE_HIGH
    };
    static const peripheral_pin_t light_pins[4] = {
        {TL1_A_GPIO_Port, TL1_A_Pin},
        {TL1_B_GPIO_Port, TL1_B_Pin},
        {TL2_A_GPIO_Port, TL2_A_Pin},
        {TL2_B_GPIO_Port, TL2_B_Pin}
    };
    static const traffic_codes_t light_codes[2] = {
        {.red = LIGHT_CODE_RED, .amber = LIGHT_CODE_AMBER, .green = LIGHT_CODE_GREEN},
        {.red = LIGHT_CODE_RED, .amber = LIGHT_CODE_AMBER, .green = LIGHT_CODE_GREEN}
    };
    static const app_services_t services = {
        .button_take_event = app_read_button_event,
        .timer_start = timer_start,
        .timer_cancel = timer_cancel,
        .timer_take_expired = timer_take_expired,
        .traffic_apply_pair = app_apply_lights,
        .traffic_all_red = traffic_all_red,
        .timer_remaining_ms = timer_remaining_ms
    };

    peripheral_i2c_t i2c_bus = peripheral_stm32_i2c(&hi2c1);
    uint8_t lcd_addr = peripheral_stm32_scan_lcd(&hi2c1, LCD_FALLBACK_ADDR);

    SCH_Init();
    timer_init();

    (void)peripheral_stm32_bind_buttons(button_pins, button_polarity);
    (void)peripheral_stm32_bind_lights(light_pins, light_codes);
    traffic_all_red();

    (void)dht20_init(&i2c_bus);
    (void)lcd_configure(&i2c_bus, lcd_addr);
    (void)display_init(app_read_display_view);
    (void)app_control_init(&services);

    /* Register scheduled tasks in contract order (1 tick = 10 ms) */
    (void)SCH_Add_Task(timer_tick, 1U, 1U);       /* 10 ms: software timers */
    (void)SCH_Add_Task(button_sample, 1U, 1U);    /* 10 ms: button debounce */
    (void)SCH_Add_Task(app_control_run, 1U, 1U);  /* 10 ms: active mode FSM */
    (void)SCH_Add_Task(dht20_task, 1U, 1U);       /* 10 ms: non-blocking DHT20 */
    (void)SCH_Add_Task(display_task, 1U, 20U);    /* 200 ms: LCD 16x2 update */
    (void)SCH_Add_Task(heartbeat_task, 1U, 50U);  /* 500 ms: onboard LD2 blink */
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  system_setup_and_schedule();

  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
  (void)HAL_TIM_Base_Start_IT(&htim2);
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

  /*Configure GPIO pins : LD2_Pin LD3_Pin TL1_A/B TL2_A/B */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD3_Pin;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    SCH_Update();
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
