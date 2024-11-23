/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define tick_init(n)                \
  tickstart = HAL_GetTick();        \
  wait = (n);                       \
  if (wait < HAL_MAX_DELAY)         \
  {                                 \
    wait += (uint32_t)(uwTickFreq); \
  }                                 \
  do                                \
  {                                 \
  } while (0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
RecvState_t state_dig = RECV_WAIT;
volatile uint8_t ack = 0;
volatile uint8_t is_paused = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void send_ack();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_WKUP_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY0_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void send_ack()
{
  HAL_UART_Transmit(&huart1, &ack, 1, 0xffff);
  ack = 0;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_Delay(50);
  uint32_t tickstart;
  uint32_t wait;
  switch (GPIO_Pin)
  {
  case KEY0_Pin:
    if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) != GPIO_PIN_RESET)
      break;
    tick_init(800);
    while (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET && (HAL_GetTick() - tickstart) < wait)
      ;
    if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
    { // long: slower
      ack = 0x22;
    }
    else
    { // short: scale
      sig_scale = 1;
    }
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    while (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
      ;
    break;
  case KEY1_Pin:
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) != GPIO_PIN_RESET)
      break;
    tick_init(800);
    while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET && (HAL_GetTick() - tickstart) < wait)
      ;
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    { // long: ff
      ack = 0x21;
    }
    else
    { // short: pause
      ack = 0x20;
      is_paused = is_paused ? 0 : 1;
      sig_UI = 1;
    }
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
      ;
    break;
  case KEY_WKUP_Pin:
    if (HAL_GPIO_ReadPin(KEY_WKUP_GPIO_Port, KEY_WKUP_Pin) != GPIO_PIN_SET)
      break;
    tick_init(800);
    while (HAL_GPIO_ReadPin(KEY_WKUP_GPIO_Port, KEY_WKUP_Pin) == GPIO_PIN_SET && (HAL_GetTick() - tickstart) < wait)
      ;
    if (HAL_GPIO_ReadPin(KEY_WKUP_GPIO_Port, KEY_WKUP_Pin) == GPIO_PIN_SET)
    { // long: stop
      ack = 0xff;
    }
    else
    { // short: PRT_SCR
      sig_save_frame = 1;
    }
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    while (HAL_GPIO_ReadPin(KEY_WKUP_GPIO_Port, KEY_WKUP_Pin) == GPIO_PIN_SET)
      ;
    break;
  default:
    break;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    switch (state_dig)
    {
    case RECV_WAIT:
      if (rxBuffer == 0x10)
      {
        state_dig = RECV_META;
        // LCD_ShowString(10, 240, 220, 12, 12, "recieving meta.");
      }
      else if (rxBuffer == 0x00)
      {
        state_dig = RECV_VIDEO;
        // LCD_ShowString(10, 240, 220, 12, 12, "recieving video.");
      }
      else if (rxBuffer == 0x30)
      {
        HAL_UART_Transmit(&huart1, &is_paused, 1, 0xffff);
      }
      else
      {
        LCD_ShowString(10, 240, 220, 12, 12, (uint8_t *)"wrong protoc recieved.");
        LCD_ShowxNum(10, 260, rxBuffer, 3, 12, 0b1000000);
      }
      break;

    case RECV_META:
      video_size = video_meta[0] * video_meta[1];
      sig_unpack_meta = 1;
      state_dig = RECV_WAIT;
      break;
    case RECV_VIDEO:
      sig_unpack_video = 1;
      state_dig = RECV_WAIT;
      break;

    default:
      break;
    }
    switch (state_dig)
    {
    case RECV_WAIT:
      HAL_UART_Receive_IT(&huart1, &rxBuffer, 1);
      uint8_t ty = 0;
      HAL_UART_Transmit(&huart1, &ty, 1, 0xffff);
      break;
    case RECV_META:
      HAL_UART_Receive_IT(&huart1, (uint8_t *)video_meta, video_meta_len<<1);
      send_ack();
      break;
    case RECV_VIDEO:
      HAL_UART_Receive_IT(&huart1, (uint8_t *)video_buffer, (video_size + 1) << 1);
      send_ack();
      break;

    default:
      break;
    }
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // if(htim->Instance == TIM3){
    ++duration;
    sig_UI = 1;
  // }
}
/* USER CODE END 1 */
