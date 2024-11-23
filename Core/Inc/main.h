/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum RecvState
{
  RECV_WAIT,
  RECV_VIDEO,
  RECV_META
} RecvState_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern UART_HandleTypeDef huart1;
extern uint8_t rxBuffer;
extern uint16_t* video_buffer;
extern uint16_t* video_meta;
extern uint32_t video_size;
extern volatile uint8_t sig_unpack_meta;
extern volatile uint8_t sig_unpack_video;
extern volatile uint8_t sig_save_frame;
extern volatile uint8_t sig_scale;
extern volatile uint8_t sig_UI;
extern volatile uint8_t is_paused;

extern const uint16_t IMG_pause[145];
extern const uint16_t IMG_start[145];
extern volatile int duration;
extern volatile uint8_t ack;

extern size_t video_meta_len;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_WKUP_Pin GPIO_PIN_0
#define KEY_WKUP_GPIO_Port GPIOA
#define KEY_WKUP_EXTI_IRQn EXTI0_IRQn
#define KEY0_Pin GPIO_PIN_5
#define KEY0_GPIO_Port GPIOC
#define KEY0_EXTI_IRQn EXTI9_5_IRQn
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
