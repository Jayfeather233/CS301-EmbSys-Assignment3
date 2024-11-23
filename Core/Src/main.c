/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
  VIDEO  0 - 135
  SCRPRT 160 - 

  bar 284 - 290
  | H, W, <- pause/start ->, time | 300 - 312
*/
#define SCR_PRT_y 160
#define BAR_y 284
#define BAR_s 6
#define INFO_y 300
#define INFO_s 12

#define H_x 10 // - 52
#define W_x 58 // - 100

#define PS_x 144
#define PS_s 12

#define TIME_x 200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t* video_buffer = NULL;
uint16_t* video_meta = NULL; // H, W, fps, total_fps
size_t video_meta_len = 4;
int scale = 1;
uint8_t rxBuffer;

volatile int duration = 0; // for TIM irq

uint32_t video_size = 0;
volatile uint8_t sig_unpack_meta = 0;
volatile uint8_t sig_unpack_video = 0;
volatile uint8_t sig_save_frame = 0;
volatile uint8_t sig_scale = 0;
volatile uint8_t sig_UI = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void save_frame();

void draw_bar();
void draw_UI();
void scale_change();
void draw_ps();
void draw_info();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BACK_COLOR = BLACK;
  POINT_COLOR = WHITE;
  LCD_Init();
  LCD_Clear(BACK_COLOR);
  HAL_UART_Receive_IT(&huart1, &rxBuffer, 1);
  video_meta = (uint16_t*)malloc(sizeof(uint16_t)*video_meta_len);
  draw_ps();
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(sig_scale){
      sig_scale = 0;
      LCD_Fill(0, 0, video_meta[0]*scale, video_meta[1]*scale, BACK_COLOR);
      scale_change();
    }

    if(sig_UI){
      sig_UI = 0;
      draw_UI();
    }

    if(sig_unpack_meta){
      sig_unpack_meta = 0;
      if(video_buffer != NULL){
        free(video_buffer);
        LCD_Fill(0, 0, video_meta[0]*scale, video_meta[1]*scale, BACK_COLOR);
      }
      video_buffer = (uint16_t*)malloc(sizeof(uint16_t)*(video_size+1));
      duration = 0;

      draw_info();
    }

    if(sig_unpack_video){
      sig_unpack_video = 0;
      if(video_buffer == NULL || video_size == 0){
        LCD_ShowString(10, 280, 220, 12, 12, (uint8_t*)"Video Buffer not init.");
        ack = 0x10;
        continue;;
      }
      LCD_myColor_Fill_Window(0, 0, video_meta[0]-1, video_meta[1]-1, video_buffer, scale);
    }

    if(sig_save_frame && video_buffer != NULL){
      save_frame();
      sig_save_frame = 0;
    }
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/* USER CODE BEGIN 4 */
int mymin(int a, int b){
  return a<b ? a : b;
}
int mymax(int a, int b){
  return a>b ? a : b;
}
void scale_change(){
  // LCD_Fill(0, 0, video_meta[0]*scale, video_meta[1]*scale, BACK_COLOR);
  int maximum = mymax(3, mymin(240/video_meta[0], 320/video_meta[1]));
  scale = scale % maximum + 1;
}
void save_frame(){
  LCD_Color_Fill(0, SCR_PRT_y, video_meta[0]-1, video_meta[1]+SCR_PRT_y-1, video_buffer);
}
void draw_info(){
  LCD_ShowString(H_x, INFO_y, 12, INFO_s, INFO_s, (uint8_t*)"H:");
  LCD_ShowNum(H_x+12, INFO_y, video_meta[1], 5, INFO_s);
  LCD_ShowString(W_x, INFO_y, 12, INFO_s, INFO_s, (uint8_t*)"W:");
  LCD_ShowNum(W_x+12, INFO_y, video_meta[0], 5, INFO_s);

  draw_ps();

  static char ss[6];
  ss[0] = '0' + duration/600%10;
  ss[1] = '0' + duration/60%10;
  ss[2] = ':';
  ss[3] = '0' + duration%60/10%10;
  ss[4] = '0' + duration%60%10;
  ss[5] = 0;
  LCD_ShowString(TIME_x, INFO_y, 40, INFO_s, INFO_s, (uint8_t*)ss);
}

void draw_ps(){
  LCD_Color_Fill(PS_x, INFO_y, PS_x+PS_s-1, INFO_y+PS_s-1, (is_paused == 0 ? IMG_pause : IMG_start));
}

void LCD_myDraw_Bar(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey,
		uint16_t nw, uint16_t tot) {
	uint16_t height, width;
	uint16_t i, j;
	width = ex - sx + 1;
	height = ey - sy + 1;
	LCD_Set_Window(sx, sy, width, height);
	LCD_SetCursor(sx, sy);
	LCD_WriteRAM_Prepare();
	for (i = 0; i < height; i++) {
    uint16_t x1 = (i*3 >= height && i*3 < height * 2);
		for (j = 0; j < width; j++){
			LCD_WR_DATA((x1 || j*tot < width * nw) ? POINT_COLOR : BACK_COLOR);
    }
	}
	LCD_Set_Window(0, 0, 240, 320);
}

void draw_bar(){
  LCD_myDraw_Bar(10, BAR_y, 230, BAR_y+BAR_s, video_buffer[video_size], video_meta[3]);
}
void draw_UI(){
  draw_info();
  draw_bar();
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
