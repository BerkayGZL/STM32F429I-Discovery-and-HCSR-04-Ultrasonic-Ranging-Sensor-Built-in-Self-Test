/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "dwt_stm32_delay.h"
#include "stdarg.h" // va_lis printf bu kÃ¼tÃ¼phanede bulunuyor.
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>
#include "stm32f4xx_hal.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* USER CODE BEGIN PV */
volatile uint32_t value;
uint32_t time;
uint16_t distance;
char str[15];

char strtime[15];
uint16_t time_seconds;
float TIMx_clock_frequency =  (double) 90000000 / 65535;
uint32_t flash_address =  0x080E0000;
int count_global = 0;
int count = 0;

osSemaphoreId_t myTask02SemaphoreHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void printMessage(char *format, ...)
{
	char comingMessage[100];

	va_list vaList;
	va_start(vaList, format);
	vsprintf(comingMessage, format, vaList);
	HAL_UART_Transmit(&huart1, (uint8_t*)comingMessage, strlen(comingMessage), HAL_MAX_DELAY);
	va_end(vaList);
}


uint32_t Read_HCSR04()
{
	uint32_t local_time=0;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

	DWT_Delay_us(10);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

		while (!(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3))){}


		while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3))				// echo pinden deger gelirse
		{
			local_time++;										// increment local time
			DWT_Delay_us(1);									// every 1 us

		}

		return local_time;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == &htim3){
		simulateButtonPress();
	}

	if (htim->Instance == TIM6) {
		    HAL_IncTick();
		  }

}


void simulateButtonPress(void)
{
    // Generate a software interrupt on the EXTI line
    EXTI->SWIER |= EXTI_SWIER_SWIER0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	  static int adddress =  0x080E0000;
	  char  stime [30] = "Current Time: ";
	  char err_time [30] = "Error Time: ";
		if (GPIO_Pin == GPIO_PIN_0 && (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)) ){


			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

			BSP_LCD_DisplayStringAt(245, 185, (uint8_t *)"Echo pin is opened!", CENTER_MODE);
			__HAL_TIM_SetCounter(&htim3, 0 );
			HAL_TIM_Base_Stop_IT(&htim3);

			time_seconds = __HAL_TIM_GET_COUNTER(&htim2)/ TIMx_clock_frequency ;



				itoa(time_seconds, strtime, 10);
				strcat(stime,strtime);
				BSP_LCD_DisplayStringAt(245, 210, (uint8_t *)stime, CENTER_MODE);




				while(*(uint32_t*)flash_address != 0xFFFFFFFF) {
							flash_address += 4;

						}

				if (*(uint32_t*)flash_address == 0xFFFFFFFF) {
					count++;
				}

				HAL_FLASH_Unlock(); // flash kilidi açıldı

				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_address, time_seconds);
							flash_address +=4;
							HAL_FLASH_Lock();
							count_global++;

				if (count == 1) {
					itoa(*(uint32_t*)(flash_address-sizeof(uint32_t)), strtime, 10);
					strcat(err_time,strtime);
					BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
					BSP_LCD_DisplayStringAt(245, 235, (uint8_t *)err_time, CENTER_MODE);

				}
				else if (count == 2) {
					itoa(*(uint32_t*)(flash_address-sizeof(uint32_t)), strtime, 10);
					strcat(err_time,strtime);
					BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
					BSP_LCD_DisplayStringAt(245, 260, (uint8_t *)err_time, CENTER_MODE);

				}

				else if (count == 3) {
					itoa(*(uint32_t*)(flash_address-sizeof(uint32_t)), strtime, 10);
					strcat(err_time,strtime);
					BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
					BSP_LCD_DisplayStringAt(245, 285, (uint8_t *)err_time, CENTER_MODE);
					count= 0;
								}



				while (1) {

					if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3))) {
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_Delay(100);

				}
					else{ break;}
				}

		}


		}


void drawTicks(uint16_t x , uint16_t y, uint16_t radius){
	int tickLength = 10;
	int loop_angle = 180;
	int count = 9;
	int angleStep = 1;
	int angle = 0;
	char str[3];

	 for (loop_angle; loop_angle <= 360; loop_angle += angleStep)
	    {
		 float radians = loop_angle * 3.14159265f / 180.0f;
		 uint16_t xPos = x + radius * cosf(radians);
		 uint16_t yPos = y + radius * sinf(radians);

	if (loop_angle / 20 == count) {
		 BSP_LCD_DrawLine(xPos, yPos, xPos + tickLength*cosf(radians), yPos + tickLength*sinf(radians));

		 count++;
		 itoa(angle, str, 10);
		 BSP_LCD_SetFont(&Font12);

		 // Calculate the coordinates for displaying the number
		 int strWidth = strlen(str) * BSP_LCD_GetFont()->Width;  // Calculate the width of the string
		 int strHeight = BSP_LCD_GetFont()->Height;  // Get the height of the font
		 int textX = xPos - strWidth / 2;  // Calculate the X-coordinate for centering the text
		 int textY = yPos - strHeight / 2;  // Calculate the Y-coordinate for centering the text


		      if (count >= 15) {
		    	  BSP_LCD_DisplayStringAt(textX - 10, textY+10, (uint8_t *)str, LEFT_MODE);
		    	  		             angle += 20;
		      }

		      else if (count == 12) {
		    	  BSP_LCD_DisplayStringAt(textX + 5, textY+15, (uint8_t *)str, LEFT_MODE);
		    	  		    	  		 angle += 20;
		      }

		      else if (count == 13) {
		     		    	  BSP_LCD_DisplayStringAt(textX , textY+15, (uint8_t *)str, LEFT_MODE);
		     		    	  		    	  		 angle += 20;
		      }

		      else if (count == 14) {
		      		     		    	  BSP_LCD_DisplayStringAt(textX , textY+15, (uint8_t *)str, LEFT_MODE);
		      		     		    	  		    	  		 angle += 20;
		     }

		      else
		    	  BSP_LCD_DisplayStringAt(textX + 10, textY+10, (uint8_t *)str, LEFT_MODE);
		    	  		    	  		             angle += 20;


	}

}
}


void drawArc(uint16_t x , uint16_t y, uint16_t radius, uint16_t startAngle, uint16_t endAngle)
{

    float angleStep = 0.1f; // Adjust this value for the desired angle resolution

    for (float angle = startAngle; angle <= endAngle; angle += angleStep)
    {

        float radians = angle * 3.14159265f / 180.0f;
        uint16_t xPos = x + radius * cosf(radians);
        uint16_t yPos = y + radius * sinf(radians);
        BSP_LCD_DrawPixel(xPos, yPos, LCD_COLOR_RED);

        // Use HAL function or direct register manipulation to set the pixel at (xPos, yPos)
        // Example: BSP_LCD_DrawPixel(xPos, yPos, LCD_COLOR_RED);
        // Draw a tick mark at this angle

    }
}

double* tick_to_needle(uint16_t x, uint16_t y, uint16_t radius, uint16_t distance) {
	double a = 0.9;
	double angle = 180;
	int count = 0;
	double b = 0.53;

	for (angle ; angle <= 216 ; angle += a) {
		 float radians = angle * 3.14159265f / 180.0f;
		 uint16_t xPos = x + radius * cosf(radians);
		 uint16_t yPos = y + radius * sinf(radians);
		if (count == distance) {
			double* ptr = (double*)malloc(3*sizeof(double));
			ptr[0] = (double)xPos;
			ptr[1] = (double)yPos;
			ptr[2] = (double)angle;
			return ptr;
		}
		count++;
	}


		for (angle ; angle <= 360 ; angle += b) {

				float radians = angle * 3.14159265f / 180.0f;
				uint16_t xPos = x + radius * cosf(radians);
				uint16_t yPos = y + radius * sinf(radians);
				if (count == distance) {
					double* ptr = (double*)malloc(2*sizeof(double));
					ptr[0] = (double)xPos;
					ptr[1] = (double)yPos;
					return ptr;
				}
				count++;

		}

}


void drawFilledCircle(uint16_t x, uint16_t y, uint16_t radius, uint32_t color) {
    BSP_LCD_SetTextColor(color);
    BSP_LCD_FillCircle(x, y, radius);
}

void drawNeedle(uint16_t x, uint16_t y, uint16_t radius, uint16_t angle)
{
    // Calculate the angle in radians
	  double* ptr = tick_to_needle(x,  y, radius, distance);

    float radians = angle * 3.14159265f / 180.0f;

    // Calculate the coordinates for the needle tip



    drawFilledCircle(115, 120, 68, LCD_COLOR_BLUE);

    // Draw the new needle line


    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_DrawLine(x, y, ptr[0], ptr[1]);
    drawFilledCircle(115, 120, 6, LCD_COLOR_RED);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    if (distance>320) {
           	BSP_LCD_Clear(LCD_COLOR_BLUE);
            drawArc_with_Lines(115, 120, 100, 180, 360);
            drawArc(115, 119, 100, 180, 360);
            drawArc(115, 118, 100, 180, 360);
            drawArc(115, 117, 100, 180, 360);
            drawArc(115, 116, 100, 180, 360);
            drawArc(115, 115, 100, 180, 360);
            drawArc(115, 114, 100, 180, 360);
            drawArc(115, 113, 100, 180, 360);
            drawArc(115, 112, 100, 180, 360);
    	}




//    // Redraw the background color in the area covered by the needle
//    uint16_t backgroundRadius = radius - 21; // Adjust the radius to clear the area around the needle
//    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
//    BSP_LCD_FillCircle(x, y, backgroundRadius);
//
    free(ptr);
}


void drawArc_with_Lines(uint16_t x , uint16_t y, uint16_t radius, uint16_t startAngle, uint16_t endAngle)
{

    float angleStep = 0.1f; // Adjust this value for the desired angle resolution

    for (float angle = startAngle; angle <= endAngle; angle += angleStep)
    {

        float radians = angle * 3.14159265f / 180.0f;
        uint16_t xPos = x + radius * cosf(radians);
        uint16_t yPos = y + radius * sinf(radians);
        BSP_LCD_DrawPixel(xPos, yPos, LCD_COLOR_RED);

        // Use HAL function or direct register manipulation to set the pixel at (xPos, yPos)
        // Example: BSP_LCD_DrawPixel(xPos, yPos, LCD_COLOR_RED);
        // Draw a tick mark at this angle

    }
    drawTicks(x,y,radius);
//    drawNeedle(x, y, radius);

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DWT_Delay_Init();
  HAL_FLASH_Unlock(); // flash kilidi açıldı
  static uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef eraseInitStruct;
  eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; // Erase a sector
  eraseInitStruct.Sector = FLASH_SECTOR_7; // Erase Sector 7 (you can change this to the desired sector)
  eraseInitStruct.NbSectors = 1; // Erase only one sector
  eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Voltage range 2.7V to 3.6V



//  HAL_FLASHEx_Erase(&eraseInitStruct,&SectorError);

HAL_FLASH_Lock(); // flash kilidi açıldı



  //write text

  myTask02SemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	 	  //BSP_LCD_DisplayStringAtLine(2," Distance");
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 65535;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  htim3.Init.Prescaler = 18000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE5 PE10 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE3 PE6 PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PF0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
	 for(;;)
	     {

	   	  time = Read_HCSR04();			// get the high time
	   	  distance =  (time / 58);	// user the formula to get the distance
	   	  uint16_t* ptr = &distance;
	   	  printMessage(" Distance: %d \n", distance);
	   	 printMessage(" Distance: %d \n", ptr);

	   	 	 	  if(distance >= 20)
	   	 	 	  {
	   	 	 		  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,1);
	   	 	 	  } else
	   	 	 		  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,0);
	     osDelay(100);
	     }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
 	for(;;)
	  {


		  		}

		  				 osDelay(1);
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */

	  BSP_LCD_Init();
		    BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
		    BSP_LCD_SelectLayer(1);  // select on which layer we write
		    BSP_LCD_DisplayOn();     // turn on LCD
		    BSP_LCD_Clear(LCD_COLOR_BLUE);         // clear the LCD on blue color
		    BSP_LCD_SetBackColor(LCD_COLOR_BLUE);  // set text background color
		    BSP_LCD_SetTextColor(LCD_COLOR_WHITE); // set text color

		    drawArc_with_Lines(115, 120, 100, 180, 360);
		    drawArc(115, 119, 100, 180, 360);
		    drawArc(115, 118, 100, 180, 360);
		    drawArc(115, 117, 100, 180, 360);
		    drawArc(115, 116, 100, 180, 360);
		    drawArc(115, 115, 100, 180, 360);
		    drawArc(115, 114, 100, 180, 360);
		    drawArc(115, 113, 100, 180, 360);
		    drawArc(115, 112, 100, 180, 360);






		    int textHeight = BSP_LCD_GetFont()->Height;  // Get the height of the font
		    int lineNumber = (BSP_LCD_GetYSize() - textHeight) / textHeight;

		    /* Infinite loop */
		    for (;;)
		    {
		    	  char err_time [30] = "Error Time: ";
		    	  char err_time1 [30] = "Error Time: ";
		    	  char err_time2 [30] = "Error Time: ";
		        lineNumber = (BSP_LCD_GetYSize() - textHeight) / textHeight;  // Calculate the line number

		        BSP_LCD_ClearStringLine(14); // Clear the line where the distance value is displayed
		        BSP_LCD_ClearStringLine(15); // Clear the line where the distance value is displayed
		        BSP_LCD_ClearStringLine(13); // Clear the line where the distance value is displayed
		        BSP_LCD_ClearStringLine(12); // Clear the line where the distance value is displayed
		        BSP_LCD_ClearStringLine(16); // Clear the line where the distance value is displayed
		        BSP_LCD_ClearStringLine(17); // Clear the line where the distance value is displayed
		        BSP_LCD_ClearStringLine(11.5); // Clear the line where the distance value is displayed
		        BSP_LCD_SetFont(&Font12);
		        BSP_LCD_DisplayStringAt(87, 124, (uint8_t *)"320", CENTER_MODE);
		        BSP_LCD_SetFont(&Font16);

		        // BSP_LCD_Clear(LCD_COLOR_WHITE);
		        itoa(distance, str, 10);


		        if (distance>320) {
		        	BSP_LCD_Clear(LCD_COLOR_BLUE);
		        	   drawArc_with_Lines(115, 120, 100, 180, 360);
		        	   drawArc(115, 119, 100, 180, 360);
		        	   drawArc(115, 118, 100, 180, 360);
		        	   drawArc(115, 117, 100, 180, 360);
		        	   drawArc(115, 116, 100, 180, 360);
		        	   drawArc(115, 115, 100, 180, 360);
		        	   drawArc(115, 114, 100, 180, 360);
		        	   drawArc(115, 113, 100, 180, 360);
		        	   drawArc(115, 112, 100, 180, 360);
		        }
		        BSP_LCD_SetFont(&Font16);
		        BSP_LCD_DisplayStringAt(245, 190, (uint8_t *)str, CENTER_MODE);

		        // Calculate the angle for the needle based on the distance
		        uint16_t needleAngle = 180 + (distance * 1.8f); // Assuming distance range is 0-100 and needle range is 180-360 degrees

		        // Draw the needle with the updated angle
		        drawNeedle(115, 120, 63, needleAngle);

		        if (count == 1) {
		        	itoa(*(uint32_t*)(flash_address-sizeof(uint32_t)), strtime, 10);
		        	strcat(err_time,strtime);
		        	BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
		        	BSP_LCD_DisplayStringAt(245, 235, (uint8_t *)err_time, CENTER_MODE);

		        			}
		       else if (count == 2) {
		        itoa(*(uint32_t*)(flash_address-2*sizeof(uint32_t)), strtime, 10);
		       strcat(err_time,strtime);
		        BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
		        	BSP_LCD_DisplayStringAt(245, 235, (uint8_t *)err_time, CENTER_MODE);

		        	itoa(*(uint32_t*)(flash_address-sizeof(uint32_t)), strtime, 10);
		        	strcat(err_time1,strtime);
		        	BSP_LCD_ClearStringLine(22); // Clear the line where the distance value is displayed
		        	BSP_LCD_ClearStringLine(21); // Clear the line where the distance value is displayed
		        	BSP_LCD_DisplayStringAt(245, 260, (uint8_t *)err_time1, CENTER_MODE);

		        			}

		       else if (count == 3) {
		        	itoa(*(uint32_t*)(flash_address-3*sizeof(uint32_t)), strtime, 10);
		        	strcat(err_time,strtime);
		        	BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
		      	BSP_LCD_DisplayStringAt(245, 235, (uint8_t *)err_time, CENTER_MODE);

		      itoa(*(uint32_t*)(flash_address-2*sizeof(uint32_t)), strtime, 10);
		      strcat(err_time1,strtime);
		     BSP_LCD_ClearStringLine(22); // Clear the line where the distance value is displayed
		        		 BSP_LCD_ClearStringLine(21); // Clear the line where the distance value is displayed
		           BSP_LCD_DisplayStringAt(245, 260, (uint8_t *)err_time1, CENTER_MODE);
		     itoa(*(uint32_t*)(flash_address-sizeof(uint32_t)), strtime, 10);
		     strcat(err_time2,strtime);
		        BSP_LCD_ClearStringLine(20); // Clear the line where the distance value is displayed
		        BSP_LCD_DisplayStringAt(245, 285, (uint8_t *)err_time2, CENTER_MODE);
		        count= 0;
		        											}


		        osDelay(100);
		    }

  }
  /* USER CODE END StartTask03 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */

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

