/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include <stdbool.h> 

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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Nums[11];
uint8_t Chars[27];

uint8_t char_bit_map[128] = { 
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  	0b11111100, // 0
		0b01100000, // 1
		0b11011010, // 2
		0b11110010, // 3
		0b01100110, // 4
		0b10110110, // 5
		0b10111110, // 6
		0b11100000, // 7
		0b11111110, // 8
		0b11110110, // 9
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
    0b11101110,	// a
		0b00111110,	// b
		0b10011100,	// c
		0b01111010,	// d
		0b10011110,	// e
		0b10001110,	// f
		0b10111100,	// g
		0b01101110,	// h
		0b00001100,	// i
		0b01111000,	// j
		0b00000000,	//
		0b00011100,	// l
		0b11010100,	// m
		0b00101010,	// n
		0b11111100,	// o
		0b11001110,	// p
		0b11100110,	// q
		0b00001010,	// r
		0b10110110,	// s
		0b00011110,	// t
		0b00111000,	// u
		0b01111100,	// v
		0b00000000,	//
		0b00000000,	//
		0b01110110,	// y
		0b00000000, //
		0b00000000	// Empty character ( { )
};

sn74hc595_cfg_t hw_cfg;
volatile uint8_t current_digit = 0;
uint8_t display_buffer[4] = { '{', '{', '{', '{',};
uint8_t selectorMode = 0;
uint16_t counterNum = 0;
//uint8_t volume = 10;
uint8_t counterPeriod = htim2.Init.Period; //Find a way to access this value from the code
uint16_t maxDutyCycle = counterPeriod;
uint16_t dutyCycle = 0;
bool alarmSet = false;
bool alarmSounding = false;
bool militaryTime = false;
bool alarmChecked = false;
uint8_t currentTime[5] = {1, 2, 0, 0, 0};
uint8_t maxTime[4] = {1, 2, 5, 9};
uint8_t alarmTime[5] = {0, 0, 0, 0, 0};
void updateTime(){
	currentTime[3] += 1;
	for(int i = 3; i >= 0; i--){
		if(currentTime[i] > maxTime[i]){
			currentTime[i] = 0;
			if(i != 0){
				currentTime[i - 1] += 1;
			}
		}
	}
	if(currentTime[0] == 0 && currentTime[1] == 0){
		if(militaryTime == true){
			currentTime[1] = 0;
		} else {
			currentTime[1] = 1;
			if(currentTime[5] == 1){
				currentTime[5] = 0;
			} else {
				currentTime[5] = 1;
			};
		}
	}
	alarmChecked = false;
};
// void shiftBit(bool bit){
// 	/*This is the reverse since I did the entire codebase thinking that my
// 	display was a common cathode when it wasn't, so changing this one thing
// 	was faster than changing everything else*/
// 	if(bit){
// 		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, GPIO_PIN_RESET);
// 	} else {
// 		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, GPIO_PIN_SET);
// 	}
// 	HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, GPIO_PIN_SET);
// 	HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, GPIO_PIN_RESET);
// };

void activate_display(uint8_t char_0, uint8_t char_1, uint8_t char_2, uint8_t char_3, uint8_t dot_location){
  display_buffer[0] = char_bit_map[char_0] | (dot_location == 0) ? 0b00000001 : 0;
  display_buffer[1] = char_bit_map[char_1] | (dot_location == 1) ? 0b00000001 : 0;
  display_buffer[2] = char_bit_map[char_2] | (dot_location == 2) ? 0b00000001 : 0;
  display_buffer[3] = char_bit_map[char_3] | (dot_location == 3) ? 0b00000001 : 0;
};

void changeTimeOrAlarm(uint8_t *arr){
	uint8_t newTime[4] = {8, 8, 8, 8};
	for(int i = 0; i < 4; i++){
		while(selectorMode == 0){
			switch(i){
				case 0:
					activateDisplay(counterNum, newTime[1], newTime[2], newTime[3]);
					if(counterNum > maxTime[i]){
						counterNum = maxTime[i];
					}
					break;
				case 1:
					activateDisplay(newTime[0], counterNum, newTime[2], newTime[3]);
					if(counterNum > maxTime[i]){
						counterNum = maxTime[i];
					}
					break;
				case 2:
					activateDisplay(newTime[0], newTime[1], counterNum, newTime[3]);
					if(counterNum > maxTime[i]){
						counterNum = maxTime[i];
					}
					break;
				case 3:
					activateDisplay(newTime[0], newTime[1], newTime[2], counterNum);
						if(counterNum > maxTime[i]){
							counterNum = maxTime[i];
						}
					break;
			}
		}
		newTime[i] = counterNum;
		counterNum = 0;
		selectorMode = 0;
		if(newTime[0] == 0){
			newTime[0] = 10;
		}
	}
	if(militaryTime == false){
		while(selectorMode == 0){
			if(counterNum % 2 == 0){
				activateDisplay('a', 'm', '{', '{');
			} else {
				activateDisplay('{', '{', 'p', 'm');
			}
		}
		// LED dot at the end indicates PM
		if(counterNum % 2 == 0){
			newTime[4] = 0;
		} else {
			newTime[4] = 1;
		}
	}
	for(int i = 0; i < 5; i++){
		arr[i] = newTime[i];
	}
};
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 	 if(GPIO_Pin == Selector_Pin){
 		 selectorMode += 1;
 	 } else if (GPIO_Pin == Decrease_Pin){
 		 if(counterNum > 0){
 			 counterNum--;
 		 }
 	 } else if (GPIO_Pin == Increase_Pin){
 		 counterNum++;
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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  if (sn74hc595_config( &hw_cfg, &hspi1, RCLK_GPIO_Port, RCLK_Pin, 3) != 0){
    return -1;
  }

  HAL_SPI_Transmit_DMA(&hspi1, 0x00, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  activateDisplay(currentTime[0], currentTime[1], currentTime[2], currentTime[3], currentTime[4]);

	  if(selectorMode > 0){

		if(alarmSounding == true){
			alarmSounding = false;
			TIM2->CCR1 = 0;

		} else {

		  counterNum = 0;
		  selectorMode = 1;

		  while(selectorMode == 1){
			  switch(counterNum){
				case 0: activateDisplay('a', 'l', 'r', 'm'); break;
				case 1: activateDisplay('t', 'i', 'm', 'e'); break;
				//case 2: activateDisplay('v', 'o', 'l'); break;
				case 2: activateDisplay('l', 'e', 'd', '{'); break;
				case 3:	if(militaryTime){activateDisplay(2, 4, 'h', 'r');} else {activateDisplay(1, 2, 'h', 'r');}; break;
				case 4: activateDisplay('c', 'n', 'c', 'l'); break;
				case 5: counterNum--; break;
				default: counterNum = 0;
			  }
		  }

		  selectorMode = 0;
		  if(counterNum == 0){
			  changeTimeOrAlarm(alarmTime);

		  } else if(counterNum == 1){
			counterNum = 0;
			changeTimeOrAlarm(currentTime);
		  } /* else if(counterNum == 2){
			counterNum = 0;
			while(selectorMode == 0){
				if(counterNum > 9){
					activateDisplay('t', 'o', 'p');
					counterNum = 10;
				} else {
					activateDisplay('v', 'o', 'l', counterNum);
				}
			}
			volume = counterNum;
			selectorMode = 0;
		  } */ else if(counterNum == 2){
			counterNum = 0;
			while(selectorMode == 0){
				if(counterNum > 9){
					activateDisplay('t', 'o', 'p', '{');
					counterNum = 10;
				} else {
					activateDisplay('l', 'e', 'd', counterNum);
				}
			}
			maxDutyCycle = (counterPeriod * counterNum ) / 10;
			selectorMode = 0;
		  }	else if (counterNum == 3){
			counterNum = 0;
				while(selectorMode == 0){
					if(counterNum % 2 == 0){
						activateDisplay(1, 2, 'h', 'r');
					} else {
						activateDisplay(2, 4, 'h', 'r');
					}
					if(counterNum % 2 == 0){
					militaryTime = false;
					} else {
					militaryTime = true;
					}
				}
				if(militaryTime == false){
					if(currentTime[0] > 1 || currentTime[1] > 2){
						int temp = currentTime[1] - 2;
						if(temp == -1){
							currentTime[1] = 9;
						} else if(temp == -2){
							currentTime[1] = 2;
						} else {
							currentTime[1] = temp;
						}
						currentTime[0] -= 1;
						currentTime[4] = 1;
					}
				}
				if(militaryTime == true && currentTime[4] == 1){
					currentTime[0] += 1;
					currentTime[1] += 2;
					currentTime[4] = 0;
				}
		  } else if (counterNum == 4){
			  counterNum = 0;
		  }
		 }
	  }
	  if(alarmSet == true && alarmChecked == false){
		if(militaryTime == true){
			if( alarmTime[3] == currentTime[3] &&
				alarmTime[2] == currentTime[2] &&
  	   	   	  ((alarmTime[1] == currentTime[1] ) ||
  	   	   	   (alarmTime[1] == 0 && currentTime[1] == 3) )){
					TIM2->CCR1 = maxDutyCycle / 6;
			} else {
				for(int i = 5; i > 0; i--){
					if( (currentTime[2] + i) == alarmTime[2] ){
						TIM2->CCR1 = maxDutyCycle - maxDutyCycle * i / 6;
						break;
					}
				}
			alarmChecked = true;
			}
		} else {
			if(alarmTime[3] == currentTime[3] && alarmTime[2] == currentTime[2] &&
  	   	   ( (alarmTime[1] == currentTime[1] ) || ((alarmTime[1] == 1 && currentTime[1] == 12) && (alarmTime[5] != currentTime[5]) ) )){
				TIM2->CCR1 = maxDutyCycle / 6;
			} else {
				for(int i = 5; i > 0; i--){
					if( (currentTime[2] + i) == alarmTime[2] ){
						TIM2->CCR1 = maxDutyCycle - maxDutyCycle * i / 6;
						break;
					}
				}
				alarmChecked = true;
			}
		}
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x1;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 300;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D1_Pin|D2_Pin|D3_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RCLK_Pin|SRCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SER_Pin|Alarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D1_Pin D2_Pin D3_Pin D4_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D3_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RCLK_Pin SRCLK_Pin */
  GPIO_InitStruct.Pin = RCLK_Pin|SRCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SER_Pin Alarm_Pin */
  GPIO_InitStruct.Pin = SER_Pin|Alarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Selector_Pin Decrease_Pin Increase_Pin */
  GPIO_InitStruct.Pin = Selector_Pin|Decrease_Pin|Increase_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
  RTC_AlarmTypeDef sAlarm;
  HAL_RTC_GetAlarm(hrtc,&sAlarm,RTC_ALARM_A,FORMAT_BIN);
  if(sAlarm.AlarmTime.Seconds>58) {
    sAlarm.AlarmTime.Seconds=0;
  }else{
    sAlarm.AlarmTime.Seconds=sAlarm.AlarmTime.Seconds+1;
  }
    while(HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN)!=HAL_OK){}
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi->Instance == hw_cfg->hspi)
    {
      switch (current_digit){
        case 0:
        HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
        sn74hc595_shift_byte(hw_cfg, display_buffer[0]);
        sn74hc595_latch_data(hw_cfg);
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
        current_digit = 1;
        break;
        case 1:
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
        sn74hc595_shift_byte(hw_cfg, display_buffer[1]);
        sn74hc595_latch_data(hw_cfg);
        HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
        current_digit = 2;
        break;
        case 2:
        HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
        sn74hc595_shift_byte(hw_cfg, display_buffer[2]);
        sn74hc595_latch_data(hw_cfg);
        HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
        current_digit = 3;
        break;
        case 3:
        HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
        sn74hc595_shift_byte(hw_cfg, display_buffer[3]);
        sn74hc595_latch_data(hw_cfg);
        HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
        current_digit = 0;
        break;
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
