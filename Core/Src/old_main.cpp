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

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t Nums[11] = {
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
		0b00000000	// Empty digit
		};
uint8_t Chars[27] = {
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
		0b11010100,	// m, sorta
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
void shiftBit(bool bit){
	/*This is the reverse since I did the entire codebase thinking that my
	display was a common cathode when it wasn't, so changing this one thing
	was faster than changing everything else*/
	if(bit){
		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(SER_GPIO_Port, SER_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SRCLK_GPIO_Port, SRCLK_Pin, GPIO_PIN_RESET);
};


void displayByte(uint8_t digit, bool activateDot = false){
	if(activateDot == true){
		digit |= 0b00000001;
	};
	for(int i = 0; i < 8; i++){
		uint8_t temp = digit;
		shiftBit(temp & 1);
		digit >>= 1;
	};
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RCLK_GPIO_Port, RCLK_Pin, GPIO_PIN_RESET);
};
void displayNum(unsigned int num, bool isMiddle = false){
	displayByte(Nums[num], isMiddle);
};
void displayChar(char letter, bool isMiddle = false){
	displayByte(Chars[letter - 97], isMiddle);
};
void activateDisplay(int Digit1, int Digit2, int Digit3, int Digit4, uint8_t AMPM = 0){
	if(Digit1 != 10){
displayNum(Digit1);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
	}
displayNum(Digit2, true);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
HAL_Delay(1);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	displayNum(Digit3);
HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
HAL_Delay(1);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
	displayNum(Digit4, AMPM);
HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
};
void activateDisplay(char C1 = '[', char C2 = '[', char C3 = '[', char C4 = '['){
	displayChar(C1);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
	displayChar(C2);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
HAL_Delay(1);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	displayChar(C3);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
	if(C4 == '[') return;
	displayChar(C4);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
};
void activateDisplay(char C1 = '[', char C2 = '[', char C3 = '[', int Digit4 = 10){
	displayChar(C1);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
	displayChar(C2);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	displayChar(C3, true);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
	displayNum(Digit4);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
};
void activateDisplay(int Digit1, int Digit2, char C3 = '[', char C4 = '['){
	if(Digit1 != 10){
		displayNum(Digit1);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, GPIO_PIN_RESET);
	}
	displayNum(Digit2);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, GPIO_PIN_RESET);
	displayNum(C3);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
	displayNum(C4);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);
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
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
