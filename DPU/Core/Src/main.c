/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include"DPU_3200_48.h"
#include "LCD.h"
#define Adresss 0x1FFFF7BA
#define Adress2 0x1FFFF7BB
uint32_t ADC_Count_DPU[4]={0};
uint16_t Vint_Cal=0;
char DPU_Char_Vol[5]={0};
uint16_t DisplayCounter=0;
uint16_t Encod_Counter=0;

#define LCD16xN
Lcd_HandleTypeDef lcd;
Lcd_PortType LCDArrayPort[]={D4_GPIO_Port, D5_GPIO_Port,D6_GPIO_Port,D7_GPIO_Port};
Lcd_PinType LCDArrayPins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};


//HAL_StatusTypeDef val=0;

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
 ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void EncoderAPI(void);

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

//  uint8_t* pint =;
//  uint8_t* pint2 =;
//  uint8_t pint3=*pint;
//  uint8_t pint4=*pint;
  Vint_Cal=*((uint16_t*)0x1FFFF7BA);


  FactorDpu=17920;
  TIM3->CCR4=duty;


   HAL_ADC_Start_DMA(&hadc,(uint32_t*)ADC_Count_DPU,4);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
 // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
  lcd = Lcd_create(LCDArrayPort, LCDArrayPins, RS_GPIO_Port, RS_Pin, GPIOB, EN_Pin, LCD_4_BIT_MODE);
  Lcd_cursor(&lcd, 0,1);
  Lcd_string(&lcd, "  WELCOME ");
  Lcd_cursor(&lcd, 1,1);
  Lcd_string(&lcd, "  ELECTRIFUEL ");

	 long_delay(110);
	 DPU_Average();
	  HAL_TIM_Base_Start_IT(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	   TIM3->CCR4=duty;
	  // setdpuvoltage();
	   DpuVol=(FactorDpu*DPU_Vol_Count/1000);
	   EncoderAPI();


	  if(duty > 480)
	  {
		  duty=480;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 480-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_Pin|RS_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CCLK_ENCOD_Pin DT_ENCOD_Pin */
  GPIO_InitStruct.Pin = CCLK_ENCOD_Pin|DT_ENCOD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



void Bitwise (void)
{
	   DPU_Char_Vol[0]=(DpuVol/10000)+48;
	   DPU_Char_Vol[1]=(DpuVol%10000)/1000+48;
	   DPU_Char_Vol[2]=(DpuVol%1000)/100+48;
	   DPU_Char_Vol[3]=(DpuVol%100)/10+48;
	   DPU_Char_Vol[4]=(DpuVol%10)+48;
}
void DPU_Display(void)
{
	  Lcd_clear(&lcd);
 	if(DpuVol < 17000)
	{
		  Lcd_cursor(&lcd, 0,1);
		  Lcd_string(&lcd, "  DPU OFF  ");
		  Lcd_cursor(&lcd, 1,1);
		  Lcd_string(&lcd, "   TURN ON DPU  ");
	}
	else
	{
		  Lcd_cursor(&lcd, 0,0);
		  Lcd_string(&lcd, "SV:");
		  Lcd_cursor(&lcd, 1,0);
		  Lcd_string(&lcd, "SI:");

		  Lcd_cursor(&lcd, 0,7);
		  Lcd_string(&lcd, "DV:");
		  Lcd_cursor(&lcd, 1,7);
		  Lcd_string(&lcd, "DI:");
	}
	if(DpuVol < 26000)
	{
		   DpuVol=DpuVol+0;

		  Lcd_cursor(&lcd, 0,10);
		  Lcd_string(&lcd, DPU_Char_Vol);
		  Lcd_cursor(&lcd, 1,10);
		  Bitwise();
		  Lcd_string(&lcd, DPU_Char_Vol);
	}
	else if(DpuVol < 42000)
	{
		  DpuVol=DpuVol+0;
		  Bitwise();
		  Lcd_cursor(&lcd, 0,10);
		  Lcd_string(&lcd, DPU_Char_Vol);
		  Lcd_cursor(&lcd, 1,10);
		  Lcd_string(&lcd, DPU_Char_Vol);
	}
	else if(DpuVol < 57000)
	{
		  DpuVol=DpuVol+0;
		  Bitwise();
		  Lcd_cursor(&lcd, 0,10);
		  Lcd_string(&lcd, DPU_Char_Vol);
		  Lcd_cursor(&lcd, 1,10);
		  Lcd_string(&lcd, DPU_Char_Vol);
	}
}


void EncoderAPI(void)
{
 	if(HAL_GPIO_ReadPin(CCLK_ENCOD_GPIO_Port, CCLK_ENCOD_Pin)==GPIO_PIN_SET)
	{
		if(HAL_GPIO_ReadPin(DT_ENCOD_GPIO_Port, DT_ENCOD_Pin)==GPIO_PIN_SET)
		{
			while(HAL_GPIO_ReadPin(DT_ENCOD_GPIO_Port, DT_ENCOD_Pin)==GPIO_PIN_SET){};
 			 SetVolt=SetVolt-Encod_Counter;
 		    while(HAL_GPIO_ReadPin(CCLK_ENCOD_GPIO_Port, CCLK_ENCOD_Pin)==GPIO_PIN_SET){};


		}


	}

	else if(HAL_GPIO_ReadPin(DT_ENCOD_GPIO_Port, DT_ENCOD_Pin)==GPIO_PIN_SET)
	{
		//if(HAL_GPIO_ReadPin(CCLK_ENCOD_GPIO_Port, CCLK_ENCOD_Pin)==GPIO_PIN_SET)
		{
			while(HAL_GPIO_ReadPin(CCLK_ENCOD_GPIO_Port, CCLK_ENCOD_Pin)==GPIO_PIN_SET){};
			 SetVolt=SetVolt+Encod_Counter;
			 //if(Encod_Counter>360) Encod_Counter=360;

		    while(HAL_GPIO_ReadPin(DT_ENCOD_GPIO_Port, DT_ENCOD_Pin)==GPIO_PIN_SET){};
			while(HAL_GPIO_ReadPin(CCLK_ENCOD_GPIO_Port, CCLK_ENCOD_Pin)==GPIO_PIN_SET){};


		}


	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim2)
	{
		DisplayCounter++;
		 ControlVoltage();
		 if(DisplayCounter > 5)
		 {
			 DisplayCounter=0;
			 DPU_Display();
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
