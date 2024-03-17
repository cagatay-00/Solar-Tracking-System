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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ILI9341_Lib/ili9341.h"
#include "ILI9341_Lib/fonts.h"
#include "stdlib.h"
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t row  = 0;
uint32_t page = 0;

uint32_t track_flag = 0;
uint32_t monitor_flag = 0;

uint32_t lastDebounceTime = 0;
uint32_t debounceDelay = 50;

uint32_t ADC1_Value[2];
uint32_t ADC2_Value[2];

float current, voltage, ACS712_Value;
uint32_t LDR[4];
int rt, lt, rd, ld;

uint32_t alt_servo = 75, ust_servo = 25;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Girizgah (void)
{
	ILI9341_FillScreen(ILI9341_WHITE);

	ILI9341_WriteString(15, 50, " EGE UNIVERSITESI ", Font_16x26, ILI9341_BLUE, ILI9341_WHITE);
	HAL_Delay(1000);
	ILI9341_WriteString(55, 140, "ELEKTRIK ELEKTRONIK", Font_11x18, ILI9341_BLUE, ILI9341_WHITE);
	ILI9341_WriteString(80, 160, " MUHENDISLIGI ", Font_11x18, ILI9341_BLUE, ILI9341_WHITE);
	HAL_Delay(3000);

	ILI9341_FillScreen(ILI9341_YELLOW);

	ILI9341_WriteString(70, 40, "IKI EKSENLI", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
	ILI9341_WriteString(15, 70, "ISIK TAKIP SISTEMI", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
	ILI9341_WriteString(75, 170, "BITIRME PROJESI", Font_11x18, ILI9341_BLACK, ILI9341_YELLOW);
	HAL_Delay(2000);
}

void ScreenMenu (int r, int p)
{
	row  = r;
	page = p;

	switch(page)
	{
	case 1:
		ILI9341_FillScreen(ILI9341_YELLOW);

		ILI9341_WriteString(130, 15, "MENU", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
		ILI9341_WriteString(80, 60, "Gunes Takip", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
		ILI9341_WriteString(80, 100, "Veri Izleme", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
		ILI9341_WriteString(80, 140, "Sabit Sistem", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);

		switch(row)
		{
		case 1:
			ILI9341_WriteString(40, 60, ">>", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
			break;
		case 2:
			ILI9341_WriteString(40, 100, ">>", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
			break;
		case 3:
			ILI9341_WriteString(40, 140, ">>", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
			break;
		}
	break;

	case 2:
			ILI9341_FillScreen(ILI9341_YELLOW);

			switch(row)
			{
			case 1:
				ILI9341_WriteString(80, 100, "TARANIYOR..", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
				track_flag = 1;
				break;
			case 2:
				ILI9341_WriteString(110, 90, "VERILER", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
				ILI9341_WriteString(80, 120, "GETIRILIYOR", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
				monitor_flag = 1;
				break;
			case 3:
				ILI9341_WriteString(70, 110, "SISTEM SABIT", Font_16x26, ILI9341_BLACK, ILI9341_YELLOW);
//				__HAL_GPIO_EXTI_GENERATE_SWIT(SFT_EXTI_INT_Pin);
//				track_flag = 0;
				NVIC_SystemReset();
				break;
			}
	break;

	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t currentTime = HAL_GetTick();
	if(GPIO_Pin == DOWN_Pin)
	{
		if ((currentTime - lastDebounceTime) > debounceDelay) {
			if (HAL_GPIO_ReadPin(DOWN_GPIO_Port, DOWN_Pin) == GPIO_PIN_RESET) {
				switch(row)
				{
				case 1:
					ScreenMenu(2, page);
					break;
				case 2:
					ScreenMenu(3, page);
					break;
				case 3:
					ScreenMenu(1, page);
					break;
				}
			lastDebounceTime = currentTime;
			}
		}
	}

	if(GPIO_Pin == UP_Pin)
	{
		if ((currentTime - lastDebounceTime) > debounceDelay) {
			if (HAL_GPIO_ReadPin(UP_GPIO_Port, UP_Pin) == GPIO_PIN_RESET) {
				switch(row)
				{
				case 1:
					ScreenMenu(3, page);
					break;
				case 2:
					ScreenMenu(1, page);
					break;
				case 3:
					ScreenMenu(2, page);
					break;
				}
			lastDebounceTime = currentTime;
			}
		}
	}

	if(GPIO_Pin == RIGHT_Pin)
	{
		if ((currentTime - lastDebounceTime) > debounceDelay) {
			if (HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin) == GPIO_PIN_RESET) {
				switch(page)
				{
				case 1:
					ScreenMenu(row, 2);
					break;
				case 2:
					ScreenMenu(row, 3);
					break;
				}
				lastDebounceTime = currentTime;
			}
		}
	}

	if(GPIO_Pin == LEFT_Pin)
	{
		if ((currentTime - lastDebounceTime) > debounceDelay) {
			if (HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) == GPIO_PIN_RESET) {
				switch(page)
				{
				case 1:
					ScreenMenu(row, 1);
					break;
				case 2:
					ScreenMenu(row, 1);
					break;
				case 3:
					ScreenMenu(row, 2);
					break;
				}
				lastDebounceTime = currentTime;
			}
		}
	}
	if(GPIO_Pin == SFT_EXTI_INT_Pin)
	{
		track_flag = 0;
		htim1.Instance->CCR1 = 25;
		htim1.Instance->CCR2 = 25;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC1_Value, 2);
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC2_Value, 2);


		LDR[0] = ADC1_Value[0];
		LDR[1] = ADC1_Value[1];
		LDR[2] = ADC2_Value[0];
		LDR[3] = ADC2_Value[1];

		rd = LDR[0] / 100;
		ld = LDR[1] / 100;
		lt = LDR[2] / 100;
		rt = LDR[3] / 100;

 	}

//	if(htim == &htim2)
//	{
//		HAL_ADC_Start_IT(&hadc3);
//	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	if(hadc == &hadc3)
//	{
//		ACS712_Value = HAL_ADC_GetValue(&hadc3);
//		voltage = (float)ACS712_Value * 6 / 4096.0f; // STM32F4 ADC çözünürlüğü 12 bit (4096)
//
//		// ACS712 sensöründen elde edilen akımı hesaplamak için formül: I = (ADC değeri - 2048) / 4096 * 5A
//		current = ((float)ACS712_Value - 2048.0f) / 4096.0f * 5.0f;
//
//		HAL_ADC_Stop_IT(&hadc3);
//	}
//}

void Solar_Tracking(void)
{
	//htim1.Instance->CCR1 --> alt servo
	//htim1.Instance->CCR2 --> üst servo

	if(abs(lt - rt) > 5 && abs(lt - ld) > 5)
	{
		while(abs(lt - rt)/10 != 0)
		{
			htim1.Instance->CCR1 += 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR1 > 125)
				htim1.Instance->CCR1 = 125;
			if((rd + ld) > (rt + lt))
				break;
		}

		while(abs((lt + rt) - (rd + ld))/5 != 0)
		{
			htim1.Instance->CCR2 += 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR2 > 75)
				htim1.Instance->CCR2 = 75;
		}
	}

	if(abs(rt - lt) > 5 && abs(rt - rd) > 5)
	{
		while(abs(rt - lt)/10 != 0)
		{
			htim1.Instance->CCR1 -= 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR1 < 25)
				htim1.Instance->CCR1 = 25;
			if((rd + ld) > (rt + lt))
				break;
		}

		while(abs((lt + rt) - (rd + ld))/5 != 0)
		{
			htim1.Instance->CCR2 += 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR2 > 75)
				htim1.Instance->CCR2 = 75;
		}
	}

	if(abs(rd - rt) > 5 && abs(rd - ld) > 5)
	{
		while(abs(rd - rt)/10 != 0)
		{
			htim1.Instance->CCR1 += 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR1 < 25)
				htim1.Instance->CCR1 = 25;
			if((rd + ld) > (rt + lt))
				break;
		}

		while(abs((ld + rd) - (rt + lt))/5 != 0)
		{
			htim1.Instance->CCR2 -= 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR2 < 25)
				htim1.Instance->CCR2 = 25;
		}
	}

	if(abs(ld - lt) > 5 && abs(ld - rd) > 5)
	{
		while(abs(ld - lt)/10 != 0)
		{
			htim1.Instance->CCR1 -= 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR1 < 25)
				htim1.Instance->CCR1 = 25;
			if((rd + ld) > (rt + lt))
				break;
		}

		while(abs((ld + rd) - (rt + lt))/5 != 0)
		{
			htim1.Instance->CCR2 -= 1;
			HAL_Delay(50);
			if(htim1.Instance->CCR2 < 25)
				htim1.Instance->CCR2 = 25;
		}
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
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  htim1.Instance->CCR1 = alt_servo;
  htim1.Instance->CCR2 = ust_servo;

  ILI9341_Unselect();
  ILI9341_Init();

  //Girizgah();
  ScreenMenu(1,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(track_flag == 1)
	  {
		  //üstteki servo 25-95
		  //alttaki servo 25-75

//		  for(int i = 25; i <= 75; i++)
//		  {
//			  htim1.Instance->CCR1 = i;
//			  htim1.Instance->CCR2 = i;
//			  HAL_Delay(50);
//		  }
//
//		  for(int i = 75; i >= 25; i--)
//		  {
//			  htim1.Instance->CCR1 = i;
//			 htim1.Instance->CCR2 = i;
//			  HAL_Delay(50);
//		  }
		  Solar_Tracking();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
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
  htim1.Init.Prescaler = 3360-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim2.Init.Prescaler = 1680-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  htim3.Init.Prescaler = 168-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50-1;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ILI9341_DC_GPIO_Port, ILI9341_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ILI9341_CS_Pin|ILI9341_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DOWN_Pin RIGHT_Pin UP_Pin LEFT_Pin */
  GPIO_InitStruct.Pin = DOWN_Pin|RIGHT_Pin|UP_Pin|LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SFT_EXTI_INT_Pin */
  GPIO_InitStruct.Pin = SFT_EXTI_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SFT_EXTI_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ILI9341_DC_Pin */
  GPIO_InitStruct.Pin = ILI9341_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ILI9341_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ILI9341_CS_Pin ILI9341_RES_Pin */
  GPIO_InitStruct.Pin = ILI9341_CS_Pin|ILI9341_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

