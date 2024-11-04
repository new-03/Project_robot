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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI            3.1415926545 
#define chieuDaiLink1 10
#define chieuDaiLink2 18

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum{doc_du_lieu, tinh_dong_hoc_thuan, tinh_dong_hoc_nghich, run} state_machine;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void dongHocThuan(float TT1, float TT2);
void dongHocNghich(float xP, float yP);
int checkTheTa1(float TT1, float TT2, float xP_true, float yP_true);
void Run(float TT1, float TT2);
void run_home(void);
void guiDuLieu_ketThuc(void);
void guiDuLieu_T(void);
void guiDuLieu_N(void);
void guiDuLieu_ERROR(void);
void xuLyDuLieu(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float	gocQuay1 = 0.0, gocQuay2 = 0.0;
const float saiSo = 0.1;
float goc1HienTai = 0.0, goc2HienTai = 0.0;
int end = 0;
int runHome = 0;

char thuanOrNgich = 'A';

float toaDoX = 0.0, toaDoY = 0.0;
float theTa1 = 0.0, theTa2 = 0.0;
float theTa1N = 0.0, theTa2N = 0.0;
int error = 0;

uint8_t dataRX[14];

static state_machine currentState = doc_du_lieu;

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	run_home();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(currentState)
		{
			case doc_du_lieu:
			{
				HAL_UART_Receive(&huart2, dataRX, sizeof(dataRX), 20);
				if (dataRX[0] != 0)
				{
					xuLyDuLieu();
				}
				break;
			}
			
			case tinh_dong_hoc_thuan:
			{
				dongHocThuan(theTa1, theTa2);
				if (error == 1)
				{
					error = 0;
					guiDuLieu_ERROR();
					currentState = doc_du_lieu;
					break;
				}
				guiDuLieu_T();
				currentState = run;
				break;
			}
			
			case tinh_dong_hoc_nghich:
			{
				dongHocNghich(toaDoX, toaDoY);
				if (error == 1)
				{
					error = 0;
					guiDuLieu_ERROR();
					currentState = doc_du_lieu;
					break;
				}					
				guiDuLieu_N();
				currentState = run;
				break;
			}
			
			case run:
			{
				if (runHome == 1)
				{
					Run(0.0, 0.0);
					if (end == 1)
					{
						runHome = 0;
						end = 0;
						currentState = doc_du_lieu;
					}
					break;
				}
				
				Run(theTa1, theTa2);
				if (end == 1)
				{
					end = 0;
					HAL_Delay(30);
					guiDuLieu_ketThuc();
					currentState = doc_du_lieu;
				}
				break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//====================================================================
//DONG HOC
//====================================================================
void dongHocThuan(float TT1, float TT2)
{
	if (TT1 < 30.0 || TT1 > 150.0 || TT2 < 30.0 || TT2 > 150.0)
	{
		error = 1;
		return;
	}
	
	float theTa1F = TT1 * PI / 180.0;
	float theTa2F = TT2 * PI / 180.0;
	
	toaDoX = chieuDaiLink1 * cos(theTa1F) + chieuDaiLink2 * cos(theTa1F + theTa2F);
	toaDoY = chieuDaiLink1 * sin(theTa1F) + chieuDaiLink2 * sin(theTa1F + theTa2F);
}

//====================================================================
void dongHocNghich(float xP, float yP)
{
	float q1_1, q1_2;
	float q2_1, q2_2;
	float maxReach = chieuDaiLink1 + chieuDaiLink2;
	
	/*CHECK WORKBASE*/
	if (sqrt(chieuDaiLink1 * chieuDaiLink1 + chieuDaiLink2 * chieuDaiLink2) > maxReach)
	{
		error = 1;
		return;
	}
	
	q2_1 = acos((pow(xP, 2) + pow(yP, 2) - pow(chieuDaiLink1, 2) - pow(chieuDaiLink2, 2)) / (2 * chieuDaiLink1 * chieuDaiLink2));
	q2_2 = -q2_1;
	
	q1_1 = atan(yP / xP) - atan((chieuDaiLink2 * sin(q2_1)) / (chieuDaiLink1 + chieuDaiLink2 * cos(q2_1)));
	q1_2 = atan(yP / xP) + atan((chieuDaiLink2 * sin(q2_1)) / (chieuDaiLink1 + chieuDaiLink2 * cos(q2_1)));
	
	q1_1 = q1_1 * (180.0 / PI);
	q1_2 = q1_2 * (180.0 / PI);
	q2_1 = q2_1 * (180.0 / PI);
	q2_2 = q2_2 * (180.0 / PI);
	
	/*CHECK NGHIEM*/
	if (checkTheTa1(q1_1, q2_1, xP, yP) == 1)
	{
		if (q1_1 >= 30.0 && q1_1 <= 150.0 && q2_1 >= 30.0 && q2_1 <= 150.0)
		{
			theTa1 = (round(q1_1 * 10))/10;
			theTa1N = (round(q1_2 * 10))/10;
			theTa2 = (round(q2_1 * 10))/10;
			theTa2N = (round(q2_2 * 10))/10;
			return;
		}
		
		if (q1_2 >= 30.0 && q1_2 <= 150.0 && q2_2 >= 30.0 && q2_2 <= 150.0)
		{
			theTa1 = (round(q1_2 * 10))/10;
			theTa1N = (round(q1_1 * 10))/10;
			theTa2 = (round(q2_2 * 10))/10;
			theTa2N = (round(q2_1 * 10))/10;
			return;
		}
	}
	else
	{
		//TIM NGHIEM MOI NEU NGHIEM DAU TIEN KHONG DUNG
		/*Q1_1*/
		if (q1_1 < 0.0)
		{
			q1_1 = q1_1 + 180.0;
		}
		else if (q1_1 > 0.0)
		{
			q1_1 = q1_1 - 180.0;
		}
		else if (q1_1 == 180.0 || q1_1 == -180.0 || q1_1 == 0)
		{
			q1_1 = 0;
		}
	
		/*Q1_2*/
		if (q1_2 < 0.0)
		{
			q1_2 = q1_2 + 180.0;
		}
		else if (q1_2 > 0.0)
		{
			q1_2 = q1_2 - 180.0;
		}
		else if (q1_2 == 180.0 || q1_2 == -180.0 || q1_2 == 0)
		{
			q1_2 = 0;
		}
	
		if (q1_1 >= 30.0 && q1_1 <= 150.0 && q2_1 >= 30.0 && q2_1 <= 150.0)
		{
			theTa1 = (round(q1_1 * 10))/10;
			theTa1N = (round(q1_2 * 10))/10;
			theTa2 = (round(q2_1 * 10))/10;
			theTa2N = (round(q2_2 * 10))/10;
			return;
		}
	}
	error = 1;
}

//====================================================================
int checkTheTa1(float TT1, float TT2, float xP_true, float yP_true)
{
	float TT1F = TT1 * PI / 180.0;
	float TT2F = TT2 * PI / 180.0;
	
	float xP = chieuDaiLink1 * cos(TT1F) + chieuDaiLink2 * cos(TT1F + TT2F);
	float yP = chieuDaiLink1 * sin(TT1F) + chieuDaiLink2 * sin(TT1F + TT2F);
	
	if (xP < xP_true + saiSo && xP > xP_true - saiSo && yP < yP_true + saiSo && yP > yP_true - saiSo)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//====================================================================
//RUN
//====================================================================
void Run(float TT1, float TT2)
{
	/*SERVO 1*/
	gocQuay1 = TT1 - goc1HienTai;
	if (gocQuay1 > 0)
	{
		goc1HienTai += 0.1;
		if (goc1HienTai < TT1)
		{
			htim1.Instance->CCR1 = servo1Write(goc1HienTai);
		}
	}
	if (gocQuay1 < 0)
	{
		goc1HienTai -= 0.1;
		if (goc1HienTai > TT1)
		{
			htim1.Instance->CCR1 = servo1Write(goc1HienTai-2.0); //Khu ro dong co
		}
	}
	
	/*SERVO 2*/
	gocQuay2 = TT2 - goc2HienTai;
	if (gocQuay2 > 0)
	{
		goc2HienTai += 0.1;
		if (goc2HienTai < TT2)
		{
			htim1.Instance->CCR2 = servo2Write(goc2HienTai);
		}
	}
	if (gocQuay2 < 0)
	{
		goc2HienTai -= 0.1;
		if (goc2HienTai > TT2)
		{
			htim1.Instance->CCR2 = servo2Write(goc2HienTai-2.0); //Khu ro dong co
		}
	}
	
	HAL_Delay(1);
	
	/*KET THUC*/
	if (fabs(goc1HienTai - TT1) < saiSo && fabs(goc2HienTai - TT2) < saiSo)
	{
		end = 1;
	}
}

//====================================================================
void run_home(void)
{
	htim1.Instance->CCR1 = servo1Write(0);
	htim1.Instance->CCR2 = servo2Write(0);
}

//====================================================================
//TRUYEN THONG UART
//====================================================================
void guiDuLieu_ketThuc(void)
{
	uint8_t data[6] = "*END#";
	HAL_UART_Transmit(&huart2, data, sizeof(data), 10000);
}

//====================================================================
void guiDuLieu_T(void)
{
	uint8_t data[13];
	sprintf((char*)data, "%.2f:%.2f#", toaDoX, toaDoY);
	HAL_UART_Transmit(&huart2, data, sizeof(data), 10000);
}

//====================================================================
void guiDuLieu_N(void)
{
	uint8_t data[30];
	sprintf((char*)data, "%.2f:%.2f:%.2f:%.2f#", theTa1, theTa2, theTa1N, theTa2N);
	HAL_UART_Transmit(&huart2, data, sizeof(data), 10000);
}

//====================================================================
void guiDuLieu_ERROR(void)
{
	uint8_t data[10] = "*ERROR#";
	HAL_UART_Transmit(&huart2, data, sizeof(data), 10000);
}

//====================================================================
void xuLyDuLieu(void)
{
	float x, y;
	
	sscanf((char *)dataRX, "%c:%f:%f", &thuanOrNgich, &x, &y);
	if (thuanOrNgich == 'T')
	{
		theTa1 = x;
		theTa2 = y;
	}
	else if (thuanOrNgich == 'N')
	{
		toaDoX = x;
		toaDoY = y;
	}
	
	if (thuanOrNgich == 'T')
	{
		currentState = tinh_dong_hoc_thuan;
	}
	else if (thuanOrNgich == 'N')
	{
		currentState = tinh_dong_hoc_nghich;
	}
	
	HAL_Delay(10);
	
	for (int i = 0; i <= sizeof(dataRX); i++)
	{
		dataRX[i] = 0;
	}
}

//====================================================================
//TRANG THAI HOME
//====================================================================
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0) 
	{
		currentState = run;
		runHome = 1;
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
