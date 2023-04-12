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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//motor datasheet
#define resolution_MF 512
#define reductionratio_MF 16
#define resolution_ML 500
#define reductionratio_ML 13.2
#define resolution_MR 500
#define reductionratio_MR 13.2

//motor control
#define TIM_ENC_MF &htim1
#define TIM_ENC_ML &htim4
#define TIM_ENC_MR &htim23
#define TIM_PWM_MF &htim2
#define TIM_PWM_ML &htim2
#define TIM_PWM_MR &htim8
#define CH_PWM_MF TIM_CHANNEL_1
#define CH_PWM_ML TIM_CHANNEL_4
#define CH_PWM_MR TIM_CHANNEL_3
#define motor_span 0.001

//pin names
#define INA_MR_PORT GPIOE
#define INA_MR_PIN GPIO_PIN_8
#define INB_MR_PORT GPIOE
#define INB_MR_PIN GPIO_PIN_7
#define INA_ML_PORT GPIOE
#define INA_ML_PIN GPIO_PIN_10
#define INB_ML_PORT GPIOE
#define INB_ML_PIN GPIO_PIN_12
#define INA_MF_PORT GPIOE
#define INA_MF_PIN GPIO_PIN_6
#define INB_MF_PORT GPIOE
#define INB_MF_PIN GPIO_PIN_15

//motor PWM output
#define motorARR 49 //ARR of TIM2

#define pi 3.14159265359
#define theta_1 pi/3
#define theta_2 pi/6

#define ratio_motor2wheel 1.6 //motor 1 revolution wheel 32/20 revolution
#define L 100
#define wheel_radius 12.5 //how much?

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim23;

/* USER CODE BEGIN PV */
double Vx = 0;
double Vy = 0;
double W = 0;

double A = 0;

//encoder & motor speed variables
int16_t enc_MF;
double speed_MF;
int16_t enc_ML;
double speed_ML;
int16_t enc_MR;
double speed_MR;

double spd_cmd_MF = 0; //speed command for front motor
double spd_cmd_ML = 0; //speed command for left motor
double spd_cmd_MR = 0; //speed command for right motor

double err_MF, err_old_MF=0; //initialize?
double err_MR, err_old_ML=0; //initialize?
double err_MR, err_old_MR=0; //initialize?

float kp_MF = 1.8149;
float ki_MF = 33.1176;
float kd_MF = 0;
float kp_ML = 2.044;
float ki_ML = 38.9168;
float kd_ML = 0;
float kp_MR = 2.1367;
float ki_MR = 40.502;
float kd_MR = 0;

double inte_MF = 0;
double diff_MF = 0;
double PID_MF = 0;
double inte_ML = 0;
double diff_ML = 0;
double PID_ML = 0;
double inte_MR = 0;
double diff_MR = 0;
double PID_MR = 0;

int straight_test_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM23_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM23_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(TIM_ENC_MF, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(TIM_ENC_MF, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(TIM_ENC_ML, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(TIM_ENC_ML, TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(TIM_ENC_MR, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(TIM_ENC_MR, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(TIM_PWM_MF, CH_PWM_MF);
	HAL_TIM_PWM_Start(TIM_PWM_ML, CH_PWM_ML);
	HAL_TIM_PWM_Start(TIM_PWM_MR, CH_PWM_MR);
  HAL_TIM_Base_Start_IT(&htim5);
  straight_test_cnt = 8000; //go straight 8 seconds-----------------------------------
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 63;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 63;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 49;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 0;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 4294967295;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim23, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE6 PE7 PE8 PE10
                           PE12 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void pwmO_MF(double u);
double PIDcalc_MF(double sp, double speed);
void pwmO_ML(double u);
double PIDcalc_ML(double sp, double speed);
void pwmO_MR(double u);
double PIDcalc_MR(double sp, double speed);

double Va;
double Vb;
double Vc;

double bhv_MF[500];
double bhv_ML[500];
double bhv_MR[500];
int arrcnt_MF = 0;
int arrcnt_ML = 0;
int arrcnt_MR = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){

		if(straight_test_cnt>0){
			Vx = 10;
			Vy = 0;
			W = 0;
			straight_test_cnt--;
		}
		else if(straight_test_cnt == 0){
			Vx = 0;
			Vy = 0;
			W = 0;
		}

		//car speed to motor speed
		Va = cos(A)*Vx + sin(A)*Vy - L*W;
	    Vb = ((-1)*cos(theta_1)*cos(A) + sin(theta_1)*sin(A))*Vx + ((-1)*cos(theta_1)*sin(A)-sin(theta_1)*cos(A))*Vy - L*W;
	    Vc = ((-1)*sin(theta_2)*cos(A) - cos(theta_2)*sin(A))*Vx + ((-1)*sin(theta_2)*sin(A)+cos(theta_2)*cos(A))*Vy - L*W;

	    spd_cmd_MF = Va/wheel_radius/ratio_motor2wheel;
	    spd_cmd_MR = Vb/wheel_radius/ratio_motor2wheel*(-1); //因為這顆輪子就是反方向轉，所以給指令也要反的!
	    spd_cmd_ML = Vc/wheel_radius/ratio_motor2wheel;

		/* �???�馬??��?��?? */
		//front wheel motor
		enc_MF = __HAL_TIM_GetCounter(TIM_ENC_MF);
		speed_MF = (double) enc_MF / (4*resolution_MF * reductionratio_MF)/motor_span;
		__HAL_TIM_SetCounter(TIM_ENC_MF, 0);
		if(spd_cmd_MF != 0){
			if(arrcnt_MF<500){
				bhv_MF[arrcnt_MF]=speed_MF;
				arrcnt_MF++;
			}
		}
		//left wheel motor
		enc_ML = __HAL_TIM_GetCounter(TIM_ENC_ML);
		speed_ML = (double) enc_ML / (4*resolution_ML * reductionratio_ML)*1000;
		speed_ML = -speed_ML;
		__HAL_TIM_SetCounter(TIM_ENC_ML, 0);
		if(spd_cmd_ML != 0){
			if(arrcnt_ML<500){
				bhv_ML[arrcnt_ML]=speed_ML;
				arrcnt_ML++;
			}
		}
		//right wheel motor
		enc_MR = __HAL_TIM_GetCounter(TIM_ENC_MR);
		speed_MR = (double) enc_MR / (4*resolution_MR * reductionratio_MR *motor_span);
		__HAL_TIM_SetCounter(TIM_ENC_MR, 0);
		if(spd_cmd_MR != 0){
			if(arrcnt_MR<500){
				bhv_MR[arrcnt_MR]=speed_MR;
				arrcnt_MR++;
			}
		}

		/*計�?�PID*/
		PID_MF = PIDcalc_MF(spd_cmd_MF, speed_MF);
		PID_ML = PIDcalc_ML(spd_cmd_ML, speed_ML);
		PID_MR = PIDcalc_MR(spd_cmd_MR, speed_MR);

		/* �? u(t) 轉�?��?? PWM 訊�?? */
		pwmO_MF(PID_MF);
		pwmO_ML(PID_ML);
		pwmO_MR(PID_MR);
	}

}


void pwmO_MF(double u){
		int pulse;
		if (u > 0) {
				pulse = (int) (u * (motorARR+1));
				HAL_GPIO_WritePin(INA_MF_PORT, INA_MF_PIN, GPIO_PIN_SET); // INA
				HAL_GPIO_WritePin(INB_MF_PORT, INB_MF_PIN, GPIO_PIN_RESET); // INB
		}
		else {
				pulse = (int) (-u * (motorARR+1));
				HAL_GPIO_WritePin(INA_MF_PORT, INA_MF_PIN, GPIO_PIN_RESET); // INA
				HAL_GPIO_WritePin(INB_MF_PORT, INB_MF_PIN, GPIO_PIN_SET); // INB
		}
		__HAL_TIM_SET_COMPARE(TIM_PWM_MF, CH_PWM_MF, pulse); // PWM
}
int pulse_ML;
void pwmO_ML(double u){
		if (u > 0) {
				pulse_ML = (int) (u * (motorARR+1));
				HAL_GPIO_WritePin(INA_ML_PORT, INA_ML_PIN, GPIO_PIN_SET); // INA
				HAL_GPIO_WritePin(INB_ML_PORT, INB_ML_PIN, GPIO_PIN_RESET); // INB
		}
		else {
				pulse_ML = (int) (-u * (motorARR+1));
				HAL_GPIO_WritePin(INA_ML_PORT, INA_ML_PIN, GPIO_PIN_RESET); // INA
				HAL_GPIO_WritePin(INB_ML_PORT, INB_ML_PIN, GPIO_PIN_SET); // INB
		}
		__HAL_TIM_SET_COMPARE(TIM_PWM_ML, CH_PWM_ML, pulse_ML); // PWM
}
void pwmO_MR(double u){
		int pulse;
		if (u > 0) {
				pulse = (int) (u * (motorARR+1));
				HAL_GPIO_WritePin(INA_MR_PORT, INA_MR_PIN, GPIO_PIN_SET); // INA
				HAL_GPIO_WritePin(INB_MR_PORT, INB_MR_PIN, GPIO_PIN_RESET); // INB
		}
		else {
				pulse = (int) (-u * (motorARR+1));
				HAL_GPIO_WritePin(INA_MR_PORT, INA_MR_PIN, GPIO_PIN_RESET); // INA
				HAL_GPIO_WritePin(INB_MR_PORT, INB_MR_PIN, GPIO_PIN_SET); // INB
		}
		__HAL_TIM_SET_COMPARE(TIM_PWM_MR, CH_PWM_MR, pulse); // PWM
}
double PIDcalc_MF(double sp, double speed){
		/* ?��??�e(t) */
		double error = sp - speed;
		/* 計�?��?��?? */
		inte_MF += error * motor_span;
		double bound = 1/ki_MF;
		if (ki_MF * inte_MF > 1) inte_MF = bound;
		else if (ki_MF * inte_MF < -1) inte_MF = -bound;

		/* 計�?? u(t) */
		float u = kp_MF * error + ki_MF * inte_MF;
		if (u > 1) u = 1;
		else if (u < -1) u = -1;

		return u;
}
double PIDcalc_ML(double sp, double speed){
		/* ?��??�e(t) */
		double error = sp - speed;
		/* 計�?��?��?? */
		inte_ML += error * motor_span;
		double bound = 1/ki_ML;
		if (ki_ML * inte_ML > 1) inte_ML = bound;
		else if (ki_ML * inte_ML < -1) inte_ML = -bound;

		/* 計�?? u(t) */
		float u = kp_ML * error + ki_ML * inte_ML;
		if (u > 1) u = 1;
		else if (u < -1) u = -1;

		return u;
}
double PIDcalc_MR(double sp, double speed){
		/* ?��??�e(t) */
		double error = sp - speed;
		/* 計�?��?��?? */
		inte_MR += error * motor_span;
		double bound = 1/ki_MR;
		if (ki_MR * inte_MR > 1) inte_MR = bound;
		else if (ki_MR * inte_MR < -1) inte_MR = -bound;

		/* 計�?? u(t) */
		float u = kp_MR * error + ki_MR * inte_MR;
		if (u > 1) u = 1;
		else if (u < -1) u = -1;

		return u;
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
