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
#include <math.h>
#include <stdio.h>
#include "oled.h"
#include "motors.h"
#include "imu.h"
#include "servo.h"
#include "comm.h"
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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorServo */
osThreadId_t motorServoHandle;
const osThreadAttr_t motorServo_attributes = {
  .name = "motorServo",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for imu */
osThreadId_t imuHandle;
const osThreadAttr_t imu_attributes = {
  .name = "imu",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for uart */
osThreadId_t uartHandle;
const osThreadAttr_t uart_attributes = {
  .name = "uart",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ultrasound */
osThreadId_t ultrasoundHandle;
const osThreadAttr_t ultrasound_attributes = {
  .name = "ultrasound",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ori_semaphore */
osSemaphoreId_t ori_semaphoreHandle;
const osSemaphoreAttr_t ori_semaphore_attributes = {
  .name = "ori_semaphore"
};
/* USER CODE BEGIN PV */
uint8_t start = 0;

float orientation = 0;

double pos_x = 0;
double pos_y = 0;

float us_distchange_x = 0;
float us_distchange_y = 0;

char oledbuf[20];
uint8_t buf[20];

uint16_t echo_upEdge = 65535;
uint16_t echo_downEdge = 65535;
uint16_t echo, lastEcho1, lastEcho2;
uint8_t us_alert = 0;
double distToObstacle = 10000;	// Distance to nearesr obstacle detected by ultrasound (cm)
int reqDist = 0;				// Flag: if != 0, us will actively measuring distance
int distReady = 0;				// Set to 1 when us finished active measurement

Instruction curInst;
CompleteError cpltErr;
uint8_t turnDir = 0xFF;
double distToObstacle_min = 25;

MotorData mtrA;
MotorData mtrB;
MotorPIDData mtrAPID;
MotorPIDData mtrBPID;

MotorServoStatus backupObj;

AccelGyroResult gyro;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void StartMotorServo(void *argument);
void StartIMU(void *argument);
void StartUART(void *argument);
void StartUS(void *argument);

/* USER CODE BEGIN PFP */
void Delay_us(uint16_t us);
void usEnableActiveMeasure();
int getIRReading(uint8_t ir);
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
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  uint8_t imuerr = imu_init(&hi2c1);
  if (imuerr != 0) {
	  OLED_Clear();
		sprintf(oledbuf, "Imu err: %d", imuerr);
		OLED_ShowString(10, 15, &oledbuf[0]);
		OLED_Refresh_Gram();
  }
  HAL_TIM_Base_Start(&htim6);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  comm_init(&huart3, &curInst, &cpltErr);
  HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_IT(&huart3, (uint8_t*) buf, UART_PACKET_SIZE);
  mtr_init(&htim8, &htim2, &htim3, &mtrA, &mtrB, &mtrAPID, &mtrBPID, &backupObj, &orientation, &ori_semaphoreHandle);
  servoInit(&htim1);
  //HAL_UART_Receive_IT(&huart1, (uint8_t*) buf, UART_PACKET_SIZE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ori_semaphore */
  ori_semaphoreHandle = osSemaphoreNew(1, 1, &ori_semaphore_attributes);

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

  /* creation of motorServo */
  motorServoHandle = osThreadNew(StartMotorServo, NULL, &motorServo_attributes);

  /* creation of imu */
  imuHandle = osThreadNew(StartIMU, NULL, &imu_attributes);

  /* creation of uart */
  uartHandle = osThreadNew(StartUART, NULL, &uart_attributes);

  /* creation of ultrasound */
  ultrasoundHandle = osThreadNew(StartUS, NULL, &ultrasound_attributes);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE5 PE6 PE7 PE8
                           LED3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : US_TRIG_Pin */
  GPIO_InitStruct.Pin = US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US_TRIG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {		// Ultrasound Echo
		if (echo_upEdge > 20000) {
			echo_upEdge = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		}
		else {
			echo_downEdge = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (echo_upEdge > echo_downEdge) {
				echo = (echo_downEdge + 20000) - echo_upEdge;
			}
			else {
				echo = echo_downEdge - echo_upEdge;
			}
			echo_upEdge = 65535;
			echo_downEdge = 65535;

			/*OLED_Clear();
			sprintf(oledbuf, "Dist: %5.1f", echo * 0.01715f);
			OLED_ShowString(10, 15, &oledbuf[0]);
			OLED_Refresh_Gram();*/

			// Calculate distance
			if (reqDist > 0) {
				if (((abs(lastEcho1 - echo) > 300) || (abs(lastEcho2 - echo) > 300)) || (abs(lastEcho1 - lastEcho2) > 300)) {	// System not stabilised yet, wait
					lastEcho2 = lastEcho1;
					lastEcho1 = echo;
					return;
				}
				if (us_alert < 5) {
					if (distToObstacle > 9999) {
						distToObstacle = 0;
					}
					distToObstacle += echo * 0.01715f;
					us_alert++;
				}
				else {
					distToObstacle /= 5;
					us_alert = 0;
					if (distToObstacle < distToObstacle_min) {
						reqDist = -1;
					}
					else {
						reqDist = 0;
					}
					distReady = 1;
				}
			}
			else if (reqDist == 0) {
				if (!cpltErr.finished) {	// Car is running, needs passive measurement
					if (echo * 0.01715f <= distToObstacle_min) {
						stopPID();
						mtrA.suspend = SUS_STOP;
						mtrB.suspend = SUS_STOP;
						// Activate active measurement to confirm
						usEnableActiveMeasure();
					}
				}
			}
		}
	}
	if (htim == &htim2) {		// Motor A's interrupt
		mtrAPID.count = -(int16_t)__HAL_TIM_GET_COUNTER(htim);
	}
	if (htim == &htim3) {		// Motor B's interrupt
		mtrBPID.count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	UNUSED(huart);

	//MX_USART3_UART_Init();
	turnDir = uart_receive_cam((uint8_t*) buf);
	if (turnDir == 0x80) {
		start = 1;
		turnDir = 0xFF;
	}
	OLED_Clear();
	sprintf(oledbuf, "%2x %2x %2x %2x", buf[0], buf[1], buf[2], buf[3]);
	OLED_ShowString(10, 15, &oledbuf[0]);
	OLED_Refresh_Gram();
	//huart->RxState = HAL_UART_STATE_READY;
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;

	HAL_UART_Receive_IT(huart, (uint8_t*) buf, UART_PACKET_SIZE);

	/*
	if (uart_stt == HAL_OK) {
		OLED_Clear();
		sprintf(oledbuf, "Id: %d", curInst.id);
		OLED_ShowString(10, 15, &oledbuf[0]);
		OLED_Refresh_Gram();
		sprintf(oledbuf, "Type: %d", curInst.type);
		OLED_ShowString(10, 30, &oledbuf[0]);
		OLED_Refresh_Gram();
		sprintf(oledbuf, "Val: %d", curInst.val);
		OLED_ShowString(10, 45, &oledbuf[0]);
		OLED_Refresh_Gram();
	}
	else {
		OLED_Clear();
		sprintf(oledbuf, "Err: %d", buf[3]);
		OLED_ShowString(10, 15, &oledbuf[0]);
		OLED_Refresh_Gram();
		sprintf(oledbuf, "CpltErr: %d", cpltErr.id);
		OLED_ShowString(10, 30, &oledbuf[0]);
		OLED_Refresh_Gram();
		sprintf(oledbuf, "Fin: %d", cpltErr.finished);
		OLED_ShowString(10, 45, &oledbuf[0]);
		OLED_Refresh_Gram();
	}
	*/
}

void Delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < us);
	return;
}

void usEnableActiveMeasure() {
	distReady = 0;
	distToObstacle = 10000;
	reqDist = 1;
}

int getIRReading(uint8_t ir) {
	int retval;
	for (int i = 0; i < 5; i++) {
		if (ir == LEFT) {
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 10);
			retval += HAL_ADC_GetValue(&hadc2);
			HAL_ADC_Stop(&hadc2);
		}
		else if (ir == RIGHT) {
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 10);
			retval += HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
		}
	}
	return retval/5;
}
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
    osDelay(10000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorServo */
/**
* @brief Function implementing the motorServo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorServo */
void StartMotorServo(void *argument)
{
  /* USER CODE BEGIN StartMotorServo */
	uint8_t carTurnOk = 0;
	uint8_t selectedIr = 0xFF;
	double distPreRun = 0;
	double distPostRun = 0;
	double distTmp, obs2Width;
	int ir;
	/*for (;;) {
		OLED_Clear();
		sprintf(oledbuf, "L: %d", getIRReading(LEFT));
		OLED_ShowString(10, 15, &oledbuf[0]);
		OLED_Refresh_Gram();
		sprintf(oledbuf, "R: %d", getIRReading(RIGHT));
		OLED_ShowString(10, 30, &oledbuf[0]);
		OLED_Refresh_Gram();
		osDelay(200);
	}*/
	//carTurn(1, 270);

	//carTurn(1, 45);
	//carTurn(1, 315);
	//return;
  /* Infinite loop */
  for(;;)
  {
	  // Step 0: Check for start condition
	  if (!start) continue;

	  // Step 1: Move towards 1st obstacle, safe distance = 30 - 35 cm from obstacle
	  distToObstacle_min = 30;
	  do {
		  // Request us distance
		  usEnableActiveMeasure();
		  while (!distReady) osDelay(10);
		  if (distPreRun == 0) {
			  distPreRun = distToObstacle;
		  }
		  if ((distToObstacle - distToObstacle_min > -1) && (distToObstacle - distToObstacle_min < 1)) {
			  break;
		  }
		  cpltErr.finished = 0;
		  pos_y += mtr_mov_cm(distToObstacle - distToObstacle_min);
		  cpltErr.finished = 1;
		  while ((mtrA.suspend) || (mtrB.suspend)) {	// Operation is interrupted by us
			  while (!distReady) osDelay(10);
			  reqDist = -1;
			  mtrA.suspend = SUS_OFF;
			  mtrB.suspend = SUS_OFF;
			  cpltErr.finished = 0;
			  pos_y += mtr_mov_cm(distToObstacle - distToObstacle_min);
			  cpltErr.finished = 1;
		  }
		  usEnableActiveMeasure();
		  while (!distReady) osDelay(10);
	  } while((distToObstacle < distToObstacle_min) || (distToObstacle > distToObstacle_min + 5));
	  distPostRun = distToObstacle;
	  pos_y += distPreRun - distPostRun;

	  // Step 2: Request camera and wait for result
	  uart_send_cam(1);
	  turnDir = LEFT;		// Testing only
	  while (turnDir == 0xFF);

	  // Step 3: Turn past 1st obstacle (10x10)
	  reqDist = -1;
	  if (turnDir == LEFT) {
		  cpltErr.finished = 0;
		  carTurn(1, 45);
		  carTurn(1, 255);
		  mtr_mov_cm(7);
		  turn(60);
		  cpltErr.finished = 1;
		  pos_y += 96.5;
	  }
	  else if (turnDir == RIGHT) {
		  // Not implemented yet
		  cpltErr.finished = 0;
		  carTurn(1, 315);
		  carTurn(1, 105);
		  mtr_mov_cm(7);
		  turn(300);
		  cpltErr.finished = 1;
		  pos_y += 104.3;
	  }

	  // Step 4: Move towards 2nd obstacle, safe distance = 40 - 45 cm from obstacle (if carTurnOk)
	  // 25 - 30 cm from obstacle (if not carTurnOk)
	  usEnableActiveMeasure();
	  while (!distReady) osDelay(10);
	  distPreRun = distToObstacle;
	  if (distToObstacle <= 35) {
		  carTurnOk = 0;
		  distToObstacle_min = 16;
	  }
	  else {
		  carTurnOk = 1;
		  distToObstacle_min = 45;
	  }

	  do {
		  usEnableActiveMeasure();
		  while (!distReady) osDelay(10);
		  if ((distToObstacle - distToObstacle_min > -1) && (distToObstacle - distToObstacle_min < 1)) {
			  break;
		  }
		  cpltErr.finished = 0;
		  pos_y += mtr_mov_cm(distToObstacle - distToObstacle_min);
		  cpltErr.finished = 1;
		  while ((mtrA.suspend) || (mtrB.suspend)) {	// Operation is interrupted by us
			  while (!distReady) osDelay(10);
			  reqDist = -1;
			  mtrA.suspend = SUS_OFF;
			  mtrB.suspend = SUS_OFF;
			  cpltErr.finished = 0;
			  pos_y += mtr_mov_cm(distToObstacle - distToObstacle_min);
			  cpltErr.finished = 1;
		  }
		  usEnableActiveMeasure();
		  while (!distReady) osDelay(10);
	  } while((distToObstacle < distToObstacle_min) || (distToObstacle > distToObstacle_min + 5));
	  distPostRun = distToObstacle;
	  pos_y += distPreRun - distPostRun;

	  // Step 5: Request camera and wait for result
	  turnDir = 0xFF;
	  uart_send_cam(2);
	  turnDir = RIGHT;		// Testing only
	  while (turnDir == 0xFF);

	  // Step 6: Go around 2nd obstacle
	  reqDist = -1;
	  if (turnDir == LEFT) {
		  cpltErr.finished = 0;
		  if (carTurnOk) {
			  carTurn(1, 90);
			  pos_x += -45;
			  pos_y += 39.5;
		  }
		  else {
			  turn(90);
			  pos_x += -10.3;
		  }
		  cpltErr.finished = 1;
		  selectedIr = RIGHT;
	  }
	  else if (turnDir == RIGHT) {
		  cpltErr.finished = 0;
		  if (carTurnOk) {
			  carTurn(1, 270);
			  pos_x += 43.25;
			  pos_y += 30.75;
		  }
		  else {
			  turn(270);
			  pos_x += 15.7;
		  }
		  cpltErr.finished = 1;
		  selectedIr = LEFT;
	  }

	  // Go to edge of 2nd obstacle
	  ir = getIRReading(selectedIr);
	  mtrA_init(0xFFFF, 0, 0, 0, 1);
	  mtrB_init(0xFFFF, 0, 0, 0, 1);
	  if (ir > 800) {
		  while (ir > 800) {
			  mtr_SetParamAndMove(&mtrA, DIR_FWD, 3000);
			  mtr_SetParamAndMove(&mtrB, DIR_FWD, 3000);
			  osDelay(10);
			  ir = getIRReading(selectedIr);
		  }
		  mtr_stop();
		  distTmp = ((double)((mtrAPID.count + mtrBPID.count) / 2) / CNT_PER_CM);
		  //mtr_mov_cm(-5);
	  }
	  else if (ir <= 500) {
		  while (ir <= 500) {
			  mtr_SetParamAndMove(&mtrA, DIR_BCK, 3000);
			  mtr_SetParamAndMove(&mtrB, DIR_BCK, 3000);
			  osDelay(10);
			  ir = getIRReading(selectedIr);
		  }
		  mtr_stop();
		  distTmp = ((double)((mtrAPID.count + mtrBPID.count) / 2) / CNT_PER_CM);
		  //mtr_mov_cm(-5);
	  }
	  if (turnDir == LEFT) {
		  pos_x -= (distTmp - 10);
	  }
	  else if (turnDir == RIGHT) {
		  pos_x += (distTmp + 10);
	  }

	  if (turnDir == LEFT) {
		  cpltErr.finished = 0;
		  carTurn(1, 270);
		  carTurn(1, 270);
		  cpltErr.finished = 1;
		  pos_x += 12.5;
		  pos_y += 74;
	  }
	  else if (turnDir == RIGHT) {
		  cpltErr.finished = 0;
		  carTurn(1, 90);
		  carTurn(1, 90);
		  cpltErr.finished = 1;
		  pos_x += -5.5;
		  pos_y += 84.5;
	  }

	  // Back to start of 2nd obstacle
	  ir = getIRReading(selectedIr);
	  do {
		  mtr_SetParamAndMove(&mtrA, DIR_BCK, 3000);
		  mtr_SetParamAndMove(&mtrB, DIR_BCK, 3000);
		  osDelay(10);
		  ir = getIRReading(selectedIr);
	  } while(ir > 800);
	  mtr_stop();

	  // Fwd to end of 2nd obstacle
	  ir = getIRReading(selectedIr);
	  mtrA_init(0xFFFF, 0, 0, 0, 1);
	  mtrB_init(0xFFFF, 0, 0, 0, 1);
	  mtr_SetParamAndMove(&mtrA, DIR_FWD, 3000);
	  mtr_SetParamAndMove(&mtrB, DIR_FWD, 3000);
	  osDelay(500);
	  while(ir > 800) {
		  osDelay(10);
		  ir = getIRReading(selectedIr);
	  }
	  mtr_stop();
	  obs2Width = ((double)((mtrAPID.count + mtrBPID.count) / 2) / CNT_PER_CM);
	  //mtr_mov_cm(-10);

	  if (turnDir == LEFT) {
		  cpltErr.finished = 0;
		  carTurn(1, 270);
		  cpltErr.finished = 1;
	  }
	  else if (turnDir == RIGHT) {
		  cpltErr.finished = 0;
		  carTurn(1, 90);
		  cpltErr.finished = 1;
	  }

	  // Step 7: Return home
	  mtr_SetParamAndMove(&mtrA, DIR_FWD, 3000);
	  mtr_SetParamAndMove(&mtrB, DIR_FWD, 3000);
	  osDelay(700);
	  if (obs2Width > 65) {
		  mtr_stop();
		  if (turnDir == LEFT) {
			  carTurn(1, 315);
			  carTurn(1, 45);
		  }
		  else if (turnDir == RIGHT) {
			  carTurn(1, 45);
			  carTurn(1, 315);
		  }
	  }
	  mtr_SetParamAndMove(&mtrA, DIR_FWD, 3000);
	  mtr_SetParamAndMove(&mtrB, DIR_FWD, 3000);
	  while (ir <= 500) {
		  osDelay(10);
		  ir = getIRReading(selectedIr);
	  }
	  while (ir > 500) {
		  osDelay(10);
		  ir = getIRReading(selectedIr);
	  }
	  mtr_stop();

	  if (turnDir == LEFT) {
		  cpltErr.finished = 0;
		  carTurn(1, 270);
		  cpltErr.finished = 1;
	  }
	  else if (turnDir == RIGHT) {
		  cpltErr.finished = 0;
		  carTurn(1, 90);
		  cpltErr.finished = 1;
	  }

	  // Move horizontally towards obs 1
	  mtr_SetParamAndMove(&mtrA, DIR_BCK, 3000);
	  mtr_SetParamAndMove(&mtrB, DIR_BCK, 3000);
	  mtrA_init(0xFFFF, 0, 0, 0, 1);
	  mtrB_init(0xFFFF, 0, 0, 0, 1);
	  while ((mtrA.count + mtrB.count) / 2 < 2200) {
		  ir = getIRReading(selectedIr);
		  if (ir < 700) {
			  osDelay(10);
		  }
		  else {
			  break;
		  }
	  }
	  mtr_SetParamAndMove(&mtrA, DIR_FWD, 3000);
	  mtr_SetParamAndMove(&mtrB, DIR_FWD, 3000);
	  while (ir < 700) {
		  ir = getIRReading(selectedIr);
		  osDelay(10);
	  }

	  // Turn towards carpark
	  mtr_mov_cm(-35);
	  carTurn(1, 270);

	  // Go into carpark
	  distToObstacle_min = 10;
	  do {
		  // Request us distance
		  usEnableActiveMeasure();
		  while (!distReady) osDelay(10);
		  if (distPreRun == 0) {
			  distPreRun = distToObstacle;
		  }
		  if ((distToObstacle - distToObstacle_min > -1) && (distToObstacle - distToObstacle_min < 1)) {
			  break;
		  }
		  cpltErr.finished = 0;
		  pos_y += mtr_mov_cm(distToObstacle - distToObstacle_min);
		  cpltErr.finished = 1;
		  while ((mtrA.suspend) || (mtrB.suspend)) {	// Operation is interrupted by us
			  while (!distReady) osDelay(10);
			  reqDist = -1;
			  mtrA.suspend = SUS_OFF;
			  mtrB.suspend = SUS_OFF;
			  cpltErr.finished = 0;
			  pos_y += mtr_mov_cm(distToObstacle - distToObstacle_min);
			  cpltErr.finished = 1;
		  }
		  usEnableActiveMeasure();
		  while (!distReady) osDelay(10);
	  } while((distToObstacle < distToObstacle_min) || (distToObstacle > distToObstacle_min + 5));


	  OLED_Clear();
	  sprintf(oledbuf, "x: %5.1f", pos_x);
	  OLED_ShowString(10, 15, &oledbuf[0]);
	  OLED_Refresh_Gram();

	  sprintf(oledbuf, "y: %5.1f", pos_y);
	  OLED_ShowString(10, 30, &oledbuf[0]);
	  OLED_Refresh_Gram();


	  break;
	  /*
	  float dist = executeInstruction(&curInst, &cpltErr);
	  if ((mtrA.suspend != SUS_OFF) || (mtrB.suspend != SUS_OFF)) {
		  mtr_continue();
		  mtr_stop();
	  }
	  if (dist != 0) {
		  pos_x += dist * (float)sin((orientation / 180) * PI);
		  pos_y += dist * (float)cos((orientation / 180) * PI);
	  }
	  else {
		  if (us_distchange_x != 0) {		// When the command is turn and there's US course correction
			  pos_x += us_distchange_x;
		  }
		  if (us_distchange_y != 0) {
			  pos_y += us_distchange_y;
		  }
	  }
	  // Reset us_distchange after each instruction run
	  us_distchange_x = 0;
	  us_distchange_y = 0;

	  cpltErr.pos_x = (int16_t)pos_x;
	  cpltErr.pos_y = (int16_t)pos_y;

	  cpltErr.finished = 1;
	  */
	  /*
	  OLED_Clear();
	  sprintf(oledbuf, "X = %5.1f", pos_x);
	  OLED_ShowString(10, 15, &oledbuf[0]);
	  OLED_Refresh_Gram();
	  sprintf(oledbuf, "Y = %5.1f", pos_y);
	  OLED_ShowString(10, 30, &oledbuf[0]);
	  OLED_Refresh_Gram();
	  sprintf(oledbuf, "Ori = %5.1f", orientation);
	  OLED_ShowString(10, 45, &oledbuf[0]);
	  OLED_Refresh_Gram();
	  */
	  //osDelay(500);		// Make sure to give time for UART task to transmit instructions
  }
  /* USER CODE END StartMotorServo */
}

/* USER CODE BEGIN Header_StartIMU */
/**
* @brief Function implementing the imu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU */
void StartIMU(void *argument)
{
  /* USER CODE BEGIN StartIMU */
	uint32_t ori_lastSampleTime = 0;
  /* Infinite loop */
  for(;;)
  {
	  orientation = calcOri(&ori_lastSampleTime, orientation);
	  /*OLED_Clear();
	  sprintf(oledbuf, "Ori = %5.1f", orientation);
	  OLED_ShowString(10, 45, &oledbuf[0]);
	  OLED_Refresh_Gram();*/
	  /*if (cpltErr.finished) {
		  osDelay(5);
	  }*/
  }
  /* USER CODE END StartIMU */
}

/* USER CODE BEGIN Header_StartUART */
/**
* @brief Function implementing the uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART */
void StartUART(void *argument)
{
  /* USER CODE BEGIN StartUART */
  /* Infinite loop */
  for(;;)
  {
	  /*if (!cpltErr.finished) {		// If a task is running, put this task to sleep
		  osDelay(500);
	  }
	  else {
		  // Initiate new task
		  if (curInst.id == cpltErr.id + 1) {	// If a new instruction has been received but has not been processed
			  while (HAL_UART_Receive_IT(&huart3, (uint8_t*) buf, UART_PACKET_SIZE) != HAL_OK) {
				  osDelay(10);
			  }
			  newCpltErr(curInst.id);
		  }
		  // Send results
		  else if (curInst.id == cpltErr.id) {
			  uart_send();
		  }
		  osDelay(100);
	  }
	  */
	  osDelay(10000);
  }
  /* USER CODE END StartUART */
}

/* USER CODE BEGIN Header_StartUS */
/**
* @brief Function implementing the ultrasound thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUS */
void StartUS(void *argument)
{
  /* USER CODE BEGIN StartUS */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
	  osDelay(50);
	  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
	  Delay_us(10);
	  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
	  osDelay(50);
  }
  /* USER CODE END StartUS */
}

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
