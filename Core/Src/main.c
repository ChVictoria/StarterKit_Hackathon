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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sht2x_for_stm32_hal.h"
#include "hd44780.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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

TIM_HandleTypeDef htim4;

/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
uint32_t LCDTaskBuffer[ 128 ];
osStaticThreadDef_t LCDTaskControlBlock;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .cb_mem = &LCDTaskControlBlock,
  .cb_size = sizeof(LCDTaskControlBlock),
  .stack_mem = &LCDTaskBuffer[0],
  .stack_size = sizeof(LCDTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TeHuSensorTask */
osThreadId_t TeHuSensorTaskHandle;
uint32_t TeHuSensorTaskBuffer[ 128 ];
osStaticThreadDef_t TeHuSensorTaskControlBlock;
const osThreadAttr_t TeHuSensorTask_attributes = {
  .name = "TeHuSensorTask",
  .cb_mem = &TeHuSensorTaskControlBlock,
  .cb_size = sizeof(TeHuSensorTaskControlBlock),
  .stack_mem = &TeHuSensorTaskBuffer[0],
  .stack_size = sizeof(TeHuSensorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LightSensorTask */
osThreadId_t LightSensorTaskHandle;
uint32_t LightSensorTaskBuffer[ 128 ];
osStaticThreadDef_t LightSensorTaskControlBlock;
const osThreadAttr_t LightSensorTask_attributes = {
  .name = "LightSensorTask",
  .cb_mem = &LightSensorTaskControlBlock,
  .cb_size = sizeof(LightSensorTaskControlBlock),
  .stack_mem = &LightSensorTaskBuffer[0],
  .stack_size = sizeof(LightSensorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IrSensorTask */
osThreadId_t IrSensorTaskHandle;
uint32_t IrSensorTaskBuffer[ 128 ];
osStaticThreadDef_t IrSensorTaskControlBlock;
const osThreadAttr_t IrSensorTask_attributes = {
  .name = "IrSensorTask",
  .cb_mem = &IrSensorTaskControlBlock,
  .cb_size = sizeof(IrSensorTaskControlBlock),
  .stack_mem = &IrSensorTaskBuffer[0],
  .stack_size = sizeof(IrSensorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UsSensorTask */
osThreadId_t UsSensorTaskHandle;
uint32_t UsSensorTaskBuffer[ 128 ];
osStaticThreadDef_t UsSensorTaskControlBlock;
const osThreadAttr_t UsSensorTask_attributes = {
  .name = "UsSensorTask",
  .cb_mem = &UsSensorTaskControlBlock,
  .cb_size = sizeof(UsSensorTaskControlBlock),
  .stack_mem = &UsSensorTaskBuffer[0],
  .stack_size = sizeof(UsSensorTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempDataMutex */
osMutexId_t TempDataMutexHandle;
osStaticMutexDef_t TempDataMutexControlBlock;
const osMutexAttr_t TempDataMutex_attributes = {
  .name = "TempDataMutex",
  .cb_mem = &TempDataMutexControlBlock,
  .cb_size = sizeof(TempDataMutexControlBlock),
};
/* Definitions for HumDataMutex */
osMutexId_t HumDataMutexHandle;
osStaticMutexDef_t HumDataMutexControlBlock;
const osMutexAttr_t HumDataMutex_attributes = {
  .name = "HumDataMutex",
  .cb_mem = &HumDataMutexControlBlock,
  .cb_size = sizeof(HumDataMutexControlBlock),
};
/* Definitions for THSensorReadySem */
osSemaphoreId_t THSensorReadySemHandle;
osStaticSemaphoreDef_t THSensorReadySemControlBlock;
const osSemaphoreAttr_t THSensorReadySem_attributes = {
  .name = "THSensorReadySem",
  .cb_mem = &THSensorReadySemControlBlock,
  .cb_size = sizeof(THSensorReadySemControlBlock),
};
/* Definitions for IrSensorReadySem */
osSemaphoreId_t IrSensorReadySemHandle;
osStaticSemaphoreDef_t IrSensorReadySemControlBlock;
const osSemaphoreAttr_t IrSensorReadySem_attributes = {
  .name = "IrSensorReadySem",
  .cb_mem = &IrSensorReadySemControlBlock,
  .cb_size = sizeof(IrSensorReadySemControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
void StartLCDTask(void *argument);
void StartTeHuSensorTask(void *argument);
void StartLightSensorTask(void *argument);
void StartIrSensorTask(void *argument);
void StartUsSensorTask(void *argument);

/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned char deg_sym[FONT_HEIGHT] = {0x07,0x05,0x07,0x00,0x00,0x00,0x00,0x00};
float cel;
float rh;
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  //PWM timer
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of TempDataMutex */
  TempDataMutexHandle = osMutexNew(&TempDataMutex_attributes);

  /* creation of HumDataMutex */
  HumDataMutexHandle = osMutexNew(&HumDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexRelease (TempDataMutexHandle);
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of THSensorReadySem */
  THSensorReadySemHandle = osSemaphoreNew(1, 0, &THSensorReadySem_attributes);

  /* creation of IrSensorReadySem */
  IrSensorReadySemHandle = osSemaphoreNew(1, 0, &IrSensorReadySem_attributes);

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
  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &LCDTask_attributes);

  /* creation of TeHuSensorTask */
  TeHuSensorTaskHandle = osThreadNew(StartTeHuSensorTask, NULL, &TeHuSensorTask_attributes);

  /* creation of LightSensorTask */
  LightSensorTaskHandle = osThreadNew(StartLightSensorTask, NULL, &LightSensorTask_attributes);

  /* creation of IrSensorTask */
  IrSensorTaskHandle = osThreadNew(StartIrSensorTask, NULL, &IrSensorTask_attributes);

  /* creation of UsSensorTask */
  UsSensorTaskHandle = osThreadNew(StartUsSensorTask, NULL, &UsSensorTask_attributes);

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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  hadc2.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 160;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_E_Pin LCD_D4_Pin
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_E_Pin|LCD_D4_Pin
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Interrupt callbacks

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {

	  osThreadFlagsSet(LightSensorTaskHandle, 0x00000001U);
	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  }


  else if (hadc->Instance == ADC2)
	  osSemaphoreRelease(IrSensorReadySemHandle);
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	SHT2x_RecieveRaw();
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	osSemaphoreRelease(THSensorReadySemHandle);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLCDTask */
/**
  * @brief  Function implementing the LCDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  lcdInit();
  // load degree symbol
  lcdLoadChar(deg_sym,6);

  lcdPuts("Temp:        ");
  lcdPutc(6);
  lcdPuts("C\nHum :         %");

  /* Infinite loop */
  for(;;)
  {
      osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
	  lcdGoto(1, 6);
	  osMutexAcquire (TempDataMutexHandle, osWaitForever);
	  lcdFtos(cel, 3);
	  osMutexRelease (TempDataMutexHandle);
	  lcdGoto(2, 6);
	  osMutexAcquire (HumDataMutexHandle, osWaitForever);
	  lcdFtos(rh, 3);
	  osMutexRelease (HumDataMutexHandle);
	  osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTeHuSensorTask */
/**
* @brief Function implementing the TeHuSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTeHuSensorTask */
void StartTeHuSensorTask(void *argument)
{
  /* USER CODE BEGIN StartTeHuSensorTask */
  /* Infinite loop */
  for(;;)
  {
	  SHT2x_RequestTemperature(1);
	  //osSemaphoreAcquire (THSensorReadySemHandle,  osWaitForever);
	  osMutexAcquire (TempDataMutexHandle, osWaitForever);
	  cel = SHT2x_GetTemperature();
	  osMutexRelease (TempDataMutexHandle);

	  SHT2x_RequestRelativeHumidity(1);
	  //osSemaphoreAcquire (THSensorReadySemHandle,  osWaitForever);
	  osMutexAcquire (HumDataMutexHandle, osWaitForever);
	  rh = SHT2x_GetRelativeHumidity();
	  osMutexRelease (HumDataMutexHandle);

	  osThreadFlagsSet(LCDTaskHandle, 0x00000001U);
  }
  /* USER CODE END StartTeHuSensorTask */
}

/* USER CODE BEGIN Header_StartLightSensorTask */
/**
* @brief Function implementing the LightSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightSensorTask */
void StartLightSensorTask(void *argument)
{
  /* USER CODE BEGIN StartLightSensorTask */
  uint32_t adcValue;
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x00000001U, osFlagsWaitAny, osWaitForever);
	  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	  adcValue = HAL_ADC_GetValue(&hadc1);

	  float volts = adcValue * 5.0 / 4096.0;
	  float amps = volts / 10000.0;  // across 10,000 Ohms
	  float microamps = amps * 1000000;
	  float lux = microamps * 2.0;

	  uint32_t dutyCycle = 1000 - lux;

	  TIM4->CCR1=dutyCycle;
	  TIM4->CCR3=dutyCycle;
	  TIM4->CCR4=dutyCycle;

	  HAL_ADC_Start_IT(&hadc1);
  }
  /* USER CODE END StartLightSensorTask */
}

/* USER CODE BEGIN Header_StartIrSensorTask */
/**
* @brief Function implementing the IrSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIrSensorTask */
void StartIrSensorTask(void *argument)
{
  /* USER CODE BEGIN StartIrSensorTask */
  uint32_t adcValue;
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire (IrSensorReadySemHandle,  osWaitForever);
	  adcValue = HAL_ADC_GetValue(&hadc2);

	  if (adcValue < 2048)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

		}
		else if (adcValue >= 2048)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		}
	  HAL_ADC_Start_IT(&hadc2);
  }
  /* USER CODE END StartIrSensorTask */
}

/* USER CODE BEGIN Header_StartUsSensorTask */
/**
* @brief Function implementing the UsSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsSensorTask */
void StartUsSensorTask(void *argument)
{
  /* USER CODE BEGIN StartUsSensorTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUsSensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
