/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum State {Initial=1,
					Line_Search,Line_Search2,
					TurnRight,TurnRight2,TurnLeft,
					GoStraight_Until_Barrier,
					Go_Mile_1,Go_Mile_2_Until_Barrier,Go_Line_Follow,Go_to_Bridge,
					Cross_bridge,
					Mile_Adjust,
					Idle,
					Unknow} State;
typedef struct Angle
{
	float  x;
	float  y;
	float  z;
} Angle;
typedef struct Distance
{
	float  front;
	float  right;
} Distance;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PWM_Mid 1000  //无反馈时电机工作占空
#define PWM_Lowest 500
#define PWM_Higest 1500 //for our motor, this value should less than 1300
#define Angle_stable_cycles 3
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId StreamHandle;
osThreadId GyroReceiveHandle;
osThreadId DistanceCheckHandle;
osThreadId MileageHandle;
osThreadId GoStraightHandle;
osThreadId LineSearchHandle;
osThreadId LineSearch2Handle;
osSemaphoreId CameraUARTSemHandle;
osSemaphoreId GyroReadySemHandle;
osSemaphoreId CriticalDistanceSemHandle;
osSemaphoreId MileageSemHandle;
osSemaphoreId MileageNegSemHandle;
/* USER CODE BEGIN PV */
State state;
State temp_state;
volatile uint8_t Rx_Buf[2]={0,0};
//uint8_t Rx_Buf_Sonic[3]={0,0,0};
//volatile uint8_t OpenmvData[2]={0,0};
volatile uint16_t Camera_Data=0x0000;
Angle angle={0.0,0.0,0.0};
//Distance distance={0.0,0.0};
Distance critical_distance={0.0,0.0};
Distance current_distance={0,0};
int distance_flag=0;
int gyro_reset_flag=0;
int camera_recieve_IT_flag=0;
int go_straight_speed=PWM_Mid;
//Encoder PV
int32_t mileage_IT_number=-1;
//uint8_t direction;
int32_t number_of_pulses=0;
int32_t critical_pulses=0;
int PID_Straight_Reset_Flag=1;
//Encoder PV END

//PID PV

//PID PV END

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
void StreamTask(void const * argument);
void GyroReceiveTask(void const * argument);
void DistanceCheckTask(void const * argument);
void MileageTask(void const * argument);
void GoStraightTask(void const * argument);
void LineSearchTask(void const * argument);
void LineSearch2Task(void const * argument);

/* USER CODE BEGIN PFP */
void Car_Initial(void);
void Car_Stop(void);
uint8_t State_Transition(State* current_state);
void PWM_SET_LEFT(int32_t duty);
void PWM_SET_RIGHT(int32_t duty);
float Angle_Diff(float target, float input);
//int Angle_Stable_Check(float Error, float Accept_Error, int wait_cycles);
int PID_Turning(float increment_angle,float Accept_Error);
void PID_Straight(float speed);
float PID_Line_Follow(float Accept_Error);
Distance Ultrasonic_Feedback(void);
void delay(uint32_t time_ms);
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
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of CameraUARTSem */
  osSemaphoreDef(CameraUARTSem);
  CameraUARTSemHandle = osSemaphoreCreate(osSemaphore(CameraUARTSem), 1);

  /* definition and creation of GyroReadySem */
  osSemaphoreDef(GyroReadySem);
  GyroReadySemHandle = osSemaphoreCreate(osSemaphore(GyroReadySem), 1);

  /* definition and creation of CriticalDistanceSem */
  osSemaphoreDef(CriticalDistanceSem);
  CriticalDistanceSemHandle = osSemaphoreCreate(osSemaphore(CriticalDistanceSem), 1);

  /* definition and creation of MileageSem */
  osSemaphoreDef(MileageSem);
  MileageSemHandle = osSemaphoreCreate(osSemaphore(MileageSem), 1);

  /* definition and creation of MileageNegSem */
  osSemaphoreDef(MileageNegSem);
  MileageNegSemHandle = osSemaphoreCreate(osSemaphore(MileageNegSem), 1);

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
  /* definition and creation of Stream */
  osThreadDef(Stream, StreamTask, osPriorityNormal, 0, 128);
  StreamHandle = osThreadCreate(osThread(Stream), NULL);

  /* definition and creation of GyroReceive */
  osThreadDef(GyroReceive, GyroReceiveTask, osPriorityNormal, 0, 128);
  GyroReceiveHandle = osThreadCreate(osThread(GyroReceive), NULL);

  /* definition and creation of DistanceCheck */
  osThreadDef(DistanceCheck, DistanceCheckTask, osPriorityNormal, 0, 128);
  DistanceCheckHandle = osThreadCreate(osThread(DistanceCheck), NULL);

  /* definition and creation of Mileage */
  osThreadDef(Mileage, MileageTask, osPriorityNormal, 0, 128);
  MileageHandle = osThreadCreate(osThread(Mileage), NULL);

  /* definition and creation of GoStraight */
  osThreadDef(GoStraight, GoStraightTask, osPriorityNormal, 0, 128);
  GoStraightHandle = osThreadCreate(osThread(GoStraight), NULL);

  /* definition and creation of LineSearch */
  osThreadDef(LineSearch, LineSearchTask, osPriorityNormal, 0, 128);
  LineSearchHandle = osThreadCreate(osThread(LineSearch), NULL);

  /* definition and creation of LineSearch2 */
  osThreadDef(LineSearch2, LineSearch2Task, osPriorityNormal, 0, 128);
  LineSearch2Handle = osThreadCreate(osThread(LineSearch2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 2;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1001-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 901-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 3;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 150;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LEDBlue_Pin|LEDGreen_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(rightPWMGND_GPIO_Port, rightPWMGND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(leftPWMGND_GPIO_Port, leftPWMGND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDBlue_Pin LEDGreen_Pin */
  GPIO_InitStruct.Pin = LEDBlue_Pin|LEDGreen_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : rightPWMGND_Pin */
  GPIO_InitStruct.Pin = rightPWMGND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(rightPWMGND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : leftPWMGND_Pin */
  GPIO_InitStruct.Pin = leftPWMGND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(leftPWMGND_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Car_Initial(void)
{
	taskENTER_CRITICAL();
	state=Initial;
	temp_state = Unknow;
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//�????????????????????????????????????????????????????????????????????????启左侧PWM
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//�????????????????????????????????????????????????????????????????????????启右侧PWM
	taskEXIT_CRITICAL();
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim2,500);
	HAL_TIM_Base_Start_IT(&htim2);
	//vTaskSuspend(UART_RTHandle);//Suspend UART R and T
	//vTaskSuspend(PIDCameraHandle);//Suspend PID module
}

void Car_Stop(void)
{
	taskENTER_CRITICAL();
	PWM_SET_LEFT(1);
	PWM_SET_RIGHT(1);
	taskEXIT_CRITICAL();
}

void delay(uint32_t time_ms)
{
	uint32_t PreviousWakeTime=osKernelSysTick();
	osDelayUntil(&PreviousWakeTime, time_ms);
}

float Angle_Diff(float target, float input)
{
	float Error;
	if(target >= 180)
		target=-360+target;
	else if(target <=-180)
		target=360+target;
	Error = target - input;
		if(Error >= 180)
			Error=Error-360;
		else if(Error <= -180)
			Error=Error+360;
	return Error;
}

//int Angle_Stable_Check(float Error, float Accept_Error, int wait_cycles, uint8_t* pFlag, int* pt)
//{
//#define Flag *Flag
//#define t *t
// 	 if(( (Error > -Accept_Error) && (Error < Accept_Error) ) && Flag == 0)
// 	 {
// 		 t++;
// 		if(t>2)
// 		{
// 			Flag = 1;
// 			t=0;
// 		}
// 	 }
// 	 if(Flag)
// 	 {
// 		if(t>wait_cycles)
// 		{
// 			Flag=0;
// 			t=0;
// 			return 0;
// 		}
// 		else if((Error > -Accept_Error) && (Error < Accept_Error))
// 		{
// 			t++;
// 		}
// 		else
// 		{
// 			Flag=0;
// 			t=0;
// 		}
// 	 }
//}

int PID_Turning(float increment_angle,float Accept_Error)//If we want to turn right, parameter is negative
{
			float PID_target=0;
			float PID_Error_Last=0;
			float initial_yaw=0;
			float PID_Output=0,PID_Input=0;;
			float Error = 0, Error_Total=0;
			float KP=7, KI=0.05, KD=0;
			int t=0;
			uint8_t Flag=0; //Indicate that if verifying process begin.
			Car_Stop();
			//delay(1500);
			for(int i=0;i<10;i++)			//Get average initial direction
			{
//				if(gyro_ready_flag)
//				{
					//gyro_ready_flag=0;
					osSemaphoreWait(GyroReadySemHandle, osWaitForever);
					//ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
					initial_yaw+=angle.z;
//				}
//				else
//					continue;
			}
			initial_yaw=initial_yaw/10;
			PID_target=initial_yaw + increment_angle;
			if(PID_target > 180)
				PID_target=-360+PID_target;
			if(PID_target <-180)
				PID_target=360+PID_target;


		  for(;;)
		  {
//			  if(state == Idle)
//			  		  {
//			  			  return 1;
//			  		  }
//			  	  if(gyro_ready_flag==0)
//			  		  continue;
//			  	  gyro_ready_flag=0;
			  	 osSemaphoreWait(GyroReadySemHandle, osWaitForever);
			  	 //ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));

			  	 PID_Input = angle.z;
			  	 Error=Angle_Diff(PID_target, PID_Input);
			  	 if(( (Error > -Accept_Error) && (Error < Accept_Error) ) && Flag == 0)
			  	 {
			  		 t++;
			  		if(t>2)
			  		{
			  			Flag = 1;
			  			t=0;
			  		}
			  	 }
			  	 if(Flag)
			  	 {
			  		if(t>Angle_stable_cycles)
			  		{
			  			Flag=0;
			  			t=0;
			  			return 0;
			  		}
			  		else if((Error > -Accept_Error) && (Error < Accept_Error))
			  		{
			  			t++;
			  		}
			  		else
			  		{
			  			Flag=0;
			  			t=0;
			  		}
			  	 }
			  	 Error_Total=Error_Total+KI*Error;
			     PID_Output = KP * Error  +
			 				  KD * (Error - PID_Error_Last ) +
							  Error_Total;
			     PID_Error_Last = Error;
			     if(PID_Output < 0)
			     {
			    	 PID_Output-=PWM_Lowest;
			    	 if(-PID_Output > PWM_Higest)
			    	 	PID_Output=-PWM_Higest;
			     }

			     else if(PID_Output > 0)
			     {
			    	 PID_Output+=PWM_Lowest;
			    	 if(-PID_Output > PWM_Higest)
			    	 	PID_Output=-PWM_Higest;
			     }
			     else
			    	PID_Output=0;
//			     if(PID_Output >= 0)
//			     {
//
//			    	 if(PID_Output > PWM_Higest)
//			    		 PID_Output=PWM_Higest;
//			    	 if(PID_Output < PWM_Lowest)
//			    		 PID_Output=PWM_Lowest;
//			    	 //taskENTER_CRITICAL();
//
//			    	 //taskEXIT_CRITICAL();
//			     }
//			     if(PID_Output <0)
//			     {
//			    	 if(-PID_Output > PWM_Higest)
//			    		 PID_Output=-PWM_Higest;
//			    	 if(-PID_Output < PWM_Lowest)
//			    	 	 PID_Output=-PWM_Lowest;
			    	 taskENTER_CRITICAL();
			    	 PWM_SET_RIGHT ((int32_t) PID_Output);
			    	 PWM_SET_LEFT((int32_t)(-PID_Output));
			    	 taskEXIT_CRITICAL();
			     }
			     delay(2);
}

void PID_Straight(float speed)
{
					float PID_target=0;
					float PID_Error_Last=0;
					float initial_yaw=0;
					float PID_Output=0,PID_Input=0;;
					float Error = 0, Error_Total=0,Error_Total_Total=0;
					float KP=10, KI=3, KD=5, KI2=0.013;
					int32_t pwm_right=0;
					int32_t pwm_left=0;
					//uint8_t Flag=0; //Indicate that if verifying process begin.
					Car_Stop();
					if (PID_Straight_Reset_Flag)
						return;
					osSemaphoreWait(GyroReadySemHandle, osWaitForever);
					for(int i=0;i<10;i++)			//Get average initial direction
					{
							osSemaphoreWait(GyroReadySemHandle, osWaitForever);
							initial_yaw+=angle.z;
							delay(10);
					}
					initial_yaw=initial_yaw/10;
					PID_target=initial_yaw;
				  for(;;)
				  {
					  	 if (PID_Straight_Reset_Flag)
					  		 return;
		  	  	  	  	 //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green
					  	 osSemaphoreWait(GyroReadySemHandle, osWaitForever);
					  	 PID_Input = angle.z;
					  	 Error=Angle_Diff(PID_target, PID_Input);
					  	 Error_Total=Error_Total+KI*Error;
					     Error_Total_Total= Error_Total_Total+KI2*Error_Total;
					     PID_Output = KP * Error  +
					 				  KD * (Error - PID_Error_Last ) +
									  Error_Total;
					     PID_Error_Last = Error;

					     pwm_right=	speed+	(int32_t) 	PID_Output;
					     pwm_left=	speed-	(int32_t)  	PID_Output;
					     pwm_right = pwm_right<PWM_Lowest ? PWM_Lowest : pwm_right;
					     pwm_right = pwm_right>PWM_Higest ? PWM_Higest : pwm_right;
					     pwm_left = pwm_left<PWM_Lowest ? PWM_Lowest : pwm_left;
					     pwm_left = pwm_left>PWM_Higest ? PWM_Higest : pwm_left;
					     if (PID_Straight_Reset_Flag)
					     	return;
					     taskENTER_CRITICAL();
					     PWM_SET_RIGHT (pwm_right);
					     PWM_SET_LEFT(pwm_left);
					     taskEXIT_CRITICAL();
					     }
}

float PID_Line_Follow(float Accept_Error)
{
#define MAX_TIME 250
			volatile uint16_t PID_Target=0;
		    volatile float Kp = 3, Ki = 0, Kd =0;     // PID系数
			float PID_Error_Last=0;
			float PID_Output=0;                    // PWM输出占空
			float Error = 0, Error_Total=0,First_Error=0;
			int32_t PID_Input=0;
			HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
			//delay(100);
			osSemaphoreWait(CameraUARTSemHandle, osWaitForever);
						  	 for(int i = 0;i<3;i++)
						  	 {
						  		 //osSemaphoreRelease(GyroReadySemHandle);
						  		osSemaphoreWait(CameraUARTSemHandle, osWaitForever);
						  		 PID_Input = (Camera_Data & (0x07FF))-1000;
						  		 //osSemaphoreWait(CameraUARTSemHandle, osWaitForever);
						  	 }
						  	 PID_Input/=3;
			First_Error= PID_Target - PID_Input;
			Error=PID_Target - PID_Input;

			  	 Error = PID_Target - PID_Input;		  // 偏差 = 目标 - 实际
			  	 Error_Total=Error_Total+Ki*Error;
			  	 PID_Output = Kp * Error  +
			  				  Kd * (Error - PID_Error_Last ) +
			  				  Error_Total;
			  	 PID_Error_Last = Error;
			     if(PID_Output > MAX_TIME) 			PID_Output =	MAX_TIME;	    // 限幅
			     else if(PID_Output <-(MAX_TIME)) 	PID_Output = 	-MAX_TIME;

			     if(PID_Output>0)
			     {
			    	 taskENTER_CRITICAL();
			    	 PWM_SET_RIGHT (PWM_Lowest+700);
			    	 PWM_SET_LEFT  (PWM_Lowest-700);
			    	 taskEXIT_CRITICAL();
			     }
			     else
			     {
			    	 taskENTER_CRITICAL();
			    	 PWM_SET_RIGHT (-PWM_Lowest-700);
			    	 PWM_SET_LEFT  (PWM_Lowest+700);
			    	 taskEXIT_CRITICAL();
			     }
			     PID_Output=PID_Output>0?PID_Output:-PID_Output;
			     delay((uint32_t) PID_Output);
			     return First_Error;
}

Distance Ultrasonic_Feedback(void)
{
	uint8_t info=0xA0;
	uint8_t Rx_Buf[3]={0,0,0};
	uint32_t Data=0x00000000;
	Distance distance={0.0,0.0};
	taskENTER_CRITICAL();
	HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,0xFFFF);
	taskEXIT_CRITICAL();
	delay(200);
	taskENTER_CRITICAL();
	HAL_UART_Receive(&huart5,(uint8_t*) &Rx_Buf,3,1);
	taskEXIT_CRITICAL();
	Data=Data | (((uint32_t) (Rx_Buf[0]))<<16);
	Data=Data | (((uint32_t) (Rx_Buf[1]))<<8);
	Data=Data |((uint32_t) (Rx_Buf[2]));
	//HAL_UART_Transmit(&huart1, (uint8_t *) &Data, 4, 0xFFFF);
	distance.front=Data/1000;
	return distance;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
  	if (huart->Instance==USART2){
  		Camera_Data=0x0000;
  		//osSemaphoreWait(CameraUARTSemHandle, 0);
  		Camera_Data=Camera_Data | (((uint16_t) (Rx_Buf[0]))<<8);
  		Camera_Data=Camera_Data|((uint16_t) (Rx_Buf[1]));
  		osSemaphoreRelease(CameraUARTSemHandle);
  		//Data=Data & (0x07F0);
  		HAL_UART_Transmit(&huart1, (uint8_t*) &Camera_Data,2,10);
  		//HAL_UART_AbortReceive_IT(&huart1);
  		Rx_Buf[0]=0;
  		Rx_Buf[1]=0;
  		//camera_ready_flag=1;
  		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
  		//if(camera_recieve_IT_flag)
  		HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
  	}
  }

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
}

uint8_t State_Transition(State* current_state)
{
	State next_state = Unknow;
	switch(state)
	{
		case Initial:
					next_state = Line_Search;
					break;
		case Line_Search:
					if(distance_flag==0)
						next_state = Line_Search;
					else
						next_state= TurnRight;
					break;
		case Line_Search2:
						next_state= TurnRight;
					break;
		case Go_Line_Follow:
						next_state = Line_Search;
				  	break;
		case TurnRight:
					next_state = Go_Mile_1;
					break;
		case TurnRight2:
				 	next_state = Line_Search2;
					break;
		case Go_to_Bridge:
					next_state= Cross_bridge;
					break;
		case Cross_bridge:
					next_state= Go_Mile_2_Until_Barrier;
					break;
		case GoStraight_Until_Barrier:
					//osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
					if(distance_flag==0)
						next_state = GoStraight_Until_Barrier;
					else
						next_state = TurnRight;

					break;
		case Go_Mile_1:
//					if(*current_state == Mile_Adjust)
//						next_state = TurnRight;
//					else
//						{
//						temp_state = *current_state;
//						next_state = Mile_Adjust;
//						}
					next_state=Go_to_Bridge;
					break;
		case Go_Mile_2_Until_Barrier:
					next_state=TurnRight2;
					break;
		case Mile_Adjust:
					switch (temp_state)
					{
					case Go_Mile_1:
						next_state = Line_Search;
						break;
					default:
						next_state = Initial;
						break;
					}
					//temp_state = Mile_Adjust;
					break;
		default:
					next_state = Initial;
					break;
	}
	if (next_state == *current_state)
		return 1;
	else
	{

		*current_state=next_state;
		return 0;
	}
}

void PWM_SET_LEFT(int32_t duty)
{
	if ( duty < 0 )
		{
		if (duty <= -2000)
			duty = 1;
		else
			duty = 2000 + duty;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
		}
	else
		{
			if (duty == 0)
				duty = 1;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
		}
	if (duty > 2000)
		duty = 2000;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,duty);
}

void PWM_SET_RIGHT(int32_t duty)
{
	if ( duty < 0 )
		{
		if (duty <= -2000)
			duty = 1;
		else
			duty = 2000 + duty;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		}
	else
		{
			if (duty == 0)
				duty = 1;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		}
	if (duty > 2000)
		duty = 2000;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StreamTask */
/**
  * @brief  Function implementing the Stream thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StreamTask */
void StreamTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t Same_State_Flag=0;
	uint32_t pulse_incremnet=0;
	Car_Initial();
	delay(3000);
  /* Infinite loop */
  for(;;)
  {

	  delay(50);
	  //delay(10);
	  //PreviousWakeTime = osKernelSysTick()
	  //osDelayUntil(&PreviousWakeTime = osKernelSysTick(), 500);
	  //HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);

	  Same_State_Flag = State_Transition(&state);
	  if(Same_State_Flag)
		  continue;
	  switch(state)
	  {
	  case Initial:
		  	  	  	  	  state= Idle;
		  	  	  	  	  delay(500);
		  	  	  	      state= Initial;
		  	  	  	  	  Car_Initial();
		  	  	  	  	  break;
	  case Line_Search:
		  		  	  	  delay(500);
		  		  	  	  critical_distance.front=150;
		  		  	  	  camera_recieve_IT_flag=1;
		  		  	  	  HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
		  		  	  	  vTaskResume(MileageHandle);
		  		  	  	  vTaskResume(LineSearchHandle);
		  		  	  	  vTaskResume(DistanceCheckHandle);
		  		  	  	  delay(1500);
		  		  	  	  vTaskResume(LineSearchHandle);
		  		  	  	  delay(2000);
		  		  	  	  //delay(300000);
		  		  	  	  osSemaphoreWait(CriticalDistanceSemHandle, 0);
		  		  	  	  osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  		  	  	  vTaskDelete(LineSearchHandle);
		  	  	  	  	  camera_recieve_IT_flag=0;
	  	  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  	  	  	  	  break;
	  case Line_Search2:
		  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  	  	  	  	  vTaskSuspend(GyroReceiveHandle);
		  	  	  	  	  vTaskSuspend(GoStraightHandle);
		  	  	  	  	  vTaskResume(MileageHandle);
		  		  	  	  delay(500);
		  		  	  	  critical_distance.front=350;
		  		  	  	  vTaskResume(LineSearch2Handle);
		  		  	  	  delay(50000);
		  	  	  	  	  break;
	  case Go_Line_Follow:
		  	  	  	  	  break;
	  case Go_to_Bridge:
		  	  	  	  	  delay(1000);
						  taskENTER_CRITICAL();
						  PWM_SET_RIGHT (PWM_Lowest+100);
						  PWM_SET_LEFT(PWM_Lowest+100);
						  taskEXIT_CRITICAL();
						  delay(5000);
		  	  	  	  	  break;
	  case Cross_bridge:
		  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  //pulse_incremnet=2400;//室外
						  //pulse_incremnet=600; //小正方形

		  	  	  	  	  pulse_incremnet=5400;//上下�??
						  critical_pulses=0;
						  vTaskResume(MileageHandle);
						  delay(100);
						  osSemaphoreWait(MileageSemHandle, osWaitForever);
						  critical_pulses=pulse_incremnet+number_of_pulses;
						  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  PID_Straight_Reset_Flag=1;
		  	  	  	  	  go_straight_speed=PWM_Mid+500;
		  	  	  	  	  vTaskResume(GoStraightHandle);
		  	  	  	  	  delay(200);
		  	  	  	  	  PID_Straight_Reset_Flag=0;
		  	  	  	      osSemaphoreWait(MileageSemHandle, osWaitForever);
		  	  	  	      PID_Straight_Reset_Flag=1;
		  	  	  	      vTaskSuspend(GoStraightHandle);
		  	  	  		  Car_Stop();
		  	  	  		  gyro_reset_flag=1;
		  	  	  		  //vTaskSuspend(MileageHandle);
		  	  	  	  	  break;
	  case TurnRight:
		  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(-90,5);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight2:
		  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  	  	  	  	  vTaskSuspend(GoStraightHandle);
		  	  	  	  	  vTaskSuspend(MileageHandle);
						  Car_Stop();
						  delay(50);
						  distance_flag=0;
						  gyro_reset_flag=0;
						  vTaskResume(GyroReceiveHandle);
						  delay(500);
						  PID_Turning(-90,5);
						  gyro_reset_flag=1;
						  Car_Stop();
						  break;
	  case GoStraight_Until_Barrier:
		  	  	  	  	  //state= Idle;
		  	  	  	  	  vTaskSuspend(GyroReceiveHandle);
		  	  	  	  	  camera_recieve_IT_flag=0;
		  	  	  	  	  delay(500);
		  	  	  	  	  //state= GoStraight;
		  	  	  	  	  critical_distance.front=350;
		  	  	  	  	  vTaskResume(DistanceCheckHandle);
		  	  	  	  	  PWM_SET_LEFT(PWM_Mid);
		  	  	  	  	  PWM_SET_RIGHT(PWM_Mid);
		  	  	  	  	  osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  	  	  	  	  break;
	  case Go_Mile_1:
		  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=2400;//室外
						  //pulse_incremnet=600; //小正方形
						  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
						  critical_pulses=0;
						  vTaskResume(MileageHandle);
						  delay(500);
						  osSemaphoreWait(MileageSemHandle, osWaitForever);
						  critical_pulses=pulse_incremnet+number_of_pulses;
		  	  	  	  	  PID_Straight_Reset_Flag=1;
		  	  	  	  	  go_straight_speed=PWM_Mid;
		  	  	  	  	  vTaskResume(GoStraightHandle);
		  	  	  	  	  delay(200);
		  	  	  	  	  PID_Straight_Reset_Flag=0;
		  	  	  	      osSemaphoreWait(MileageSemHandle, osWaitForever);
		  	  	  	      PID_Straight_Reset_Flag=1;
		  	  	  	      vTaskSuspend(GoStraightHandle);
		  	  	  		  Car_Stop();
		  	  	  		  gyro_reset_flag=1;
		  	  	  		  //vTaskSuspend(MileageHandle);
		  	  	  	  	  break;
	  case Go_Mile_2_Until_Barrier:
		  	  	  	  	  critical_distance.front=250;
		  	  	  	  	  vTaskResume(DistanceCheckHandle);
						  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Straight_Reset_Flag=1;
		  	  	  	  	  go_straight_speed=PWM_Mid;
		  	  	  	  	  vTaskResume(GoStraightHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Straight_Reset_Flag=0;
		  	  	  	  	  osSemaphoreWait(CriticalDistanceSemHandle, 0);
		  	  	  		  osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  	  	  		  //osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  	  	  	      PID_Straight_Reset_Flag=1;
		  	  	  	      vTaskSuspend(GoStraightHandle);
		  	  	  		  Car_Stop();
		  	  	  		  gyro_reset_flag=1;
		  	  	  		  //vTaskSuspend(MileageHandle);
		  	  	  	  	  break;
	  case Mile_Adjust:
		  	  	  	  	  vTaskResume(MileageHandle);
		  	  	  	  	  PWM_SET_LEFT(-PWM_Lowest-80);
		  	  	  		  PWM_SET_RIGHT(-PWM_Lowest-80);
		  	  	  		  osSemaphoreRelease(MileageNegSemHandle);
		  	  	  		  osSemaphoreWait(MileageNegSemHandle, osWaitForever);
		  	  	  		  osSemaphoreWait(MileageNegSemHandle, osWaitForever);
		  	  	  		  Car_Stop();
		  	  	  		  vTaskSuspend(MileageHandle);
		  	  	  	  	  break;
	  case Idle:
		  	  	  	  	  Car_Stop();
		  	  	  	  	  break;
	  default :
		  	  	  	  	  Car_Initial();
		  	  	  	  	  break;
	  }

  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GyroReceiveTask */
/**
* @brief Function implementing the GyroReceive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GyroReceiveTask */
void GyroReceiveTask(void const * argument)
{
  /* USER CODE BEGIN GyroReceiveTask */
	vTaskSuspend(GyroReceiveHandle);
  /* Infinite loop */
  for(;;)
  {

	  delay(100);
	  uint8_t AxH=0, AxL=0;
	  int16_t Ax=0;

	  uint8_t AyH=0,AyL=0;
	  int16_t Ay=0;

	  uint8_t YawH=0,YawL=0;
	  int16_t Yaw=0;

	  uint8_t sum=0;
	  int i=0;
	  int h=0;
	  uint8_t GyroData[21]={0};
	  taskENTER_CRITICAL();
	  HAL_UART_Receive(&huart3, (uint8_t *) &GyroData, sizeof(GyroData), 50);
	  taskEXIT_CRITICAL();
	  while(h<14)
	  {
		  if(GyroData[h]==0x55)
			  break;
		  h++;
	  }
	  if(GyroData[h]!=0x55)
		  continue;
	  if(GyroData[h+1]!=0x53)
		  continue;
	  i=0;
	  sum=0;
	  while(i<10)
	  {
		  sum=sum+GyroData[h+i];
		  i++;
	  }
	  	  if (sum!=GyroData[h+10])
	  		  continue;
	  AxL=GyroData[h+2];
	  AxH=GyroData[h+3];

	  AyL=GyroData[h+4];
	  AyH=GyroData[h+5];

	  YawL=GyroData[h+6];
	  YawH=GyroData[h+7];

	  Ax=((((int16_t) AxH)<<8) | AxL);
	  Ay=((((int16_t) AyH)<<8) | AyL);
	  Yaw=((((int16_t) YawH)<<8) | YawL);
	  //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
	  //taskENTER_CRITICAL();
	  //HAL_UART_Transmit(&huart1, (uint8_t *) &Yaw, sizeof(Yaw), 0xFFFF);
	  //taskEXIT_CRITICAL();
	  if(gyro_reset_flag)
	  {
		  vTaskSuspend(GyroReceiveHandle);
		  continue;
	  }

	  angle.x=(((float)Ax) / 32768.0 * 180.0);
	  angle.y=(((float)Ay) / 32768.0 * 180.0);
	  angle.z=(((float)Yaw) / 32768.0 * 180.0);
	  osSemaphoreRelease(GyroReadySemHandle);
  }
  /* USER CODE END GyroReceiveTask */
}

/* USER CODE BEGIN Header_DistanceCheckTask */
/**
* @brief Function implementing the DistanceCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DistanceCheckTask */
void DistanceCheckTask(void const * argument)
{
  /* USER CODE BEGIN DistanceCheckTask */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
	vTaskSuspend(DistanceCheckHandle);
	delay(1000);
  /* Infinite loop */
  for(;;)
  {
	  Distance distance={0.0,0.0};
	  Distance temp=Ultrasonic_Feedback();
	  //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
	  for(int i=0;i<1;i++)
	  {
		  distance.front+=temp.front;
	  }
	  distance.front/=1;
	  if(distance.front < critical_distance.front)
	  {
		  distance_flag=1;
		  osSemaphoreRelease(CriticalDistanceSemHandle);
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
	  }
	  else
	  {
		  distance_flag=0;
		  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
	  }
  }
  /* USER CODE END DistanceCheckTask */
}

/* USER CODE BEGIN Header_MileageTask */
/**
* @brief Function implementing the Mileage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MileageTask */
void MileageTask(void const * argument)
{
  /* USER CODE BEGIN MileageTask */
	//uint8_t mileage_counter;
	vTaskSuspend(MileageHandle);
  /* Infinite loop */
  for(;;)
  {
	  taskENTER_CRITICAL();
	  //mileage_counter=__HAL_TIM_GET_COUNTER(&htim2);
	  //number_of_pulses=1000*(mileage_IT_number-1)+mileage_counter;
	  number_of_pulses=5000*mileage_IT_number+__HAL_TIM_GET_COUNTER(&htim2);
	  taskEXIT_CRITICAL();
	  //HAL_UART_Transmit(&huart1, &number_of_pulses, sizeof(number_of_pulses), 1000);
	  if (number_of_pulses>critical_pulses)
		  osSemaphoreRelease(MileageSemHandle);
	  else
		  osSemaphoreRelease(MileageNegSemHandle);
	  delay(50);
	  //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
	  //HAL_Delay(1000);
	  //osDelay(1);
  }
  /* USER CODE END MileageTask */
}

/* USER CODE BEGIN Header_GoStraightTask */
/**
* @brief Function implementing the GoStraight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GoStraightTask */
void GoStraightTask(void const * argument)
{
  /* USER CODE BEGIN GoStraightTask */
	vTaskSuspend(GoStraightHandle);
  /* Infinite loop */
  for(;;)
  {
	if (PID_Straight_Reset_Flag)
		continue;
	PID_Straight((float)go_straight_speed);
    delay(100);
  }
  /* USER CODE END GoStraightTask */
}

/* USER CODE BEGIN Header_LineSearchTask */
/**
* @brief Function implementing the LineSearch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LineSearchTask */
void LineSearchTask(void const * argument)
{
  /* USER CODE BEGIN LineSearchTask */
	int32_t pulse_increment=1200;
	float Error=0;
	float Error_total=0;
	float pulse_increment_float=0;
	float Kp=9;//,Ki=0,Kd=0;
	vTaskSuspend(LineSearchHandle);
	vTaskResume(MileageHandle);
	//HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
  /* Infinite loop */
  for(;;)
  {
	  						  critical_pulses=0;
	  						  vTaskResume(MileageHandle);
	  						  critical_pulses=pulse_increment+number_of_pulses;
	  						  osSemaphoreRelease(MileageSemHandle);
	  						  osSemaphoreWait(MileageSemHandle, 0);
	  						  taskENTER_CRITICAL();
	  					      PWM_SET_LEFT(PWM_Mid);
	  						  PWM_SET_RIGHT(PWM_Mid);
	  						  taskEXIT_CRITICAL();
	  		  	  	  	      osSemaphoreWait(MileageSemHandle, osWaitForever);
	  		  	  	  		  Car_Stop();
	  		  	  	  		  vTaskSuspend(MileageHandle);
	  		  	  	  		  Error=PID_Line_Follow(10);
	  		  	  	  		  Error_total+=Error;

	  		  	  	  		  pulse_increment_float=300-((int32_t) (Kp*(Error>0?Error:-Error)));
	  		  	  	  		  pulse_increment= pulse_increment_float>0?(int)pulse_increment_float:50;

  }
  /* USER CODE END LineSearchTask */
}

/* USER CODE BEGIN Header_LineSearch2Task */
/**
* @brief Function implementing the LineSearch2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LineSearch2Task */
void LineSearch2Task(void const * argument)
{
  /* USER CODE BEGIN LineSearch2Task */
		float Error=0;
		float Error_total=0;
		float pulse_increment_float=0;
		float Kp=9;//,Ki=0,Kd=0;
		vTaskSuspend(LineSearch2Handle);
		int32_t pulse_increment=300;
		vTaskResume(MileageHandle);
		//HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
	  /* Infinite loop */
	  for(;;)
	  {
		  						  critical_pulses=0;
		  						  vTaskResume(MileageHandle);
		  						  critical_pulses=pulse_increment+number_of_pulses;
		  						  osSemaphoreRelease(MileageSemHandle);
		  						  osSemaphoreWait(MileageSemHandle, 0);
		  						  taskENTER_CRITICAL();
		  					      PWM_SET_LEFT(PWM_Mid);
		  						  PWM_SET_RIGHT(PWM_Mid);
		  						  taskEXIT_CRITICAL();
		  		  	  	  	      osSemaphoreWait(MileageSemHandle, osWaitForever);
		  		  	  	  		  Car_Stop();
		  		  	  	  		  vTaskSuspend(MileageHandle);
		  		  	  	  		  Error=PID_Line_Follow(10);
		  		  	  	  		  Error_total+=Error;

		  		  	  	  		  pulse_increment_float=300-((int32_t) (Kp*(Error>0?Error:-Error)));
		  		  	  	  		  pulse_increment= pulse_increment_float>0?(int)pulse_increment_float:50;

	  }
  /* USER CODE END LineSearch2Task */
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
  else if(htim->Instance==TIM2)
  	{
	   if(__HAL_TIM_GET_COUNTER(&htim2)>3000)
	    	mileage_IT_number--;
	   else
	    	mileage_IT_number++;
	   //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
  	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
