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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum State {Initial=1,
					Line_Search,
					TurnRight,
					TurnRight1_1,
					TurnRight1_2,
					TurnRight1_3,
					TurnRight2_1,
					TurnRight2_3,
					TurnRight3_1,
					TurnRight3_3,
					TurnRight4,
					TurnRight5,
					TurnRight6,
					TurnRight7,
					TurnRight8,
					GoStraight_Until_Barrier,
					Go_Mile,
					Go_Mile_1,
					Go_Mile_2_1,
					Go_Mile_2_2,
					Go_Mile_2_3,
					Go_Mile_3_1,
					Go_Mile_3_3,
					Go_Mile_4,
					Go_Mile_5,
					Go_Mile_6,
					Go_Mile_6_7,
					Go_Mile_7,
					Go_Mile_8_Until_Apriltag,
					Go_Mile_9,
					Go_Mile_10,
					Mile_Adjust,
					Apriltag_Adjust1,Apriltag_Adjust2,Apriltag_Check,Apriltag_Check2,
					Feeding,
					Communication,
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
#define PWM_Mid 800  //无反馈时电机工作占空
#define PWM_Lowest 500
#define PWM_Higest 1400 //for our motor, this value should less than 1300
#define Angle_stable_cycles 3
#define PWM_Bias 0.9331
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

osThreadId StreamHandle;
osThreadId PIDCameraHandle;
osThreadId GyroReceiveHandle;
osThreadId DistanceCheckHandle;
osThreadId MileageHandle;
osThreadId GoStraightHandle;
osThreadId ColorcheckHandle;
osThreadId WirelessHandle;
osSemaphoreId CameraUARTSemHandle;
osSemaphoreId GyroReadySemHandle;
osSemaphoreId CriticalDistanceSemHandle;
osSemaphoreId MileageSemHandle;
osSemaphoreId MileageNegSemHandle;
osSemaphoreId gomile6SemHandle;
osSemaphoreId UltraFrontSemHandle;
osSemaphoreId ApriltagSemHandle;
/* USER CODE BEGIN PV */
State state;
State temp_state;
volatile uint8_t Rx_Buf[2]={0,0};
//uint8_t Rx_Buf_Sonic[3]={0,0,0};
//volatile uint8_t OpenmvData[2]={0,0};
volatile uint16_t Camera_Data=0x0000;
volatile uint8_t Rx_Buf_Right[3]={0,0,0};
volatile uint8_t Rx_Buf_Front[3]={0,0,0};
Angle angle={0.0,0.0,0.0};
//Distance distance={0.0,0.0};
Distance critical_distance={0.0,0.0};
Distance current_distance={0,0};
Distance right_distance={0.0,0.0};
Distance front_distance={0.0,0.0};
int distance_flag=0;
int camera_recieve_IT_flag=0;
int gyro_reset_flag=0;
int go_straight_speed=PWM_Mid;
//Encoder PV
int32_t mileage_IT_number=-1;
//uint8_t direction;
int32_t number_of_pulses=0;
int32_t critical_pulses=0;
int PID_Straight_Reset_Flag=1;
int info=0xA0;
uint16_t blue=0, pink=0, yellow=0;
uint8_t finalcolor;
//Encoder PV END

//PID PV
volatile uint16_t PID_Target=0;
volatile float Kp = 6, Ki = 0, Kd =0;     // PID系数，这里只用到PI控制�????????????????????????????????????????????????????????????????????
//PID PV END

//HC_12_PV
RTC_DateTypeDef datenow;
RTC_TimeTypeDef timenow;

uint8_t Tx_ATCHANNEL[] = "AT+C099\r\n";
uint8_t Tx_team_name[] = "team32: che che\r\n";
uint8_t Tx_str01[] = "2429410Z Yiyao Zhong\r\n";
uint8_t Tx_str02[] = "2429513H Haiyang Hao\r\n";
uint8_t Tx_str03[] = "2429453Y Chunlei Yu\r\n";
uint8_t Tx_str04[] = "2429458Y Xiaoyu Yi\r\n";
uint8_t Tx_str05[] = "2429488L Yunfei Ling\r\n";
uint8_t Tx_str06[] = "2429494B Jinsong Bai\r\n";
uint8_t Tx_str07[] = "2429459L Xiaoyuan Li\r\n";
uint8_t Tx_str08[] = "2429491L Yuhan Li\r\n";
uint8_t Tx_str09[] = "2429567W Aodong Wei\r\n";
uint8_t Tx_str10[] = "2429264Y Jingxuan Yang\r\n";
//HC_12_PV_END

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
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
static void MX_USART6_UART_Init(void);
void StreamTask(void const * argument);
void PIDCameraTask(void const * argument);
void GyroReceiveTask(void const * argument);
void DistanceCheckTask(void const * argument);
void MileageTask(void const * argument);
void GoStraightTask(void const * argument);
void ColorcheckTask(void const * argument);
void WirelessTask(void const * argument);

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
  MX_UART4_Init();
  MX_RTC_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */


  //HAL_UART_Receive_IT(&huart5,(uint8_t*) &Rx_Buf_Sonic,3);
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

  /* definition and creation of gomile6Sem */
  osSemaphoreDef(gomile6Sem);
  gomile6SemHandle = osSemaphoreCreate(osSemaphore(gomile6Sem), 1);

  /* definition and creation of UltraFrontSem */
  osSemaphoreDef(UltraFrontSem);
  UltraFrontSemHandle = osSemaphoreCreate(osSemaphore(UltraFrontSem), 1);

  /* definition and creation of ApriltagSem */
  osSemaphoreDef(ApriltagSem);
  ApriltagSemHandle = osSemaphoreCreate(osSemaphore(ApriltagSem), 1);

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

  /* definition and creation of PIDCamera */
  osThreadDef(PIDCamera, PIDCameraTask, osPriorityNormal, 0, 128);
  PIDCameraHandle = osThreadCreate(osThread(PIDCamera), NULL);

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

  /* definition and creation of Colorcheck */
  osThreadDef(Colorcheck, ColorcheckTask, osPriorityNormal, 0, 128);
  ColorcheckHandle = osThreadCreate(osThread(Colorcheck), NULL);

  /* definition and creation of Wireless */
  osThreadDef(Wireless, WirelessTask, osPriorityNormal, 0, 128);
  WirelessHandle = osThreadCreate(osThread(Wireless), NULL);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  sTime.Seconds = 0x0;
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
  sConfig.IC2Filter = 1;
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
  htim3.Init.Prescaler = 601-1;
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
  htim4.Init.Prescaler = 601-1;
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4|LEDBlue_Pin|LEDGreen_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(rightPWMGND_GPIO_Port, rightPWMGND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(leftPWMGND_GPIO_Port, leftPWMGND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

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
	HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//�?????????????????????????????????????????????????????????????????????????启左侧PWM
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//�?????????????????????????????????????????????????????????????????????????启右侧PWM
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
	float KP=13, KI=2, KD=0;
	//15 2 0
	int t=0;
	float pwm_left=0,pwm_right=0;
	uint8_t Flag=0; //Indicate that if verifying process begin.
	Car_Stop();
	//delay(1500);
	for(int i=0;i<10;i++)			//Get average initial direction
	{
			osSemaphoreWait(GyroReadySemHandle, osWaitForever);
			initial_yaw+=angle.z;
	}
	initial_yaw=initial_yaw/10;
	PID_target=initial_yaw + increment_angle;
	if(PID_target > 180)
		PID_target=-360+PID_target;
	if(PID_target <-180)
		PID_target=360+PID_target;
  for(;;)
  {
	  	 osSemaphoreWait(GyroReadySemHandle, osWaitForever);
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
	     pwm_right =   PID_Output;
	     pwm_left  = - PID_Output;
	     pwm_right += pwm_right>0 ?PWM_Lowest:-PWM_Lowest;
	     pwm_left  += pwm_left>0  ?PWM_Lowest:-PWM_Lowest;
	     pwm_right =  pwm_right>= PWM_Higest?PWM_Higest:pwm_right;
	     pwm_right =  pwm_right<= -PWM_Higest?-PWM_Higest:pwm_right;
	     pwm_left  =  pwm_left >= PWM_Higest?PWM_Higest:pwm_left;
	     pwm_left  =  pwm_left <= -PWM_Higest?-PWM_Higest:pwm_left;// 限幅
	    	 taskENTER_CRITICAL();
	    	 PWM_SET_RIGHT ((int32_t)   pwm_right);
	    	 PWM_SET_LEFT  ((int32_t)   pwm_left );
	    	 taskEXIT_CRITICAL();
  }

}
void PID_Straight(float speed)
{
					float PID_target=0;
					float PID_Error_Last=0;
					float initial_yaw=0;
					float PID_Output=0,PID_Input=0;
					float Error = 0, Error_Total=0,Error_Total_Total=0;
					float KP=15, KI=6, KD=10, KI2=0.013;
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

Distance Ultrasonic_Feedback(void)
{
	uint8_t info=0xA0;
	uint8_t Rx_Buf[3]={0,0,0};
	uint32_t Data=0x00000000;
	Distance distance={0.0,0.0};
	taskENTER_CRITICAL();
	HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);
	delay(200);
	HAL_UART_Receive(&huart5,(uint8_t*) &Rx_Buf,3,1000);
	taskEXIT_CRITICAL();
	Data=Data | (((uint32_t) (Rx_Buf[0]))<<16);
	Data=Data | (((uint32_t) (Rx_Buf[1]))<<8);
	Data=Data |((uint32_t) (Rx_Buf[2]));
	//HAL_UART_Transmit(&huart1, (uint8_t *) &Data, 4, 0xFFFF);
	distance.front=Data/1000;
	return distance;
}

void Ultrasonic_Feedback_front(void)
{

	HAL_UART_Receive_IT(&huart5,(uint8_t*) &Rx_Buf_Front,3);
	HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,0xFFFF);

}

float Ultrasonic_Feedback_front_gomile6(void)
{
	uint8_t info=0xA0;
	uint8_t Rx_Buf[3]={0,0,0};
	uint32_t Data=0x00000000;
	Distance distance={0.0,0.0};
	float diatance_temp=0;


		HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);
	//delay(200);
	    HAL_UART_Receive(&huart5,(uint8_t*) &Rx_Buf,3,1000);
	    Data=Data | (((uint32_t) (Rx_Buf[0]))<<16);
	    Data=Data | (((uint32_t) (Rx_Buf[1]))<<8);
	    Data=Data |((uint32_t) (Rx_Buf[2]));
	//HAL_UART_Transmit(&huart1, (uint8_t *) &Data, 4, 0xFFFF);
	    Rx_Buf[0]=0;
	    Rx_Buf[1]=0;
	    Rx_Buf[2]=0;
	    distance.front=Data/1000;
	    diatance_temp=distance.front;

	    return diatance_temp;


}

float Ultrasonic_Feedback_right(void)
{
    float diatance_temp=0;

	HAL_UART_Receive_IT(&huart4,(uint8_t*) &Rx_Buf_Right,3);
	HAL_UART_Transmit(&huart4,(uint8_t*) &info,1,0xFFFF);
	osSemaphoreWait(gomile6SemHandle, osWaitForever);
	diatance_temp=right_distance.right;
	return diatance_temp;

	//delay(200);
}

//Distance Ultrasonic_Feedback_right_gomile7(void)
//{
//	uint8_t info=0xA0;
//	uint8_t Rx_Buf[3]={0,0,0};
//	uint32_t Data=0x00000000;
//	Distance distance={0.0,0.0};//右侧超声波数�?????????????
//
//	uint8_t _Rx_Buf[3]={0,0,0};
//	uint32_t _Data=0x00000000;
//	Distance _distance={0.0,0.0};//前方超声波数�?????????????
//
//	                    float PID_target=0;
//						float PID_Error_Last=0;
//						float initial_distance=100;
//						float PID_Output=0,PID_Input=0;;
//						float Error = 0, Error_Total=0;
//						float KP=10, KI=3, KD=5;
//						int32_t pwm_right=0;
//						int32_t pwm_left=0;
//						//uint8_t Flag=0; //Indicate that if verifying process begin.
//						Car_Stop();
//						if (PID_Straight_Reset_Flag)
//							return distance;
//
//						PID_target=initial_distance;
//					  for(;;)
//					  {
//						  	 if (PID_Straight_Reset_Flag)
//						  		 return distance;
//
//			  	  	  	  	 //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green
//						 	HAL_UART_Transmit(&huart4,(uint8_t*) &info,1,1000);
//						 	//delay(200);
//						 	HAL_UART_Receive(&huart4,(uint8_t*) &Rx_Buf,3,1000);
//						 	Data=Data | (((uint32_t) (Rx_Buf[0]))<<16);
//						 	Data=Data | (((uint32_t) (Rx_Buf[1]))<<8);
//						 	Data=Data |((uint32_t) (Rx_Buf[2]));
//						 	//HAL_UART_Transmit(&huart1, (uint8_t *) &Data, 4, 0xFFFF);
//						 	distance.right=Data/1000;
//
//							HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);
//						//delay(200);
//						    HAL_UART_Receive(&huart5,(uint8_t*) &_Rx_Buf,3,1000);
//						    _Data=_Data | (((uint32_t) (_Rx_Buf[0]))<<16);
//						    _Data=_Data | (((uint32_t) (_Rx_Buf[1]))<<8);
//						    _Data=_Data |((uint32_t) (_Rx_Buf[2]));
//						//HAL_UART_Transmit(&huart1, (uint8_t *) &Data, 4, 0xFFFF);
//						    _distance.front=_Data/1000;
//						    if(_distance.front<100)
//						        return distance;
//
//						  	 PID_Input = distance.right;
//						  	 Error=PID_Input-PID_target;
//						     PID_Output = KP * Error  +
//						 				  KD * (Error - PID_Error_Last ) +
//										  Error_Total;
//						     Error_Total=Error_Total+KI*Error;
//						     PID_Error_Last = Error;
//
//						     pwm_right=	PWM_Mid-	(int32_t) 	PID_Output;
//						     pwm_left=	PWM_Mid+	(int32_t)  	PID_Output;
//						     pwm_right = pwm_right<PWM_Lowest ? PWM_Lowest : pwm_right;
//						     pwm_right = pwm_right>PWM_Higest ? PWM_Higest : pwm_right;
//						     pwm_left = pwm_left<PWM_Lowest ? PWM_Lowest : pwm_left;
//						     pwm_left = pwm_left>PWM_Higest ? PWM_Higest : pwm_left;
//						     if (PID_Straight_Reset_Flag)
//						     	return distance;
//						     taskENTER_CRITICAL();
//						     PWM_SET_RIGHT (pwm_right);
//						     PWM_SET_LEFT(pwm_left);
//						     taskEXIT_CRITICAL();
//						     }
//
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
  {
  	if (huart->Instance==USART2){
  		Camera_Data=0x0000;
  		Camera_Data=Camera_Data | (((uint16_t) (Rx_Buf[0]))<<8);
  		Camera_Data=Camera_Data|((uint16_t) (Rx_Buf[1]));
  		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
  		Rx_Buf[0]=0;
  		Rx_Buf[1]=0;
  		osSemaphoreRelease(CameraUARTSemHandle);
  		if((Camera_Data & 0x4000) != 0)//IF Apriltag is found
  		{
  			osSemaphoreRelease(ApriltagSemHandle);
  		}
  		if(camera_recieve_IT_flag)
  			HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
  	}
  	else if (huart->Instance==UART4)
	{
		uint32_t Data=0x00000000;
	 	Data=Data | (((uint32_t) (Rx_Buf_Right[0]))<<16);
	 	Data=Data | (((uint32_t) (Rx_Buf_Right[1]))<<8);
	 	Data=Data |((uint32_t) (Rx_Buf_Right[2]));
	 	Rx_Buf_Right[0]=0;
		Rx_Buf_Right[1]=0;
	    Rx_Buf_Right[2]=0;
	 	right_distance.right=Data/1000;
	 	osSemaphoreRelease(gomile6SemHandle);
	}
  	else if (huart->Instance==UART5){
  			uint32_t Data=0x00000000;

  			Data=Data | (((uint32_t) (Rx_Buf_Front[0]))<<16);
  			Data=Data | (((uint32_t) (Rx_Buf_Front[1]))<<8);
  			Data=Data |((uint32_t) (Rx_Buf_Front[2]));
  			//HAL_UART_Transmit(&huart1, (uint8_t *) &Data, sizeof(Data), 0xFFFF);
  			front_distance.front=Data/1000;
  			Rx_Buf_Front[0]=0;
  			Rx_Buf_Front[1]=0;
  			Rx_Buf_Front[2]=0;
  		 	front_distance.front=Data/1000;
  		 	if(front_distance.front<250){
  	  		 	osSemaphoreRelease(UltraFrontSemHandle);
  		 	}
  		 	else
  		 	{
  		 		HAL_UART_Receive_IT(&huart5,(uint8_t*) &Rx_Buf_Front,3);
  		 		HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,0xFFFF);
  		 	}
  			//HAL_UART_Receive_IT(&huart5,(uint8_t*) &Rx_Buf_Sonic,3);
  			//HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);
  		}
  }

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance==USART2)
	{
		__HAL_UART_CLEAR_OREFLAG(&huart2);
		HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
	}
	else if (huart->Instance==UART4)
	{
		__HAL_UART_CLEAR_OREFLAG(&huart4);
		HAL_UART_Receive_IT(&huart4,(uint8_t*) &Rx_Buf,3);
	}
}

void color_judge(void)
{
	finalcolor=blue>pink? 1:2;
	if(finalcolor==1)
	{
	    finalcolor=blue>yellow? 1:3;
	}
	else if(finalcolor==2)
	{
		finalcolor=pink>yellow? 2:3;
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
	duty=duty*PWM_Bias;
	if ( duty < 0 )
		{
		if (duty <= -2000*PWM_Bias)
			duty = 1;
		else
			duty = 2000*PWM_Bias + duty;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		}
	else
		{
			if (duty == 0)
				duty = 1;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		}
	if (duty > 2000*PWM_Bias)
		duty = 2000*PWM_Bias;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,duty);
}

int PID_Apriltag(float Accept_Error)
{

	float PID_target=0;
	float PID_Error_Last=0;
	float PID_Output=0,PID_Input=0;;
	float Error = 0, Error_Total=0;
	float KP=2, KI=0, KD=0.5;
	int t=0;
	uint8_t Flag=0; //Indicate that if verifying process begin.
	Car_Stop();
 	osSemaphoreWait(ApriltagSemHandle, 1000);
  for(;;)
  {
	  	 osSemaphoreWait(CameraUARTSemHandle, 0);
	  	 osSemaphoreWait(CameraUARTSemHandle, osWaitForever);
	  	 PID_Input = (Camera_Data & (0x07FF))-1000;
	  	 Error=PID_target - PID_Input;
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
	  		if(t>3)
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
	    	 taskENTER_CRITICAL();
	    	 PWM_SET_RIGHT ((int32_t) (-PID_Output));
	    	 PWM_SET_LEFT  ((int32_t)   PID_Output );
	    	 taskEXIT_CRITICAL();
	     }
	     delay(2);

}

int Apriltag_Verify(void)
{
	int sem_count=0;
	for(int i=0;i<10;i++)
	{
		if(osSemaphoreWait(ApriltagSemHandle, 500)==0)
			sem_count++;
	}
	if(sem_count>8)
		return 1;
	else
		return 0;
}

void stepping(void)
{
     float Ultra_Input=0, Ultra_Input_last=0;
     float error=0;
     int32_t pulse_increment=50;
     //float pulse_increment_float=0;
     float Kp=15;
     float PWM=0, pwm_left, pwm_right;
     vTaskResume(MileageHandle);
//	 int32_t pwm_right=0;
//   int32_t pwm_left=0;

     for(int i=0;;i++)
     {
    	  if(distance_flag)
    	  {
    		  return;
    	  }
		  critical_pulses=0;
		  vTaskResume(MileageHandle);
		  //osSemaphoreWait(MileageSemHandle, osWaitForever);
		  critical_pulses=pulse_increment+number_of_pulses;
	  	  vTaskSuspend(PIDCameraHandle);
	  	  //vTaskResume(GyroReceiveHandle);
	  	  //PID_Straight_Reset_Flag=1;
	  	  //vTaskResume(GoStraightHandle);
	  	  //delay(200);
	  	  //PID_Straight_Reset_Flag=0;
	      //osSemaphoreWait(MileageSemHandle, osWaitForever);
	      //PID_Straight_Reset_Flag=1;
	      //vTaskSuspend(GoStraightHandle);
	  	  osSemaphoreRelease(MileageSemHandle);
	      osSemaphoreWait(MileageSemHandle, 0);
	      taskENTER_CRITICAL();
	      PWM_SET_LEFT(1000);
	  	  PWM_SET_RIGHT(1000);
	      taskEXIT_CRITICAL();
	      osSemaphoreWait(MileageSemHandle, osWaitForever);
	  	  Car_Stop();
	  	  delay(100);
	  	  vTaskSuspend(MileageHandle);
	     Ultra_Input = Ultrasonic_Feedback_right();
	     error = Ultra_Input - Ultra_Input_last;
	     Ultra_Input_last = Ultra_Input;
	     if(i==0)
	     {
	    	 continue;
	     }
	     else
	     {
//         if(error>0)
//	     {
	    	 PWM=Kp*error;
	    	 PWM = PWM > 250 ? 250 : PWM;
	    	 pwm_right = -PWM;
	    	 pwm_left = PWM;
			 taskENTER_CRITICAL();
	         PWM_SET_RIGHT (pwm_right);
	         PWM_SET_LEFT(pwm_left);
			 taskEXIT_CRITICAL();
			 delay(100);
	     }
     }
}

void stepping2(void)
{
     float Ultra_Input=0, Ultra_Input_last=0,Ultra_Input_last_last=0;
     float error=0;
     float judge=0;
     int32_t pulse_increment=50;
     float Kp=15;
     float PWM=0, pwm_left, pwm_right;
     //float pulse_increment_float=0;
     //float Kp=2;
     vTaskResume(MileageHandle);
//	 int32_t pwm_right=0;
//   int32_t pwm_left=0;

     for(int i=0;;i++)
     {

		  critical_pulses=0;
		  vTaskResume(MileageHandle);
		  //osSemaphoreWait(MileageSemHandle, osWaitForever);
		  critical_pulses=pulse_increment+number_of_pulses;
	  	  vTaskSuspend(PIDCameraHandle);
	  	  //vTaskResume(GyroReceiveHandle);
	  	  //PID_Straight_Reset_Flag=1;
	  	  //vTaskResume(GoStraightHandle);
	  	  //delay(200);
	  	  //PID_Straight_Reset_Flag=0;
	      //osSemaphoreWait(MileageSemHandle, osWaitForever);
	      //PID_Straight_Reset_Flag=1;
	      //vTaskSuspend(GoStraightHandle);
	  	  osSemaphoreRelease(MileageSemHandle);
	      osSemaphoreWait(MileageSemHandle, 0);
	      taskENTER_CRITICAL();
	      PWM_SET_LEFT(1000);
	  	  PWM_SET_RIGHT(1000);
	      taskEXIT_CRITICAL();
	      osSemaphoreWait(MileageSemHandle, osWaitForever);
	  	  Car_Stop();
	  	  delay(100);
	  	  vTaskSuspend(MileageHandle);
	     Ultra_Input = Ultrasonic_Feedback_right();
	     judge = Ultra_Input<Ultra_Input_last? Ultra_Input:Ultra_Input_last;
	     judge = judge<Ultra_Input_last_last? judge:Ultra_Input_last_last;
	     if(judge>3000)
	     {
	    	 return;
	     }
	     error = Ultra_Input - Ultra_Input_last;
	     Ultra_Input_last_last = Ultra_Input_last;
	     Ultra_Input_last = Ultra_Input;
	     if(i==0)
	     {
	    	 continue;
	     }
	     else
	     {
	    	 PWM=Kp*error;
	    	 PWM = PWM > 250 ? 250 : PWM;
	    	 pwm_right = -PWM;
	    	 pwm_left = PWM;
			 taskENTER_CRITICAL();
	         PWM_SET_RIGHT (pwm_right);
	         PWM_SET_LEFT(pwm_left);
			 taskEXIT_CRITICAL();
			 delay(100);
	     }
     }
}

void feeding(void)
{
	for(int i = 0; i<3; i++)
	{
	   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_SET);
	   delay(500);
	   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_4,GPIO_PIN_RESET);
	   delay(1500);
	}
}

void sendall(void)
{

   printf("team32: che che\r\n2429410Z Yiyao Zhong\r\n2429513H Haiyang Hao\r2429453Y Chunlei Yu\r\n2429458Y Xiaoyu Yi\r\n2429488L Yunfei Ling\r\n2429494B Jinsong Bai\r\n2429459L Xiaoyuan Li\r\n2429491L Yuhan Li\r\n2429567W Aodong Wei\r\n2429264Y Jingxuan Yang\r\n");

		HAL_RTC_GetTime(&hrtc,&timenow,RTC_FORMAT_BIN);//get the time from RTC as the real world time
		HAL_RTC_GetDate(&hrtc,&datenow,RTC_FORMAT_BIN);//get the date from RTC as the real world time
		taskENTER_CRITICAL();
		printf("%02d/%02d/%02d \r\n", datenow.Year, datenow.Month, datenow.Date);	//print real time date to uart2
		printf("%02d/%02d/%02d \r\n", timenow.Hours, timenow.Minutes, timenow.Seconds);//print real world time to uart2
		taskEXIT_CRITICAL();
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch,1,HAL_MAX_DELAY);
    return ch;
}

//int fputc(int ch,FILE *fp)
//{
////    while(__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE) != SET);
////    huart6.Instance->DR = ch & 0XFF;
//	uint8_t temp=ch;
//	HAL_UART_Transmit(&huart6, &temp, 1, 0xFFFF);
//    return ch;
//}

uint8_t State_Transition(State* current_state)
{
	State next_state = Unknow;
	switch(state)
	{
		case Initial:
					next_state = Go_Mile_1;
					break;
		case Line_Search:
					if(distance_flag==0)
						next_state = Line_Search;
					else
						next_state= TurnRight;
					break;
		/*case TurnRight:
					next_state = Go_Mile_1;
					break;*/
		case TurnRight1_1:
					next_state = Go_Mile_2_1;
					break;
		case TurnRight1_2:
					next_state = Go_Mile_2_2;
					break;
		case TurnRight1_3:
					next_state = Go_Mile_2_3;
					break;
		case TurnRight2_1:
					next_state = Go_Mile_3_1;
					break;
		case TurnRight2_3:
					next_state = Go_Mile_3_3;
					break;
		case TurnRight3_1:
					next_state = Go_Mile_4;
					break;
		case TurnRight3_3:
					next_state = Go_Mile_4;
					break;
		case TurnRight4:
					next_state = GoStraight_Until_Barrier;
					break;
		case TurnRight5:
					next_state = Go_Mile_6;
					break;
		case TurnRight6:
					next_state = Go_Mile_7;
					break;
		case TurnRight7:
					next_state = Go_Mile_8_Until_Apriltag;
					break;
		case TurnRight8:
					next_state = Go_Mile_10;
					break;
		//case GoStraight_Until_Barrier:
					//osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
//					if(distance_flag==0)
//						next_state = GoStraight_Until_Barrier;
//					else
//						next_state = TurnRight;
			        //next_state = TurnRight5;
					//break;
		case Go_Mile_1:
					if(*current_state == Mile_Adjust)
		                switch(finalcolor)
		                {
		                case 1:
		                	next_state = TurnRight1_3;
		                	break;
		                case 2:
		                	next_state = TurnRight1_1;
		                	break;
		                case 3:
		                	next_state = TurnRight1_2;
		                	break;
		                default:
		                	break;
		                }


					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;
		case Go_Mile_2_1:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight2_1;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;
		case Go_Mile_2_2:
					if(*current_state == Mile_Adjust)
						next_state = Go_Mile_4;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;
		case Go_Mile_2_3:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight2_3;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;

		case Go_Mile_3_1:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight3_1;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;

		case Go_Mile_3_3:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight3_3;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;

		case Go_Mile_4:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight4;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;
		case GoStraight_Until_Barrier:
					//if(*current_state == Mile_Adjust)
						next_state = TurnRight5;
//					else
//						{
//						temp_state = *current_state;
//						next_state = Mile_Adjust;
//						}
					break;
		case Go_Mile_6:
					//if(*current_state == Mile_Adjust)
						next_state = Go_Mile_6_7;
					/*else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}*/
					 break;
		case Go_Mile_6_7:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight6;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;
		case Go_Mile_7:
//					if(*current_state == Mile_Adjust)
						next_state = TurnRight7;
//					else
//						{
//						temp_state = *current_state;
//						next_state = Mile_Adjust;
//						}
					break;
		case Go_Mile_8_Until_Apriltag:
					next_state=Apriltag_Check;
					break;
		case Go_Mile_9:
					if(*current_state == Mile_Adjust)
						next_state = TurnRight8;
					else
						{
						temp_state = *current_state;
						next_state = Mile_Adjust;
						}
					break;
//		case Go_Mile_10:
//					if(*current_state == Mile_Adjust)
//						next_state = Idle;
//					else
//						{
//						temp_state = *current_state;
//						next_state = Mile_Adjust;
//						}
//					break;
		case Apriltag_Adjust1:
					next_state = Feeding;
					break;
		case Apriltag_Check:
					if(Apriltag_Verify())
						next_state = Apriltag_Adjust1;
					else
						next_state=Go_Mile_8_Until_Apriltag;
					break;
		case Feeding:
					next_state=Go_Mile_9;
					break;
		case Communication:
					next_state=Communication;
					break;
		case Mile_Adjust:
					switch (temp_state)
					{
					case Go_Mile_1:
		                switch(finalcolor)
		                {
		                case 1:
		                	next_state = TurnRight1_3;
		                	break;
		                case 2:
		                	next_state = TurnRight1_1;
		                	break;
		                case 3:
		                	next_state = TurnRight1_2;
		                	break;
		                default:
		                	break;
		                }
						break;
					case Go_Mile_2_1:
						next_state = TurnRight2_1;
						break;
					case Go_Mile_2_2:
						next_state = Go_Mile_4;
						break;
					case Go_Mile_2_3:
						next_state = TurnRight2_3;
						break;
					case Go_Mile_3_1:
						next_state = TurnRight3_1;
						break;
					case Go_Mile_3_3:
						next_state = TurnRight3_3;
						break;
					case Go_Mile_4:
						next_state = TurnRight4;
						break;
					case GoStraight_Until_Barrier:
						next_state = TurnRight5;
						break;
					case Go_Mile_6:
						next_state = Go_Mile_6_7;
						break;
					case Go_Mile_6_7:
						next_state = TurnRight6;
						break;
					case Go_Mile_7:
						next_state = TurnRight7;
						break;
					case Go_Mile_9:
						next_state = TurnRight8;
						break;
					case Go_Mile_10:
						next_state = Idle;
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
	delay(1000);
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
		  	  	  	  	  state= Idle;
		  		  	  	  delay(500);
		  		  	  	  state= Line_Search;
		  	  	  	  	  vTaskResume(PIDCameraHandle);
		  	  	  	  	  //vTaskResume(GyroReceiveHandle);
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
		  	  	  	  	  PID_Turning(-90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight1_1:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(45,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight1_2:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight1_3:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(135,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight2_1:
	  	  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight2_3:
	  	  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(-90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight3_1:
	  	  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(-45,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight3_3:
	  	  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(45,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight4:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(-90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight5:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight6:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(-90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case TurnRight7:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;

	  case TurnRight8:
						  vTaskSuspend(DistanceCheckHandle);
		  		  	  	  vTaskSuspend(GoStraightHandle);
		  		  	  	  vTaskSuspend(MileageHandle);
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  distance_flag=0;
	  	  	  	  	  	  gyro_reset_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  delay(500);
		  	  	  	  	  PID_Turning(90,2);
		  	  	  	  	  gyro_reset_flag=1;
		  	  	  	  	  Car_Stop();
		  		  	  	  break;

	  case GoStraight_Until_Barrier:
		  	  	  	  	  //state= Idle;
		  	  	  	  	  vTaskSuspend(PIDCameraHandle);
		  	  	  	  	  vTaskSuspend(GyroReceiveHandle);
		  	  	  	  	  camera_recieve_IT_flag=0;
		  	  	  	  	  delay(500);
		  	  	  	  	  //state= GoStraight;
		  	  	  	  	  critical_distance.front=350;
		  	  	  	  	  vTaskResume(DistanceCheckHandle);
		  	  	  	  	  PWM_SET_LEFT(PWM_Mid);
		  	  	  	  	  PWM_SET_RIGHT(PWM_Mid);
		  	  	  	  	  osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  	  	  	  	  osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  	  	  	  	  Car_Stop();
		  	  	  	      //vTaskSuspend(DistanceCheckHandle);
		  	  	  	  	  break;
	  case Go_Mile:
	  					  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=33000;//室外
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
	  case Go_Mile_1:
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=200;//室外
						  //pulse_incremnet=600; //小正方形
						  camera_recieve_IT_flag=1;
						  vTaskResume(ColorcheckHandle);
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
		  	  	  	      vTaskSuspend(ColorcheckHandle);
		  	  	  	      camera_recieve_IT_flag=0;
		  	  	  	      color_judge();
		  	  	  		  Car_Stop();
		  	  	  		  gyro_reset_flag=1;
		  	  	  		  //vTaskSuspend(MileageHandle);
		  	  	  	  	  break;
	  case Go_Mile_2_1:
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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
	  case Go_Mile_2_2:
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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
	  case Go_Mile_2_3:
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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

	  case Go_Mile_3_1:
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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
	  case Go_Mile_3_3:
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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
	  case Go_Mile_4:
	  					  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=200;//室外
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
	  case Go_Mile_5:
						  //vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  //pulse_incremnet=0;//室外
						  //pulse_incremnet=600; //小正方形
		                  Ultrasonic_Feedback_front();
						  gyro_reset_flag=0;
						  vTaskResume(GyroReceiveHandle);
						  //critical_pulses=0;
						  //vTaskResume(MileageHandle);
						  delay(500);
						  //osSemaphoreWait(MileageSemHandle, osWaitForever);
						  //critical_pulses=pulse_incremnet+number_of_pulses;
		  	  	  	  	  PID_Straight_Reset_Flag=1;
		  	  	  	  	  go_straight_speed=PWM_Mid;
		  	  	  	  	  vTaskResume(GoStraightHandle);
		  	  	  	  	  delay(200);
		  	  	  	  	  PID_Straight_Reset_Flag=0;
		  	  	  	      osSemaphoreWait(UltraFrontSemHandle, osWaitForever);
		  	  	  	      PID_Straight_Reset_Flag=1;
		  	  	  	      vTaskSuspend(GoStraightHandle);
		  	  	  		  Car_Stop();
		  	  	  		  gyro_reset_flag=1;
		  	  	  		  //vTaskSuspend(MileageHandle);
		  	  	  	  	  break;
	  case Go_Mile_6:
	  	  	              vTaskSuspend(GyroReceiveHandle);
		                  critical_pulses=0;
	  	  	              vTaskSuspend(PIDCameraHandle);
	  	  	              vTaskSuspend(DistanceCheckHandle);
		  	  	  	  	  camera_recieve_IT_flag=0;
		  	  	  	  	  delay(500);
		  	  	  	  	  //state= GoStraight;
		  	  	  	  	  //critical_distance.front=350;
		  	  	  	  	  //vTaskResume(DistanceCheckHandle);
		  	  	  	      distance_flag=0;
	  	                  stepping2();
		  	  	  	      Car_Stop();
	  		              vTaskSuspend(GyroReceiveHandle);
	  		              break;
	  case Go_Mile_6_7:
	  					  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=100;//室外
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
	  case Go_Mile_7:
	  	  	              vTaskSuspend(GyroReceiveHandle);
		                  critical_pulses=0;
	  	  	              vTaskSuspend(PIDCameraHandle);
		  	  	  	  	  camera_recieve_IT_flag=0;
		  	  	  	  	  delay(500);
		  	  	  	  	  //state= GoStraight;
		  	  	  	  	  critical_distance.front=350;
		  	  	  	  	  vTaskResume(DistanceCheckHandle);
		  	  	  	      distance_flag=0;
	  	                  stepping();
		  	  	  	      Car_Stop();
		  	  	  	      vTaskSuspend(DistanceCheckHandle);
	  		              vTaskSuspend(GyroReceiveHandle);
	  		              break;
	  case Go_Mile_8_Until_Apriltag:
						  vTaskSuspend(DistanceCheckHandle);
						  gyro_reset_flag=0;
						  camera_recieve_IT_flag=1;
						  HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
						  vTaskResume(GyroReceiveHandle);
						  delay(500);
						  PID_Straight_Reset_Flag=1;
						  go_straight_speed=PWM_Mid-200;
						  vTaskResume(GoStraightHandle);
						  delay(500);
						  PID_Straight_Reset_Flag=0;
						  osSemaphoreWait(ApriltagSemHandle, 0);
						  osSemaphoreWait(ApriltagSemHandle, osWaitForever);
						  PID_Straight_Reset_Flag=1;
						  vTaskSuspend(GoStraightHandle);
						  PWM_SET_LEFT(PWM_Mid-100);
						  PWM_SET_RIGHT(PWM_Mid-100);
						  delay(1200);
						  Car_Stop();
						  gyro_reset_flag=1;
						  //vTaskSuspend(MileageHandle);
						  break;
	  case Go_Mile_9:
		  	  	  	  	  camera_recieve_IT_flag=0;
						  vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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
	  case Go_Mile_10:
						 vTaskSuspend(DistanceCheckHandle);
						  //pulse_incremnet=6900;//室内
						  pulse_incremnet=300;//室外
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
	  case Apriltag_Check:
		  	  	  	  	  Car_Stop();
		  	  	  	  	  break;
	  case Apriltag_Adjust1:
						  vTaskSuspend(DistanceCheckHandle);
						  vTaskSuspend(GoStraightHandle);
						  //vTaskSuspend(MileageHandle);
						  gyro_reset_flag=1;
						  Car_Stop();
						  delay(50);
						  distance_flag=0;
						  delay(500);
						  PID_Apriltag(5);
						  Car_Stop();
						  break;
	  case Feeding:
		  	  	  	  	  Car_Stop();
		  	  	  	  	  feeding();
		  	  	  	  	  break;
	  case Communication:
		  	  	  	  	  vTaskResume(WirelessHandle);
		  	  	  	  	  break;
	  case Mile_Adjust:
		  	  	  	  	  vTaskResume(MileageHandle);
		  	  	  	  	  osSemaphoreWait(MileageNegSemHandle, 0);
		  	  	  	  	  PWM_SET_LEFT(-PWM_Lowest-80);
		  	  	  		  PWM_SET_RIGHT(-PWM_Lowest-80);
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

/* USER CODE BEGIN Header_PIDCameraTask */
/**
* @brief Function implementing the PIDCamera thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PIDCameraTask */
void PIDCameraTask(void const * argument)
{
  /* USER CODE BEGIN PIDCameraTask */
		vTaskSuspend(PIDCameraHandle);
		float PID_Error_Last=0;
		float PID_Output=0;                    // PWM输出占空
		float Error = 0, Error_Total=0;
		int32_t PID_Input=0;
		camera_recieve_IT_flag=1;
		HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
	  /* Infinite loop */
	  for(;;)
	  {
		  if(state == Idle)
		  {
			  vTaskSuspend(PIDCameraHandle);
			  continue;
		  }
//		  	 if(camera_ready_flag==0)
//		  		 continue;
//		  	 camera_ready_flag=0;
		  	 osSemaphoreWait(CameraUARTSemHandle, osWaitForever);
		  	 HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
		  	 //delay(10);
		  	 //Data=0x03E8;
		  	 //PID_Input=-300;
		  	 PID_Input = (Camera_Data & (0x07FF))-1000;
		  	 if (PID_Input == -1000)
		  		 continue;
		  	 Error = PID_Target - PID_Input;		  // 偏差 = 目标 - 实际
		  	 PID_Output = Kp * Error  +
		  				  Kd * (Error - PID_Error_Last ) +
		  				  Error_Total;
		  	 Error_Total=Error_Total+Ki*Error;
		  	 PID_Error_Last = Error;
		  	 if(PID_Output < 0)
		  		 PID_Output-=PWM_Lowest;
		  	 else
		  		 PID_Output+=PWM_Lowest;
		     if(PID_Output > PWM_Higest-PWM_Mid) 			PID_Output =	2000-PWM_Mid;	    // 限幅
		     else if(PID_Output <-(PWM_Higest-PWM_Mid)) 	PID_Output = 	-(2000-PWM_Mid);
		     taskENTER_CRITICAL();
		     PWM_SET_RIGHT ((PWM_Mid + (int32_t) PID_Output));
		     PWM_SET_LEFT  ((PWM_Mid - (int32_t) PID_Output));
		     taskEXIT_CRITICAL();
	  }
  /* USER CODE END PIDCameraTask */
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
	vTaskSuspend(DistanceCheckHandle);
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

/* USER CODE BEGIN Header_ColorcheckTask */
/**
* @brief Function implementing the Colorcheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ColorcheckTask */
void ColorcheckTask(void const * argument)
{
  /* USER CODE BEGIN ColorcheckTask */
	vTaskSuspend(ColorcheckHandle);
	uint16_t temp=0;
  /* Infinite loop */

	HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
	for(;;)
	{
		temp=0;
	  osSemaphoreWait(CameraUARTSemHandle, osWaitForever);
	  //camera_recieve_IT_flag=0;

	  temp = Camera_Data;
	  temp = temp&(0x1800);
	  temp = (temp>>11);
	  switch(temp)
	  {
	  case 1:
		  blue++;
		  break;
	  case 2:
		  pink++;
		  break;
	  case 3:
		  yellow++;
		  break;
	  default:
		  break;
	  //camera_recieve_IT_flag=1;
	  }

	}



  /* USER CODE END ColorcheckTask */
}

/* USER CODE BEGIN Header_WirelessTask */
/**
* @brief Function implementing the Wireless thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WirelessTask */
void WirelessTask(void const * argument)
{
  /* USER CODE BEGIN WirelessTask */
	uint8_t Wireless_Rx[1];
	uint8_t test_data=0x53;
	vTaskSuspend(WirelessHandle);
  /* Infinite loop */
	for(;;){
			//HAL_UART_Transmit(&huart6,&test_data,1,0xFFFF);//retransmission part
			sendall(); //first transmission
//			HAL_UART_Receive(&huart6,Wireless_Rx,1,10000);//retransmission part
//			if (Wireless_Rx[0]==NULL)
//			{
//				printf("go ahead\n");
//			}
//			else
//			{
//				sendall();
//			}
			HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
			delay(1000);
	}
  /* USER CODE END WirelessTask */
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
