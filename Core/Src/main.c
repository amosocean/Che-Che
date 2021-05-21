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
typedef enum State {Initial=1, Line_Search, TurnRight, GoStraight, Idle, Unknow} State;
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
#define PWM_Mid 600  //无反馈时电机工作占空
#define PWM_Lowest 400
#define PWM_Higest 1000 //for our motor, this value should less than 1300
#define Angle_stable_cycles 8
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId StreamHandle;
osThreadId PIDCameraHandle;
osThreadId GyroReceiveHandle;
osThreadId DistanceCheckHandle;
osSemaphoreId CameraUARTSemHandle;
osSemaphoreId GyroReadySemHandle;
osSemaphoreId CriticalDistanceSemHandle;
/* USER CODE BEGIN PV */
State state;
volatile uint8_t Rx_Buf[2]={0,0};
//uint8_t Rx_Buf_Sonic[3]={0,0,0};
//volatile uint8_t OpenmvData[2]={0,0};
volatile uint16_t Camera_Data=0x0000;
Angle angle={0.0,0.0,0.0};
//Distance distance={0.0,0.0};
Distance critical_distance={0.0,0.0};
Distance current_distance={0,0};
int distance_flag=0;
int camera_ready_flag=0;
int gyro_ready_flag=0;
int camera_recieve_IT_flag=0;

//PID PV
volatile uint16_t PID_Target=0;
volatile float Kp = 6, Ki = 0, Kd =0;     // PID系数，这里只用到PI控制�???????????????????????????????????????
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
void StreamTask(void const * argument);
void PIDCameraTask(void const * argument);
void GyroReceiveTask(void const * argument);
void DistanceCheckTask(void const * argument);

/* USER CODE BEGIN PFP */
void Car_Initial(void);
void Car_Stop(void);
uint8_t State_Transition(State* current_state);
void PWM_SET_LEFT(int32_t duty);
void PWM_SET_RIGHT(int32_t duty);
float Angle_Diff(float target, float input);
//int Angle_Stable_Check(float Error, float Accept_Error, int wait_cycles);
int PID_Turning(float increment_angle,float Accept_Error);
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
  htim3.Init.Prescaler = 61-1;
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
  htim4.Init.Prescaler = 61-1;
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
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//�????????????????????????????????????????????启左侧PWM
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);//�????????????????????????????????????????????启右侧PWM
	taskEXIT_CRITICAL();
	//vTaskSuspend(UART_RTHandle);//Suspend UART R and T
	//vTaskSuspend(PIDCameraHandle);//Suspend PID module
}

void Car_Stop(void)
{
	PWM_SET_LEFT(1);
	PWM_SET_RIGHT(1);
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
			for(int i=0;i<20;i++)			//Get average initial direction
			{
//				if(gyro_ready_flag)
//				{
					//gyro_ready_flag=0;
					osSemaphoreWait(GyroReadySemHandle, osWaitForever);
					//ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));
					initial_yaw+=angle.z;
					delay(40);
//				}
//				else
//					continue;
			}
			initial_yaw=initial_yaw/20;
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
			     PID_Output = KP * Error  +
			 				  KD * (Error - PID_Error_Last ) +
							  Error_Total;
			     Error_Total=Error_Total+KI*Error;
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

Distance Ultrasonic_Feedback(void)
{
	uint8_t info=0xA0;
	uint8_t Rx_Buf[3]={0,0,0};
	uint32_t Data=0x00000000;
	Distance distance={0.0,0.0};
	HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);
	//delay(200);
	HAL_UART_Receive(&huart5,(uint8_t*) &Rx_Buf,3,1000);
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
  		Camera_Data=Camera_Data | (((uint16_t) (Rx_Buf[0]))<<8);
  		Camera_Data=Camera_Data|((uint16_t) (Rx_Buf[1]));
  		//Data=Data & (0x07F0);
  		//HAL_UART_Transmit(&huart1, (uint8_t*) &Data,2,10);
  		//HAL_UART_AbortReceive_IT(&huart1);
  		Rx_Buf[0]=0;
  		Rx_Buf[1]=0;
  		//camera_ready_flag=1;
  		osSemaphoreRelease(CameraUARTSemHandle);
  		if(camera_recieve_IT_flag)
  		HAL_UART_Receive_IT(&huart2,(uint8_t*) &Rx_Buf,2);
  		/*if (Rx_Cnt>20)
  			Rx_Cnt=0;*/
  	}
//  	else if (huart->Instance==UART5){
//  			uint8_t info=0xA0;
//  			uint32_t Data=0x00000000;
//
//  			Data=Data | (((uint32_t) (Rx_Buf_Sonic[0]))<<16);
//  			Data=Data | (((uint32_t) (Rx_Buf_Sonic[1]))<<8);
//  			Data=Data |((uint32_t) (Rx_Buf_Sonic[2]));
//  			HAL_UART_Transmit(&huart1, (uint8_t *) &Data, sizeof(Data), 0xFFFF);
//  			distance.front=Data/1000;
//  			Rx_Buf_Sonic[0]=0;
//  			Rx_Buf_Sonic[1]=0;
//  			Rx_Buf_Sonic[2]=0;
//  			HAL_UART_Receive_IT(&huart5,(uint8_t*) &Rx_Buf_Sonic,3);
//  			HAL_UART_Transmit(&huart5,(uint8_t*) &info,1,1000);
//  		}
  }

uint8_t State_Transition(State* current_state)
{
	State next_state = Unknow;
	switch(state)
	{
		case Initial:
<<<<<<< HEAD
					next_state = Line_Search;
=======
					next_state = GoStraight;
>>>>>>> betterPID
					break;
		case Line_Search:
					if(distance_flag==0)
						next_state = Line_Search;
					else
						next_state= TurnRight;
					break;
		case TurnRight:
					next_state = GoStraight;
					break;
		case GoStraight:
					osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
					if(distance_flag==0)
						next_state = GoStraight;
					else
						next_state = TurnRight;
					break;
		default:
					next_state = Initial;
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
	  HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);//Green LED
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
	  	  	  	  	  	  //state= Idle;
	  	  	  	  	  	  Car_Stop();
	  	  	  	  	  	  vTaskSuspend(PIDCameraHandle);
	  	  	  	  	  	  vTaskSuspend(DistanceCheckHandle);
	  	  	  	  	  	  camera_recieve_IT_flag=0;
	  	  	  	  	  	  delay(50);
	  	  	  	  	  	  state= TurnRight;
	  	  	  	  	  	  distance_flag=0;
		  	  	  	  	  vTaskResume(GyroReceiveHandle);
		  	  	  	  	  PID_Turning(-90,1.5);
		  	  	  	  	  vTaskSuspend(GyroReceiveHandle);
		  	  	  	  	  vTaskResume(DistanceCheckHandle);
		  	  	  	  	  Car_Stop();
		  		  	  	  break;
	  case GoStraight:
		  	  	  	  	  //state= Idle;
		  	  	  	  	  vTaskSuspend(PIDCameraHandle);
		  	  	  	  	  camera_recieve_IT_flag=0;
		  	  	  	  	  delay(500);
		  	  	  	  	  //state= GoStraight;
		  	  	  	  	  critical_distance.front=350;
		  	  	  	  	  vTaskResume(DistanceCheckHandle);
		  	  	  	  	  PWM_SET_LEFT(PWM_Mid);
		  	  	  	  	  PWM_SET_RIGHT(PWM_Mid);
		  	  	  	  	  //osSemaphoreWait(CriticalDistanceSemHandle, osWaitForever);
		  	  	  	  	  //HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
		  	  	  	  	  break;
	  case Idle:
		  	  	  	  	  Car_Stop();
	  default :
		  	  	  	  	  Car_Initial();
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
	  HAL_UART_Receive(&huart3, (uint8_t *) &GyroData, sizeof(GyroData), 300);
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

	  //taskENTER_CRITICAL();
	  //HAL_UART_Transmit(&huart1, (uint8_t *) &Yaw, sizeof(Yaw), 0xFFFF);
	  //taskEXIT_CRITICAL();
	  angle.x=(((float)Ax) / 32768.0 * 180.0);
	  angle.y=(((float)Ay) / 32768.0 * 180.0);
	  angle.z=(((float)Yaw) / 32768.0 * 180.0);

	  //gyro_ready_flag=1;
	  //xSemaphoreGive(GyroReadySemHandle)
	  osSemaphoreRelease(GyroReadySemHandle);
	  //xTaskNotifyGive()
	  //portMAX_DELAY;

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

//	  if(state == Idle)
//	  	  {
//	  	  	  vTaskSuspend(DistanceCheckHandle);
//	  	  	  continue;
//	  	  }
	  Distance distance={0.0,0.0};
	  Distance temp=Ultrasonic_Feedback();

	  //Distance temp=distance;
	  for(int i=0;i<1;i++)
	  {
		  distance.front+=temp.front;
	  	  delay(100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
