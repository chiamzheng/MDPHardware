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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <oled.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId robotCommandTasHandle;
osThreadId leftEncoderTaskHandle;
osThreadId rightEncoderTasHandle;
osThreadId stopBotTaskHandle;

osThreadId motorTaskHandle;
osThreadId OLEDTaskHandle;

osThreadId gyroTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);
void robotCommand(void const * argument);
void leftEncoder(void const * argument);
void rightEncoder(void const * argument);
void stopBot(void const * argument);
void LeftMotorFix(void const * argument);
void motor(void const * argument);
void OLEDShow(void const * argument);
void rightMotorFix(void const * argument);
void gyroTask1(void const * argument);

/* USER CODE BEGIN PFP */
void stopMovement(void);
void moveFrontStraight(void);
void moveBackStraight(void);
void moveForward(char dir[10], int distance);
void moveBackward(char dir[10], int distance);
float Left_PID_control (float setpoint, float measure);
float Right_PID_control (float setpoint, float measure);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[20];

uint16_t leftIR[4096]; // PC2 is left IR
uint16_t rightIR[4096]; // PC1 is right IR
int rightEncoderVal = 0, leftEncoderVal = 0;
uint16_t rightPWMval = 0, leftPWMval = 0;
char motorDir[2] = "ST";
char replacer[2]="NO";
int replace = 99;
int angle = 90;
int totalLeftEncoder=0;
int totalRightEncoder=0;
uint8_t buff[20];
int delayOS = 300; // in milisec
int motorPower=4000;
//PID variables
float Kp=6, Ki=0.01, Kd=0.01;
float T=5; /*Sample Period*/
float ruk, ruk_1=0, rek_1=0, rek_2=0, rek=0;
float luk, luk_1=0, lek_1=0, lek_2=0, lek=0;
float umax=3500;/*Constant*/
float umin=1500; /*Constant*/
char temp[6] = "test\0";
uint8_t ICMAddr = 0x68;

double Aint = 0;

uint32_t Difference = 0;
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

double total_angle=0;

int value = 0;

void readByte(uint8_t addr, uint8_t* data){
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr<<1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddr<<1, data, 2, 20);

}

void writeByte(uint8_t addr, uint8_t data){
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 2, 20);
}
//this one new i add in 19sept onli: lowpassfilter to reduce angle overcompensation

double applyLowPassFilter(double current_angle, double new_measurement, double alpha) {
    return alpha * new_measurement + (1 - alpha) * current_angle;
}
/*
void correctDirection(double target_angle, int dir){
	int pidVal;
	    //double alpha = 0.9; // Filter coefficient for smoothing

	    // Apply low-pass filter to smooth the total angle value


	    // Calculate PID value for steering
	pidVal = (int)(149 + (dir * (total_angle - target_angle) * 3.0 + 0.0008 * Aint));

	    if (pidVal <= 100) pidVal = 100;
	    if (pidVal >= 266) pidVal = 266;

	    Aint += dir * (total_angle - target_angle);
	    htim1.Instance->CCR4 = pidVal;

	/*int pidVal;

	// calibrate these for PID and gyro
	pidVal = (int)(150 + (dir*(total_angle - target_angle)*2.0 + 0.0008*Aint));
	if(pidVal <= 100){
	pidVal = 100;
	}

	if(pidVal >= 200){
		pidVal = 200;
	}
	Aint += dir*(total_angle - target_angle);
//	htim1.Instance -> CCR4 = 150;
//	osDelayUntil(10);
	htim1.Instance -> CCR4 = pidVal;

}
*/
void correctDirection(double target_angle, int dir) {
    // PID coefficients
    double k_p = 3.5;    // Proportional gain
    double k_i = 0.0012; // Integral gain
    double k_d = 0.1;    // Derivative gain

    static double previous_error = 0.0;
    double error = total_angle - target_angle;

    // Reverse the error calculation for backward movement
    if (dir == -1) {
        k_p=3.0; // Invert the error to ensure correct steering when moving backward
    }

    // Update integral term with anti-windup
    Aint += error;
    if (Aint > 1000) Aint = 1000;  // Limit the integral term
    if (Aint < -1000) Aint = -1000;

    // Calculate derivative term
    double derivative = error - previous_error;

    // Compute PID output, use different logic for forward and backward
    int pidVal;
    if (dir == 1) {
        // Forward movement
        pidVal = (int)(150 + (k_p * error + k_i * Aint + k_d * derivative));
    } else {
        // Backward movement, reverse correction direction by adjusting around 149
        pidVal = (int)(150 - (k_p * error + k_i * Aint + k_d * derivative));
    }

    // Clamp pidVal to the steering range
    if (pidVal <= 100) pidVal = 100;
    if (pidVal >= 266) pidVal = 266;

    // Set the steering PWM
    htim1.Instance->CCR4 = pidVal;

    // Save the current error for the next derivative calculation
    previous_error = error;
}
void correctDirectionO(double target_angle, int dir){
	int pidVal;
	    //double alpha = 0.9; // Filter coefficient for smoothing

	    // Apply low-pass filter to smooth the total angle value


	    // Calculate PID value for steering
	pidVal = (int)(150 + (dir * (total_angle - target_angle) * 2.0 + 0.0008 * Aint));

	    if (pidVal <= 100) pidVal = 100;
	    if (pidVal >= 266) pidVal = 266;

	    Aint += dir * (total_angle - target_angle);
	    htim1.Instance->CCR4 = pidVal;


}

void gyroInit(){

	writeByte(0x06, 0x00);
		osDelay(10);
		writeByte(0x03, 0x80);
		osDelay(10);
		writeByte(0x07, 0x07);
		osDelay(10);
		writeByte(0x06, 0x01);
		osDelay(10);
		writeByte(0x7F, 0x20);
		osDelay(10);
		writeByte(0x01, 0x2F);
		osDelay(10);
		writeByte(0x0, 0x00);
		osDelay(10);
		writeByte(0x7F, 0x00);
		osDelay(10);
		writeByte(0x07, 0x00);
		osDelay(10);

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
+  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  //Accel_Init(); // this is for accelerometer
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 4);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128*2);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of robotCommandTas */
  osThreadDef(robotCommandTas, robotCommand, osPriorityIdle, 0, 128*3);
  robotCommandTasHandle = osThreadCreate(osThread(robotCommandTas), NULL);

  /* definition and creation of leftEncoderTask */
  osThreadDef(leftEncoderTask, leftEncoder, osPriorityIdle, 0, 128*3);
  leftEncoderTaskHandle = osThreadCreate(osThread(leftEncoderTask), NULL);

  /* definition and creation of rightEncoderTas */
  osThreadDef(rightEncoderTas, rightEncoder, osPriorityIdle, 0, 128*3);
  rightEncoderTasHandle = osThreadCreate(osThread(rightEncoderTas), NULL);

  /* definition and creation of stopBotTask */
  osThreadDef(stopBotTask, stopBot, osPriorityIdle, 0, 128*3);
  stopBotTaskHandle = osThreadCreate(osThread(stopBotTask), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, motor, osPriorityIdle, 0, 128*2);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* definition and creation of OLEDTask */
  osThreadDef(OLEDTask, OLEDShow, osPriorityIdle, 0, 128*2);
  OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);

  /* definition and creation of gyroTask */
  osThreadDef(gyroTask, gyroTask1, osPriorityRealtime, 0, 128*2);
  gyroTaskHandle = osThreadCreate(osThread(gyroTask), NULL);

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
/**
  * @brief ADC2 Initialization Function
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
  hadc1.Init.DMAContinuousRequests = ENABLE;
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
  hadc2.Init.DMAContinuousRequests = ENABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin|MotorA_IN1_Pin|MotorB_IN1_Pin|MotorB_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MotorA_IN2_Pin MotorA_IN1_Pin MotorB_IN1_Pin MotorB_IN2_Pin */
  GPIO_InitStruct.Pin = MotorA_IN2_Pin|MotorA_IN1_Pin|MotorB_IN1_Pin|MotorB_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);  // Prevent unused argument warning

    // Directly copy the first two characters to motorDir
    strncpy(motorDir, aRxBuffer, 2);
    //motorDir[2] = '\0';  // Null-terminate the string

    // Extract the 3rd and 4th characters for angle
    char angleStr[3];  // Buffer for angle (2 chars + null terminator)
    strncpy(angleStr, aRxBuffer + 2, 2);  // Copy the next 2 characters
    //angleStr[2] = '\0';  // Null-terminate the string

    // Handle special cases
    if (strncmp(angleStr, "--", 2) == 0) {
        angle = replace;  // Assign special value if "--" is received
    } else if (strncmp(angleStr, "OP", 2) == 0) {
        angle = replace;  // Assign special value if "OP" is received
    } else {
        // Convert angleStr to an integer
        angle = atoi(angleStr);
    }

    // Prepare for the next UART data reception
    HAL_UART_Receive_IT(&huart3, aRxBuffer, 4);  // Receive 4 bytes

}


void delay(uint16_t duration){

	  __HAL_TIM_SET_COUNTER(&htim4,0);
	  while(__HAL_TIM_GET_COUNTER(&htim4) < duration);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
  {
    if (Is_First_Captured==0) // if the first value is not captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
      Is_First_Captured = 1;  // set the first captured as true
      // Now change the polarity to falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    else if (Is_First_Captured==1)   // if the first is already captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
      __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

      if (IC_Val2 > IC_Val1)
      {
        Difference = IC_Val2-IC_Val1;
      }

      else if (IC_Val1 > IC_Val2)
      {
        Difference = (0xffff - IC_Val1) + IC_Val2;
      }

      Distance = Difference * .034/2;
      Distance+=3;  //ultrasonic sensor is off by 3cm
      Is_First_Captured = 0; // set it back to false

      // set polarity to rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(&htim4,TIM_IT_CC1);
    }
  }
}

void ultrasonic_read(void){
  //code for ultrasound
  HAL_GPIO_WritePin(GPIOE, TRIG_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
  osDelay(1);  // wait for 10 us
  HAL_GPIO_WritePin(GPIOE, TRIG_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
}



void stopMovement(void){
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);

	//forward - Motor A
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin,GPIO_PIN_RESET); // low

	//forward - Motor B
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin,GPIO_PIN_RESET); // low
}
void rampUpMotors(int targetPWM) {
    int pwm = 0;
    int rampStep = 100;   // Step for incrementing PWM during ramp-up
    int rampDelay = 20;   // Delay in milliseconds between each step

    // Ramp up the motor speed to target
    while (pwm < targetPWM) {
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);  // Set PWM for left motor
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);  // Set PWM for right motor
        pwm += rampStep;
        osDelay(rampDelay);
    }
}
void rampDownMotors(int currentPWM) {
    int pwm = currentPWM;
    int rampStep = pwm/25;   // Step for decrementing PWM during ramp-down
    int rampDelay = 20;   // Delay in milliseconds between each step

    // Ramp down motor speed to 0
    while (pwm > 0) {
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);  // Set PWM for left motor
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);  // Set PWM for right motor
        pwm -= rampStep;
        if (pwm < 0) pwm = 0;  // Ensure PWM does not go below 0
        osDelay(rampDelay);
    }
    stopMovement();  // Ensure robot stops after ramp-down
}
/*
void moveBackward(char dir[10], int distance) {
    //stopMovement();  // Ensure motors are stopped before moving
    //osDelay(100);    // Small delay to stabilize

    // Steering logic (Left, Right, or Straight)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if (strcmp(dir, "Left") == 0)
        htim1.Instance->CCR4 = 100;  // Adjust for left turn
    else if (strcmp(dir, "Right") == 0)
        htim1.Instance->CCR4 = 220;  // Adjust for right turn
    else
        htim1.Instance->CCR4 = 150;  // Center for straight movement

    // Set GPIO pins for backward movement
    HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);  // Reverse Motor A
    HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);  // Reverse Motor B
    HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

    // Reset encoders and PID variables
    totalRightEncoder = 0;
    totalLeftEncoder = 0;
    total_angle = 0;
    Aint = 0;  // Reset PID integral

    // Calculate target PWM and distance thresholds
    int targetPWM = distance * 100 + 1500;  // Proportional to distance
    if (targetPWM > 6000) targetPWM = 6000;  // Cap PWM to avoid over-speeding
    int changePWM = distance * 4;  // Ramp-up increment
    int slowDownDistance = (int)(0.70 * distance);  // 70% of distance to start ramp-down

    int currentPWM = 3500;
    int rampStep = currentPWM / 50;
    double currentDistance = 0.0;

    // Smooth ramp-up to target speed
    /*
    while (currentPWM < targetPWM && currentDistance < slowDownDistance) {
        currentPWM += changePWM;
        if (currentPWM > targetPWM) currentPWM = targetPWM;  // Ensure it doesn't exceed target

        // Set the motor speeds according to the current PWM
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor

        // Distance calculation (backward motion, so negative)
        currentDistance = (totalLeftEncoder) / 1550.0 * -21.04;
        osDelay(10);  // Faster control loop for smoother ramp-up
    }

    // Maintain target speed until slow-down distance
    while (currentDistance < slowDownDistance) {
        currentDistance = (totalLeftEncoder) / 1550.0 * -21.04;  // Negative for backward motion

        // Use PID to correct direction if necessary (if you have PID logic)
         correctDirection(0, -1);  // Correct direction using PID (optional)
        osDelay(10);  // Small delay for stability
    }

    // Smooth ramp-down as we approach the target distance
    while (currentDistance < distance) {
        currentDistance = (totalLeftEncoder) / 1550.0 * -21.04;  // Negative for backward motion

        // Decrease motor speeds gradually
        currentPWM -= rampStep;
        if (currentPWM < 0) currentPWM = 0;  // Ensure PWM does not go below 0

        // Apply the new PWM values to the motors
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor

        // Stop if we've reached or exceeded the target distance
        if (currentDistance >= distance) {
            stopMovement();
            break;
        }
        osDelay(10);  // Small delay for fine control
    }

    stopMovement();  // Ensure robot stops at the end

    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
           	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor
    while(currentDistance < distance){
    	correctDirection(0, -1);  // Correct direction using PID
        	        	currentDistance = (totalLeftEncoder) / 1550.0 * -21.04;
        	        if (currentDistance >= distance) {
        	                        stopMovement();  // Ensure the robot stops at the end

        	                    }
        	        }
}
*/

void moveForward(char dir[10], int distance){
    //stopMovement();
    //osDelay(100);


    // Steering logic
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if(strcmp(dir, "Left") == 0)
        htim1.Instance->CCR4 = 100;
    else if(strcmp(dir, "Right") == 0)
        htim1.Instance->CCR4 = 220;
    else
        htim1.Instance->CCR4 = 150;

    // Set direction for forward movement
    HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

    // Reset encoders and PID
    totalRightEncoder = 0;
    totalLeftEncoder = 0;
    total_angle = 0;
    Aint = 0;  // Reset PID integral

    // Calculate target PWM and distance thresholds
    int targetPWM = distance * 100 + 1500;  // Proportional to distance
    if (targetPWM > 6000) targetPWM = 6000;  // Cap PWM to avoid over-speeding
    int changePWM = distance * 4;  // Ramp-up increment
    int slowDownDistance = (int)(0.60 * distance);  // 70% of distance to start ramp-down
    float stepSize = 1;
    int currentPWM=6000;
    int totalSteps = (distance-slowDownDistance)/stepSize;
    int rampStep = targetPWM / totalSteps;
    double currentDistance = 0.0;
    double prevDistance = 0.0;
    /*
    	while (currentPWM < targetPWM && currentDistance < slowDownDistance) {
    	        currentPWM += changePWM;
    	        if (currentPWM > targetPWM) currentPWM = targetPWM;  // Ensure it doesn't exceed target

    	        // Set the motor speeds according to the current PWM
    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor

    	        currentDistance = (totalLeftEncoder) / 1550.0 * 21.04; // Average both encoders
    	        prevDistance = currentDistance;
    	        correctDirection(0, 1);
    	        osDelay(10);  // Faster control loop for smoother ramp-up

    	    }
		*/
    // Smooth ramp-up to target speed
    if(distance<40) currentPWM=2000;
    else if (distance<60)currentPWM=4000;
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor
      /*
    // Maintain target speed until slow-down distance
    while (currentDistance < slowDownDistance) {
        currentDistance = (totalLeftEncoder) / 1550.0 * 21.04; // Average both encoders

        // Use PID to correct direction if necessary
        correctDirection(0, 1);  // Correct direction using PID
        osDelay(1);  // Small delay for stability
    }
*/
    // Smooth ramp-down as we approach the target distance
       //if (distance>40){
		while(currentDistance < slowDownDistance){
			currentDistance = (totalLeftEncoder) / 1550.0 * 21.04;
			correctDirection(0, 1);  // Correct direction using PID
			/*if (currentDistance >= distance) {
				stopMovement();  // Ensure the robot stops at the end

			}*/
		}

			int basePWM = 1000;
				   int rampStep2 = currentPWM / 20;
				   while (1) {
						currentDistance = (totalLeftEncoder) / 1550.0 * 21.04; // Average both encoders

						// Decrease motor speeds gradually
						currentPWM -= rampStep2;
						if (currentPWM < basePWM) currentPWM = basePWM;  // Ensure PWM does not go below 0

						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor

						// Stop if we've reached or exceeded the target distance
						if (currentDistance >= distance) {
							stopMovement();  // Ensure the robot stops at the end
							break;
						}

						correctDirection(0, 1);  // Correct direction using PID

						osDelay(10);  // Small delay for fine control
					}
		}
		/*
       else if(distance <= 40) { // for less than 40 distance
    	   while(1){
    		   currentDistance = (totalLeftEncoder) / 1550.0 * 21.04;
    		   if(currentDistance >= distance){

    			   stopMovement();  // Ensure the robot stops at the end
    			   break;
    		   }
    	   }

       }

    	//stopMove
        //stopMovement();  // Ensure the robot stops at the end
}*/
void moveBackward(char dir[10], int distance){
    //stopMovement();
    //osDelay(100);


    // Steering logic
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if(strcmp(dir, "Left") == 0)
        htim1.Instance->CCR4 = 100;
    else if(strcmp(dir, "Right") == 0)
        htim1.Instance->CCR4 = 220;
    else
        htim1.Instance->CCR4 = 150;

    // Set direction for forward movement
    HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);  // Reverse Motor A
        HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);  // Reverse Motor B
        HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

    // Reset encoders and PID
    totalRightEncoder = 0;
    totalLeftEncoder = 0;
    total_angle = 0;
    Aint = 0;  // Reset PID integral

    // Calculate target PWM and distance thresholds
    int targetPWM = distance * 100 + 1500;  // Proportional to distance
    if (targetPWM > 6000) targetPWM = 6000;  // Cap PWM to avoid over-speeding
    int changePWM = distance * 4;  // Ramp-up increment
    int slowDownDistance = (int)(0.60 * distance);  // 70% of distance to start ramp-down
    float stepSize = 1;
    int currentPWM=6000;
    int totalSteps = (distance-slowDownDistance)/stepSize;
    int rampStep = targetPWM / totalSteps;
    double currentDistance = 0.0;
    double prevDistance = 0.0;
    /*
    	while (currentPWM < targetPWM && currentDistance < slowDownDistance) {
    	        currentPWM += changePWM;
    	        if (currentPWM > targetPWM) currentPWM = targetPWM;  // Ensure it doesn't exceed target

    	        // Set the motor speeds according to the current PWM
    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor

    	        currentDistance = (totalLeftEncoder) / 1550.0 * 21.04; // Average both encoders
    	        prevDistance = currentDistance;
    	        correctDirection(0, 1);
    	        osDelay(10);  // Faster control loop for smoother ramp-up

    	    }
		*/
    // Smooth ramp-up to target speed
    if(distance<40) currentPWM=2000;
    else if (distance<60)currentPWM=4000;
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor
      /*
    // Maintain target speed until slow-down distance
    while (currentDistance < slowDownDistance) {
        currentDistance = (totalLeftEncoder) / 1550.0 * 21.04; // Average both encoders

        // Use PID to correct direction if necessary
        correctDirection(0, 1);  // Correct direction using PID
        osDelay(1);  // Small delay for stability
    }
*/
    // Smooth ramp-down as we approach the target distance
       //if (distance>40){
		while(currentDistance < slowDownDistance){
			currentDistance = (totalLeftEncoder) / 1550.0 * -21.04;
			correctDirection(0, -1);  // Correct direction using PID
			/*if (currentDistance >= distance) {
				stopMovement();  // Ensure the robot stops at the end

			}*/
		}

			int basePWM = 1000;
				   int rampStep2 = currentPWM / 20;
				   while (1) {
						currentDistance = (totalLeftEncoder) / 1550.0 * -21.04; // Average both encoders

						// Decrease motor speeds gradually
						currentPWM -= rampStep2;
						if (currentPWM < basePWM) currentPWM = basePWM;  // Ensure PWM does not go below 0

						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, currentPWM);  // Left motor
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, currentPWM);  // Right motor

						// Stop if we've reached or exceeded the target distance
						if (currentDistance >= distance) {
							stopMovement();  // Ensure the robot stops at the end
							break;
						}

						correctDirection(0, -1);  // Correct direction using PID

						osDelay(10);  // Small delay for fine control
					}
		}
		/*
       else if(distance <= 40) { // for less than 40 distance
    	   while(1){
    		   currentDistance = (totalLeftEncoder) / 1550.0 * 21.04;
    		   if(currentDistance >= distance){

    			   stopMovement();  // Ensure the robot stops at the end
    			   break;
    		   }
    	   }

       }

    	//stopMove
        //stopMovement();  // Ensure the robot stops at the end
}*/
void moveForwardO(char dir[10], int distance){
	stopMovement();
	osDelay(100);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	if(strcmp(dir, "Left") == 0)
		htim1.Instance->CCR4 = 100;
	else if(strcmp(dir, "Right") == 0)
		htim1.Instance->CCR4 = 220;
	else
		htim1.Instance->CCR4 = 150;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000); //left wheel stronger
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000); //fix at 2k be nice

//	osDelay(2250);   //latest
	// forward movement delay CLEARED CHECKLIST @ 2725/100
	int t = 0;
	//HAL_GetTick()
	uint32_t PreviousWakeTime = osKernelSysTick();
	int tick = HAL_GetTick();
	uint32_t delay = angle * (12800/10);
	//original value before PID: (2875/100) ; 5th Oct 4pm
	//prev value (12500/10) ; 6th Oct 1pm

	//totalLeftEncoder = 0;
	totalRightEncoder = 0;
	total_angle = 0;

	Aint = 0;

	// for forward movement, adjust the denominator (1500) to be more if the distance traversed is not enough
	while((totalLeftEncoder / 1550.0 * -21.04) < distance){
	// value was 1500 at 6th Oct 1150pm; 160cm: undershoot 7cm, 20cm: perfect
	// value was 1600 at 6th Oct 1150pm; 160cm: perfect, 20cm: overshot
	// value was 1550 at 7th Oct 1209am; 160cm: ok, 10cm: overshot by 2cm
	// value was 1520 at 7th Oct 1220am; 160cm: ok, 20cm: overshot by 0.5
	// value was 1515 at 7th Oct 1240am; 160cm: ok, 20cm: overshot


		correctDirection(0, 1);
		//osDelayUntil(&PreviousWakeTime,5);
		osDelay(1);

	}
	stopMovement();


	//HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 800); //left wheel was 1650
	//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 800);

	//while(totalLeftEncoder/1600.0 * 21.04 > distance){
		// value was 1500 at 6th Oct 1150pm; 160cm: undershoot 7cm, 20cm: perfect
		// value was 1600 at 6th Oct 1150pm; 160cm: perfect, 20cm: overshot
		// value was 1550 at 7th Oct 1209am; 160cm: ok, 10cm: overshot by 2cm
		// value was 1520 at 7th Oct 1220am; 160cm: ok, 20cm: overshot by 0.5
		// value was 1515 at 7th Oct 1240am; 160cm: ok, 20cm: overshot


			//correctDirection(0, -1);
			//osDelayUntil(&PreviousWakeTime,5);
			//osDelay(1);

	//}
	//uint32_t delay = angle * (2875/100); //(2775/100) //300 for 10cm (perfect)
	//osDelay(delay);



	//stopMovement();
	//motorDir = 0;
}


void moveBackwardO(char dir[2], int distance) {
    uint32_t delay = 0;
    stopMovement();
    osDelay(100);

    // Start PWM for steering angle (Left, Right, or Straight)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    if (strcmp(dir, "Lf") == 0)
        htim1.Instance->CCR4 = 100;  // Adjust for left
    else if (strcmp(dir, "Rt") == 0)
        htim1.Instance->CCR4 = 210;  // Adjust for right
    else
        htim1.Instance->CCR4 = 150;  // Center for straight movement

    // Set GPIO pins for backward movement
    HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);  // Reverse motor A
    HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);  // Reverse motor B
    HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

    // Set PWM values for both wheels (reverse direction)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);  // Left wheel (adjust for reverse)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);  // Right wheel

    totalLeftEncoder = 0;
    totalRightEncoder = 0;  // Reset encoders
    total_angle = 0;
    Aint = 0;

    // Adjust encoder comparison for backward movement
    while (totalLeftEncoder / 1550.0 * -21.04 < distance) {  // Same ratio but moving backward
        correctDirection(0, -1);  // Correct direction for backward motion
        osDelay(1);  // Adjusted loop delay
    }

    stopMovement();  // Stop the movement after reaching the desired distance

    // Reset motorDir to "ST" after movement stops
}
void moveLBackward(char dir[2], int target_angle) {
    uint32_t delay = 0;
    total_angle = 0;
    stopMovement();
    osDelay(100);

    // Start PWM for steering angle (Left, Right, or Straight)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

     htim1.Instance->CCR4 = 100;  // Adjust for left

    // Set GPIO pins for backward movement
    HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);  // Reverse motor A
    HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);  // Reverse motor B
    HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

    // Set PWM values for both wheels (reverse direction)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500);  // Left wheel (adjust for reverse)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 3000);  // Right wheel
    while(total_angle*-1  <= target_angle){
    		osDelay(1);
    		}





    // Adjust encoder comparison for backward movement
    /*while (totalLeftEncoder / 1550.0 * -21.04 < distance) {  // Same ratio but moving backward
        correctDirection(0, -1);  // Correct direction for backward motion
        osDelay(1);  // Adjusted loop delay
    }*/

    stopMovement();  // Stop the movement after reaching the desired distance
    osDelay(100);
    Aint = 0;
    htim1.Instance->CCR4 = 150;

    // Reset motorDir to "ST" after movement stops
}
void moveRBackward(char dir[2], int target_angle) {
	total_angle = 0;
    uint32_t delay = 0;
    stopMovement();
    osDelay(100);

    // Start PWM for steering angle (Left, Right, or Straight)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

        htim1.Instance->CCR4 = 210;  // Adjust for left

    // Set GPIO pins for backward movement
    HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);  // Reverse motor A
    HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);  // Reverse motor B
    HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

    // Set PWM values for both wheels (reverse direction)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 3000);  // Left wheel (adjust for reverse)
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);  // Right wheel
    while(total_angle  <= target_angle){
    		osDelay(1);
    		}

    osDelay(100);
    stopMovement();  // Stop the movement after reaching the desired distance
    htim1.Instance->CCR4 = 150;

    // Reset motorDir to "ST" after movement stops
}
void motorRight(int target_angle){

	total_angle = 0;
	//target_angle = angle;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 266;

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_RESET); // low

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_RESET); // low

	//back
//HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // low
	//HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // high

	osDelay(250);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,	3000); //motor A; 1000 at 14 oct 245am
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500); //motor B;

	osDelay(10);
	while(total_angle*-1  <= target_angle){
		osDelay(10);
		}
	/*
	stopMovement();

	double Aint = 0;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	osDelay(10);

	while((int)-1*total_angle > (target_angle)){
		//change this (500) to improve turning correction
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (int)(500*(-1*total_angle - target_angle) + 0.1*Aint));
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (int)(500*(-1*total_angle - target_angle) + 0.1*Aint)); //motor B

		Aint += -1*total_angle - target_angle;

		osDelay(10);
	}
	*/
	stopMovement();
	htim1.Instance->CCR4 = 150;
	//motorDir = 0;
	strncpy(motorDir, "ST", sizeof(motorDir) - 1);  // Copy up to 2 characters
	    motorDir[sizeof(motorDir) - 1] = '\0';  // Ensure null termination


}
void motorLeft(int target_angle){

	total_angle = 0;
	//target_angle = angle;

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 100;

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_RESET); // low

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_RESET); // low

	//back
//	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // low
//	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // high

	osDelay(250);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500); //motor A
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 3000); //motor B; 1000 at 14 oct 245am

	osDelay(10);
	while(total_angle  <= target_angle){
		osDelay(1);
		}
	/*
	stopMovement();

	double Aint = 0;

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	osDelay(10);

	while((int)total_angle > (target_angle)){
		//motorOnA = (int)(200*(total_angle + target_angle) + 0.1*Aint);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (int)(400*(total_angle - target_angle) + 0.1*Aint));
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (int)(400*(total_angle - target_angle) + 0.1*Aint)); //motor B

		Aint += total_angle - target_angle;

		osDelay(10);
	}
	*/
	stopMovement();
	htim1.Instance->CCR4 = 150;
	//motorDir = 0;
	strncpy(motorDir, "ST", sizeof(motorDir) - 1);  // Copy up to 2 characters
	    motorDir[sizeof(motorDir) - 1] = '\0';  // Ensure null termination
	osDelay(500);

}
void moveFrontStraight(){
	stopMovement();
	osDelay(100);

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500);
}

void moveBackStraight(){
	stopMovement();
	osDelay(100);

	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500);
}

void trail_left(void){
	//does 90 degree turn but not on the spot
	stopMovement();
	osDelay(100);


	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 100;

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_RESET); // low

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // High
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_RESET); // low

	osDelay(250);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 3000); //motor B
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 50); //motor A

	uint32_t delay = angle * (1085/90);//(1075/180); //for 90 - 1350
//	osDelay(1550); //90 i think
	//osDelay(3200); // 180 i think
	//osDelay(6200);
	osDelay(delay);
	stopMovement();
	//motorDir = 0;
}

void trail_right(void){
	//does 90 kinda but not on the spot
	stopMovement();
	osDelay(100);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = 220;

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorA_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorA_IN2_Pin, GPIO_PIN_RESET); // low

	//forward
	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_SET);  // high
	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_RESET); // low

	//back
//	HAL_GPIO_WritePin(GPIOA,MotorB_IN1_Pin, GPIO_PIN_RESET);  // low
//	HAL_GPIO_WritePin(GPIOA,MotorB_IN2_Pin, GPIO_PIN_SET); // high

	osDelay(250);

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 3000); //motor A //3500 - 980 +-
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 50); //motor B


//	osDelay(1550); //90 i think
//	osDelay(3100); // 180 i think
	uint32_t delay = angle * (1190/90); // 23 sep 0903: 1160/90
	//(2710/180); //perfect previously 1347.5 - for 90: 1352.5
	osDelay(delay);
	stopMovement();
	//motorDir = 0;
	strncpy(motorDir, "ST", sizeof(motorDir) - 1);  // Copy up to 2 characters
	    motorDir[sizeof(motorDir) - 1] = '\0';  // Ensure null termination
}

void left_ninety(void)
{
	angle = 45;
	trail_left();
	osDelay(500);

	//motorDir = 7;
	moveBackward("Right", angle);

	//motorDir = 7;
	angle = 5;
	moveBackward("Right", angle);

	osDelay(500);
	//motorDir = 7;
	angle = 10;
	moveForward("Straight", angle);

	stopMovement();
	//motorDir = 0;
}

void right_ninety(void)
{
	angle = 45;
	trail_right();
	osDelay(500);

	//motorDir = 7;
	moveBackward("Left", angle);

	//motorDir = 7;
	angle = 7;
	moveBackward("Left", angle);

	osDelay(500);
	//motorDir = 7;
	angle = 10;
	moveForward("Straight", angle);

	stopMovement();
	//motorDir = 0;
}

void moveObstacleSide(void){
	//motorDir = 7;
	angle = 90;
//	trail_left();
	moveBackward("Right", angle);
	osDelay(500);

	//motorDir = 7;
	angle = 50;
	moveForward("Straight", angle);
	osDelay(500);

	//motorDir = 7;
	angle = 90;
	trail_right();
	osDelay(500);

	stopMovement();
	//motorDir = 0;
}

float Left_PID_control (float setpoint, float measure)
{
	lek_2=lek_1;
	lek_1=lek;
	lek=setpoint-measure;
	luk_1=luk;
	luk=luk_1+Kp*(lek-lek_1) +Ki*(T/2)*(lek+lek_1)+ (Kd/T)*(lek-2*lek_1+lek_2);
//	if (luk>umax) luk=umax;
//	if (luk<umin) luk=umin;
	return (luk);
}

float Right_PID_control (float setpoint, float measure)
{
	rek_2=rek_1;
	rek_1=rek;
	rek=setpoint-measure;
	ruk_1=ruk;
	ruk=ruk_1+Kp*(rek-rek_1) +Ki*(T/2)*(rek+rek_1)+ (Kd/T)*(rek-2*rek_1+rek_2);
//	if (ruk>umax) ruk=umax;
//	if (ruk<umin) ruk=umin;
	return (ruk);
}

// turns on motor in fwd direction
void motorForward(int pwm){
	// these 4 lines control direction of the motors
	// HAL is a system library that has a function "GPIO write pin"
	// GPIOA is the port (port A)
	// MotorA_IN_Pin is the pin number (PA3)
	// set is 1, reset is 0
	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

	//
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,  pwm);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);

}

void motorReverse(int pwm){
	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwm);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwm);

}

void moveForwardIndefinitely(int instruct)
{
	int dis=0;
	int dis1=0;
	if(instruct==99){
		dis=40;
		dis1=20;
	}
	else{
		dis=50;
		dis1=30;
	}
	htim1.Instance->CCR4 = 150;
	osDelay(100);


	// Set PWM to move both motors forward

	if(Distance>dis){
		HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);
			osDelay(10);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 3000);  // Adjust this value as per speed
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 3000);  // Adjust this value as per speed
		while(Distance > dis){
								//correctDirection(0, 1);

				correctDirection(0, 1);  // Correct direction using PID
				osDelay(10);
		}
		rampDownMotors(3000);
	}
	else if(Distance<dis1){
		HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);
			osDelay(10);
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
			while(Distance<dis1){
				correctDirection(0, -1);  // Correct direction using PID
				osDelay(10);
			}
			rampDownMotors(1500);
	}

}

void moveBackwardIndefinitely(void)
{
	htim1.Instance->CCR4 = 150;
		osDelay(100);


		// Set PWM to move both motors forward
		if(41826*(1/(double)leftIR[0]) - 10.03 < 60){
			HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);
				osDelay(10);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
				while(41826*(1/(double)leftIR[0]) - 10.03 < 60){
					correctDirectionO(0, 1);  // Correct direction using PID
					osDelay(10);
				}
				rampDownMotors(1500);
		}
		if(41826*(1/(double)leftIR[0]) - 10.03 >60){
			HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);
						osDelay(10);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
						while(41826*(1/(double)leftIR[0]) - 10.03 > 60){
							correctDirectionO(0, 1);  // Correct direction using PID
							osDelay(10);
						}
						rampDownMotors(1500);
				}

}

void rightWallCheck(void){
	htim1.Instance->CCR4 = 150;
		osDelay(100);
		total_angle=0;

		// Set PWM to move both motors forward
		//if(41826*(1/(double)leftIR[0]) - 10.03< 500){ // move backwards
		if(leftIR[0]< 500){
			HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);
				osDelay(10);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
				while(leftIR[0]< 500){
					correctDirectionO(0, 1);  // Correct direction using PID
					osDelay(10);
				}
				rampDownMotors(1500);
				moveForward("Straight", 10);

		}
		else //(41826*(1/(double)leftIR[0]) - 10.03 >500)
			{ //move forwards
						HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);
						osDelay(10);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
						while(leftIR[0]> 500){
							correctDirectionO(0, 1);  // Correct direction using PID
							osDelay(10);
						}
						rampDownMotors(1500);
				}

}
void leftWallCheck(void){
	htim1.Instance->CCR4 = 150;
		osDelay(100);
		total_angle=0;

		// Set PWM to move both motors forward
		//if(37411*(1/(double)rightIR[0]) - 8.57 < 500)
		if(rightIR[0]< 1000){ //move backwards
			HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);
				osDelay(10);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
				while(rightIR[0]< 1000){
					correctDirectionO(0, 1);  // Correct direction using PID
					osDelay(10);
				}
				rampDownMotors(1500);
				moveForward("Straight", 20);
		}
		else {
			HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);
						osDelay(10);
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust this value as per speed
						__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust this value as per speed
						while(rightIR[0]> 1000){
							correctDirectionO(0, 1);  // Correct direction using PID
							osDelay(10);
						}
						rampDownMotors(1500);
						moveForward("Straight", 10);
				}

}
void rightWallCheckUntilDetected(void) {
	int previous = 0;
	    htim1.Instance->CCR4 = 150;  // Some initial PWM setting
	    osDelay(100);

	    // Initially, the robot moves forward continuously until the right wall is detected.

	    	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);    // Move forward
	    	        HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	    	        HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
	    	        HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

	    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust speed
	    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust speed

	    while (previous<300) {  // Right wall not detected
	    	correctDirectionO(0, 1);  // Correct direction (e.g., using PID control)

	        osDelay(10);
	        previous=leftIR[0]-previous;
	        if (previous<0){
	        	previous=0;
	        }
	        osDelay(10);
	    }
	    rampDownMotors(1500);

}
void leftWallCheckUntilDetected(void) {
	int previous = 0;
    htim1.Instance->CCR4 = 150;  // Some initial PWM setting
    osDelay(100);

    // Initially, the robot moves forward continuously until the right wall is detected.

    	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);    // Move forward
    	        HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
    	        HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
    	        HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Adjust speed
    	        __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Adjust speed

    while (previous<300) {  // Right wall not detected
    	correctDirectionO(0, 1);  // Correct direction (e.g., using PID control)

        osDelay(10);
        previous=rightIR[0]-previous;
        if (previous<0){
        	previous=0;
        }
        osDelay(10);
    }
    rampDownMotors(1500);

}
void turnLeftIndefinitely(void)
{
	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_RESET); // Left motor backward
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);    // Right motor forward
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

	// Set PWM for turning (adjust PWM values for turning speed)
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1500);  // Left motor slower (turning)
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000);  // Right motor faster
}
void turnRightIndefinitely(void)
{
	HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);    // Left motor forward
	HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_RESET);  // Right motor backward
	HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_SET);

	// Set PWM for turning (adjust PWM values for turning speed)
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000);  // Left motor faster
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1500);  // Right motor slower (turning)
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	HAL_ADC_Start_DMA(&hadc1, rightIR, 4096); // PC1 is right IR
	HAL_ADC_Start_DMA(&hadc2,  leftIR, 4096); // PC2 is left IR
	htim1.Instance->CCR4 = 150;
	gyroInit();
	//motorDir = 20;
	//angle = 90;
	//osDelay(5000);
  for(;;)
  {
	  ultrasonic_read();
	  osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_robotCommand */
/**
* @brief Function implementing the robotCommandTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_robotCommand */
void robotCommand(void const * argument)
{
  /* USER CODE BEGIN robotCommand */
  /* Infinite loop */
	/*
	 * 5bit Instruction
	 *  |--------------------------------------------------------------------------------|
	 *  |Instruction		|			Action											 |
	 *  |--------------------------------------------------------------------------------|
	 *  |  00xxx			|	Stop Movement											 |
	 *  |  01xxx			|	Move forward for xxx distance (Straight Line)			 |
	 *  |  02xxx			|	Turn Left for xxx angle (forward)						 |
	 *  |  03xxx			|	Turn Right for xxx angle (forward)						 |
	 *  |  11xxx			|	Move backward for xxx distance (Straight Line)			 |
	 *  |  12xxx			|	Turn Left for xxx angle (Backward)						 |
	 *  |  13xxx			|	Turn Right for xxx angle (Backward)						 |
	 *  |--------------------------------------------------------------------------------|
	 *
	 */

	int turnDegree=0;
	double target_angle;
	double Aint;

	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);

	for(;;){
		target_angle = 0;

		if (strncmp(motorDir, "ST", 2) == 0){
			stopMovement();
			HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);

		}else if (strncmp(motorDir, "FW", 2) == 0){
							total_angle=0;
							if(angle==99){
								Aint = 0;
								gyroInit();
								moveForwardIndefinitely(99);
								//
							}
							else if(angle==98){
															Aint = 0;
															gyroInit();
															moveForwardIndefinitely(98);
															//
														}
							else{
								printf("FW reached");
								HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

															// forward left and right wheels CLEARED CHECKLIST @ left: 2650; right: 2000
															//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1800); //left wheel was 1650
															//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000); //right wheel was 1400


								Aint = 0;

								gyroInit();
								turnDegree = 0;
								if(angle==10){
									angle=7;
										}

									else if(angle==20){
										angle=19;
									}
								/*
									else if(angle==30){
										angle=32;
									}
									else if(angle==40){
										angle=40;
										}
									else if(angle==50){
										angle=48;
										}
									else if(angle==60){
										angle=54;
											}
									else if(angle==70){
										angle=67;
												}
												*/
									else if(angle==80){
										angle=81;
													}
									else if(angle==90){
										angle=92;
												}
								//moveForward("Straight", (angle-5)*0.9);
								moveForward("Straight", angle);
							}
							//osDelay(delayOS);
							HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);


		}else if (strncmp(motorDir, "BW", 2) == 0){
			total_angle=0;
							if(angle==99){
								moveBackwardIndefinitely();
										}
							else{
							Aint = 0;
							gyroInit();
							/*
							if(angle==10){
								angle=9;
								}
								else if(angle==20){
									angle=15;
									}
								else if(angle==30){
									angle=24;
								}
								else{
									angle=angle-10;
								}
							*/
							if(angle==60){
								angle=62;
							} else if (angle==70) {
								angle=72;
							} else if (angle==80) {
								angle=83;
							} else if (angle==90) {
								angle=93;
							}
							//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2000); //left wheel was 1650
							//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000); //right wheel was 1400
							moveBackward("Straight", angle);
							}
							//osDelay(delayOS);
							HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
		}else if (strncmp(motorDir, "FL", 2) == 0){
			total_angle=0;
							if(angle==99){
								turnLeftIndefinitely();
													}
							else{
							Aint = 0;
							gyroInit();
							//osDelay(delayOS);

							//motorLeft(90);
							//moveForward("Straight", 1);
							//osDelay(delayOS);
							motorLeft(angle-6);
							osDelay(delayOS);
							moveForward("Straight", 4);
							//moveForwardO("Straight", 5);

							/*moveBackward("Straight", 10);
							osDelay(1);
							motorLeft(40);
							moveBackward("Straight", 10);
							*/
							}
							//osDelay(delayOS);
							HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
		}else if (strncmp(motorDir, "FR", 2) == 0){
			total_angle=0;
							if(angle==99){
									turnRightIndefinitely();
																}
							else{
							Aint = 0;
							gyroInit();
							//osDelay(delayOS);
							//moveBackward("Straight", 1);//in lab
							//moveForward("Straight", 2);
							osDelay(delayOS);
							motorRight(angle-7);// inside lab
							osDelay(delayOS);
							//moveForward("Straight", 2);
							//moveForwardO("Straight", 4);
							//motorRight(90);
							//osDelay(delayOS);
							//moveBackward("Straight", 1);
							//osDelay(1);
							//motorRight(10);
							//moveBackward("Straight", 10);
							}
							//osDelay(delayOS);
							HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
		}else if (strncmp(motorDir, "BL", 2) == 0){
			total_angle=0;
										Aint = 0;
										gyroInit();
										//osDelay(delayOS);



										//motorLeft(angle);
			moveBackward("Straight", 7);//in lab
			osDelay(delayOS);
			moveLBackward("Lf", angle-6);
			osDelay(delayOS);
			//moveBackward("Straight", 3);//in lab
			osDelay(delayOS);
			moveBackward("Straight", 2);//in lab
			//osDelay(delayOS);
			HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);

		}else if (strncmp(motorDir, "BR", 2) == 0){
			total_angle=0;
			Aint = 0;
			gyroInit();
			moveBackward("Straight", 2);//in lab

			//moveBackward("Straight", 2);//for lab
			//osDelay(delayOS);
			moveRBackward("Rt", angle-5);// for lab
			osDelay(delayOS);
			moveBackward("Straight", 1);//for lab
			//moveForwardO("Straight", 1);
			//osDelay(delayOS);
			HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);


		}
		else if (strncmp(motorDir, "SL", 2) == 0){
			total_angle=0;
									angle=45;
									Aint = 0;
									gyroInit();
									osDelay(delayOS);

									motorLeft(angle-6);
									osDelay(delayOS);
									moveForward("Straight", 5);
									osDelay(delayOS);
									motorRight(angle-8);
									osDelay(delayOS);
									motorRight(angle-8);
									osDelay(delayOS);
									moveForward("Straight", 3);
									osDelay(delayOS);
									motorLeft(angle-6);
									osDelay(delayOS);


									//osDelay(delayOS);
									HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
		}
		else if (strncmp(motorDir, "SR", 2) == 0){
			total_angle=0;
				angle=45;
				Aint = 0;
				gyroInit();
				osDelay(delayOS);
				motorRight(angle-8);
				osDelay(delayOS);
				moveForward("Straight", 8);
				osDelay(delayOS);
				motorLeft(angle-6);
				osDelay(delayOS);
				motorLeft(angle-6);
				osDelay(delayOS);
				moveForward("Straight", 3);
				osDelay(delayOS);
				motorRight(angle-8);
				osDelay(delayOS);



				HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
		}
		else if (strncmp(motorDir, "LR", 2) == 0){
					total_angle=0;
					angle=90;
					Aint = 0;
					gyroInit();
					osDelay(delayOS);
					motorRight(angle-8);
					osDelay(delayOS);
					leftWallCheck();
					osDelay(delayOS);
					motorLeft(174);


					osDelay(delayOS);
					leftWallCheck();
					osDelay(delayOS);
					motorLeft(angle-6);
					osDelay(delayOS);
					leftWallCheckUntilDetected();
					/*
					rightWallCheck();
					osDelay(delayOS);
					motorLeft(angle-6);
					osDelay(delayOS);
					rightWallCheck();
					osDelay(delayOS);
					*/


				HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
						}
		else if (strncmp(motorDir, "LL", 2) == 0){
					total_angle=0;
					angle=90;
					Aint = 0;
					gyroInit();
					osDelay(delayOS);
					motorLeft(angle-6); //LEFT 90
					osDelay(delayOS);
					rightWallCheck();
					osDelay(delayOS);
					motorRight(172);
					osDelay(delayOS);
					rightWallCheck();
					osDelay(delayOS);

					motorRight(angle-8);
					osDelay(delayOS);
					rightWallCheckUntilDetected();
					/*
					rightWallCheckUntilDetected(void)


					leftWallCheck();
					osDelay(delayOS);
					motorRight(angle-8);
					osDelay(delayOS);
					leftWallCheck(); //align with 2nd box
					osDelay(delayOS);
					*/



				HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
						}
		else if (strncmp(motorDir, "RS", 2) == 0){
					//moveBackward("Right", angle);
			printf("RS reached");
			total_angle=0;
						gyroInit();
						osDelay(10);
												//printf("YOLO");
						htim1.Instance->CCR4 = 150;
						osDelay(delayOS);					//osDelay(1000);
						HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);

				}

		/*else if (strncmp(motorDir, "ST", 2) == 0){ // i still need to edit this to stop servos
			moveBackward("Right", angle);
										HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
										stopBot();

										break;
		}*/\
		/*
		switch(motorDir){
			case 20:
				motorRight(angle);
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;
			case 'ST':
				stopMovement();
				break;
			case 1:

				//move forward
				HAL_GPIO_WritePin(GPIOA, MotorA_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, MotorA_IN2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, MotorB_IN2_Pin, GPIO_PIN_RESET);

				// forward left and right wheels CLEARED CHECKLIST @ left: 2650; right: 2000
				//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1800); //left wheel was 1650
				//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000); //right wheel was 1400


				Aint = 0;

				gyroInit();
				osDelay(1000);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1800); //left wheel was 1650
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2000); //right wheel was 1400

				turnDegree = 0;
				moveForward("Straight", angle);
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;
			case 2:

				Aint = 0;
				gyroInit();
				osDelay(1000);

				//motorLeft(angle);

				motorLeft(10);
				osDelay(1);
				moveBackward("Straight", 10);
				osDelay(1);
				motorLeft(40);
				moveBackward("Straight", 10);

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;

			case 3: // turn right

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorRight(angle);

				//motorRight(55);
				osDelay(1);
				//moveBackward("Straight", 20);
				//osDelay(1);
				motorRight(10);
				moveBackward("Straight", 10);
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;

			case 4: // go fwd, stop 20cm from obstacle

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorForward(1300); // 1500 too fast; 13th Oct 6pm
				while(Distance > 20){
					correctDirection(0, 1);
					osDelay(1);
				}
				stopMovement();


				motorReverse(1000);
				while(Distance < 20){
					osDelay(1);
				}
				stopMovement();
				motorDir = 0;
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 5: // go around the first obstacle from the left side

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorLeft(45);
				total_angle = 0;
				moveForward("Straight", 15);

				motorRight(45);
				total_angle = 0;

				motorRight(45);
				total_angle = 0;

				moveForward("Straight", 15);

				motorLeft(45);
				total_angle = 0;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 6: // go around the first obstacle from the right side

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorRight(45);
				total_angle = 0;
				moveForward("Straight", 15);

				motorLeft(45);
				total_angle = 0;

				//moveForward("Straight", 10);

				motorLeft(45);
				total_angle = 0;
				moveForward("Straight", 15);

				motorRight(45);
				total_angle = 0;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 7: // turn left around the second obstacle and move to edge

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorLeft(90);
				total_angle = 0;

				motorForward(1000);
				// adjust distance value using formula a*1/IR + b
				//while(37411*(1/(double)rightIR[0]) - 8.57 < 60){
				//	correctDirection(0, 1);
				//	osDelay(1);
				//}
				stopMovement();

				motorRight(90);
				total_angle = 0;

				moveForward("Straight", 10);

				motorRight(90);
				total_angle = 0;

				motorForward(1000);
				//while(37411*(1/(double)rightIR[0]) - 8.57 < 60){
				//	osDelay(1);
				//}
				stopMovement();

				moveForward("Straight", 10);

				motorRight(97);
				total_angle = 0;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 8: // turn right around the second obstacle and move to edge

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorRight(90);
				motorForward(1000);
				while(41826*(1/(double)leftIR[0]) - 10.03 < 60){
					correctDirection(0, 1);
					osDelay(1);
				}

				stopMovement();
				motorLeft(90);
				total_angle = 0;

				moveForward("Straight", 10);

				motorLeft(88); // 90 at 14 oct 235am; too much angle
				total_angle = 0;


				motorForward(1000);
				while(41826*(1/(double)leftIR[0]) - 10.03 < 60){
					correctDirection(0, 1);
					osDelay(1);
				}
				stopMovement();

				moveForward("Straight", 10);

				motorLeft(90);
				total_angle = 0;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 9: // go fwd until detecting 1st obstacle on the right

				Aint = 0;
				gyroInit();
				osDelay(1000);

				moveForward("Straight", 30);

				motorForward(1000);
				while(37411*(1/(double)rightIR[0]) - 8.57 > 60){
					correctDirection(0, 1);
					osDelay(1);
				}
				stopMovement();

				double rightDist = 37411*(1/(double)rightIR[0]) - 8.57;

				moveForward("Straight", 25);

				motorRight(90);
				total_angle = 0;

				motorForward(rightDist);

				motorLeft(90);
				total_angle = 0;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 10: // go fwd until detecting 1st obstacle on the left

				Aint = 0;
				gyroInit();
				osDelay(1000);

				moveForward("Straight", 30);

				motorForward(1000);
				while(41826*(1/(double)leftIR[0]) - 10.03 > 60){
					correctDirection(0, 1);
					osDelay(1);
				}
				stopMovement();

				double leftDist = 41826*(1/(double)leftIR[0]) - 10.03;

				moveForward("Straight", 20);

				motorLeft(90);
				total_angle = 0;

				motorForward(leftDist);

				motorRight(92);
				total_angle = 0;

				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;

			case 11:
				Aint = 0;
				gyroInit();
				osDelay(1000);
				moveBackward("Straight", angle);
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;
			case 12:
				moveBackward("Left", angle);
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;
			case 13:
				moveBackward("Right", angle);
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);
				break;

			case 14:

				Aint = 0;
				gyroInit();
				osDelay(1000);

				motorForward(1300); // 1500 too fast; 13th Oct 6pm
				while(Distance > 20){
					correctDirection(0, 1);
					osDelay(1);
				}
				stopMovement();

				motorDir = 0;
				HAL_UART_Transmit(&huart3, (uint8_t *) "Done", 5, 0xFFFF);

				break;


			default:
				stopMovement();
				HAL_UART_Transmit(&huart3, (uint8_t *) "Err", 5, 0xFFFF);
				break;
		}
	*/
		strncpy(motorDir, replacer, 2);
			    motorDir[2] = '\0';
		osDelay(1);
	}
  /* USER CODE END robotCommand */
}

/* USER CODE BEGIN Header_leftEncoder */
/**
* @brief Function implementing the leftEncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_leftEncoder */
void leftEncoder(void const * argument)
{
  /* USER CODE BEGIN leftEncoder */
	//OLED_Refresh_Gram();
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

	int cntL1, cntL2;

	int cnt2;
	int dirL = 1;
	int diff;
	uint32_t tick;
	cntL1 = __HAL_TIM_GET_COUNTER(&htim2);

	tick = HAL_GetTick();
	//uint8_t left[30] = "Test \0";
	/* Infinite loop */
//	float speed = 0;
//	int delta_count = 0;
	for(;;)
	{
		if(HAL_GetTick()-tick > 100L)
		{

			cnt2 = __HAL_TIM_GET_COUNTER(&htim2);

			if(cnt2 > 32000){
				  		  	  			 dirL = 1;

				  		  	  			  diff = (65536 - cnt2);


				  		  	  			  } else {
				  		  	  				  dirL = -1;




				  		  	  			  	 diff = cnt2;

				  		  	  		  }




			                          if(dirL == 1){
				  		  			  totalLeftEncoder += diff;
			                          } else {
			                        	  totalLeftEncoder -= diff;
			                          }

			                          __HAL_TIM_SET_COUNTER(&htim2, 0);

			                     tick = HAL_GetTick();
//		  cntL2 = __HAL_TIM_GET_COUNTER(&htim2);
//
//		  // for left encoder
//		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
//		  {
//			  if(cntL2<cntL1)
//			  {
//				  leftEncoderVal = cntL1 - cntL2;
//			  }
//			  else
//				  leftEncoderVal = (65535-cntL2) + cntL1;
//
//		  }
//		  else
//		  {
//			  if(cntL2>cntL1)
//			  {
//				  leftEncoderVal = cntL2 - cntL1;
//			  }
//			  elsetotalLeftEncoder / 1550.0 * -21.04 < distance)
//				  lefptEncoderVal = (65535-cntL1) + cntL2;
//		  }
//
////		  leftPWMval = Left_PID_control (1500, leftEncoderVal);
//		  totalLeftEncoder += leftEncoderVal;
//
//		  cntL1 = __HAL_TIM_GET_COUNTER(&htim2);
//		  tick = HAL_GetTick();
		}
		osDelay(1);
	}
  /* USER CODE END leftEncoder */
}

/* USER CODE BEGIN Header_rightEncoder */
/**
* @brief Function implementing the rightEncoderTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rightEncoder */
void rightEncoder(void const * argument)
{
  /* USER CODE BEGIN rightEncoder */
	//OLED_Refresh_Gram();

	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

	int cntR1, cntR2;
	int dirR = 1;
	int diff;
	uint32_t tick;
	cntR1 = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
	//uint8_t right[30] = "Test\0";
	/* Infinite loop */
	for(;;)
	{
	  if(HAL_GetTick()-tick > 100L)
	  {
		  cntR2 = __HAL_TIM_GET_COUNTER(&htim3);

		  // for left encoder
		  //if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
		  //{
			  if(cntR2<32000)
			  {
				  //rightEncoderVal = cntR1 - cntR2;
				  dirR = 1;
				  diff = (65536 - cntR2);
			  }
			  else
				  dirR = -1;
			  diff = cntR2;

			  if(dirR ==1){
				  totalRightEncoder += diff;
			  } else{
				  totalRightEncoder -=diff;
			  }
			  __HAL_TIM_SET_COUNTER(&htim3, 0);
		  //}

		  //rightPWMval = Right_PID_control (1500, rightEncoderVal);

		  //cntR1 = __HAL_TIM_GET_COUNTER(&htim3);
		  tick = HAL_GetTick();
	  }
	  osDelay(1);
	}
  /* USER CODE END rightEncoder */
}

/* USER CODE BEGIN Header_stopBot */
/**
* @brief Function implementing the stopBotTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stopBot */
void stopBot(void const * argument)
{
  /* USER CODE BEGIN stopBot */
  /* Infinite loop */
  for(;;)
  {
    if(motorDir == 0)
    	stopMovement();
  }
  /* USER CODE END stopBot */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the motorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void const * argument)
{
  /* USER CODE BEGIN motor */
  /* Infinite loop */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  for(;;)
  {
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, rightPWMval);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, leftPWMval);
    osDelay(1);
  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_OLEDShow */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLEDShow */
void OLEDShow(void const * argument)
{
  /* USER CODE BEGIN OLEDShow */
  /* Infinite loop */
  for(;;)
  {
	  int encoderdis =(totalLeftEncoder) / 1550.0 * -21.04;
	  int encoderright=(totalRightEncoder)/1550.0 * 21.04;
	  uint8_t oledText[70] = "oled \0";
	  //sprintf(oledText,"Right is %5d, %5d \n Left is %5d, %5d \0",rightEncoderVal, rightPWMval, leftEncoderVal, leftPWMval);
	  sprintf(oledText,"angle: %5d", (int)total_angle);
	  OLED_ShowString(10,20, oledText);
	  sprintf(oledText, "distance: %5d", (int)Distance);
	  OLED_ShowString(10,30, oledText);
	  sprintf(oledText, "IR %4d %4d", (int)leftIR[0], (int)rightIR[0]);
	  //sprintf(oledText, "Left  %3d", (int)encoderdis);
	  OLED_ShowString(10,40, oledText);
	  sprintf(oledText, "UART: %.4s", aRxBuffer);
	  //sprintf(oledText, "Right  %3d", (int)encoderright);
	  OLED_ShowString(10,50, oledText);

	  OLED_Refresh_Gram();
	  osDelay(500);
  }
  /* USER CODE END OLEDShow */
}

/* USER CODE BEGIN Header_gyroTask1 */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyroTask1 */
void gyroTask1(void const * argument)
{
  /* USER CODE BEGIN gyroTask1 */
  /* Infinite loop */
	uint8_t val[2] = {0,0};
		double alpha = 0.9;
		char hello[20];
		int16_t angular_speed = 0;

		uint32_t tick = 0;
		gyroInit();
		int dir;
		int16_t offset = 0;

		tick = HAL_GetTick();
		osDelay(1);

	//	for(int i = 0; i < 3000; i++){
	//		readByte(0x37, val);
	//		angular_speed = val[0] << 8 | val[1];
	//		offset += angular_speed;
	//		//osDelayUntil(1);
	//	}
	//	offset = offset/3000.0f;

	  for(;;)
	  {
	      osDelay(1);

	      if(HAL_GetTick() - tick >= 100){
		  readByte(0x37, val);
		  //osDelayUntil(1);
		  //angular_speed = ((int16_t)((int8_t)val[0]))*256 +  (uint16_t)val[1];
		  angular_speed = (val[0] << 8) | val[1];


		  // for gyro drift removal, calibration needs to be added to angular speed
		  //total_angle +=(double)(angular_speed - 1.7)*((HAL_GetTick() - tick)/16400.0);
		  // Calculate new (unfiltered) angle increment
		 double alpha = 1.0;
		 double new_angle_increment = (double)(angular_speed - 1.7) * ((HAL_GetTick() - tick) / 16400.0);

//		              // Apply the low-pass filter to the new angle increment
		              double filtered_increment = applyLowPassFilter(0, new_angle_increment, alpha); // Initialize previous value as 0 for the first run

		              // Update total angle with the filtered increment
		              total_angle += filtered_increment;
		  //prevSpeed = angular_speed;
		  if(total_angle >= 720){
			  total_angle = 0;
		  }
		  if(total_angle <= -720){
			  total_angle = 0;
		  }


		  tick = HAL_GetTick();
	      }
	  }
  /* USER CODE END gyroTask1 */
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
