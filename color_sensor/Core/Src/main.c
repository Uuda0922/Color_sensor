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
#include "DEV_Config.h"
#include "TCS34725.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define BUFFER_SIZE 8
#define CAN1_DEVICE_NUM     4
#define FIRST_GROUP_ID      0x205
#define CAN_DATA_SIZE       8
#define CAN1_RX_ID_START    0x200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint16_t Accel_X_Raw;
uint16_t Accel_Y_Raw;
uint16_t Accel_Z_Raw;
float Ax;
float Ay;
float Az;
int16_t Gyro_X_Raw;
int16_t Gyro_Y_Raw;
int16_t Gyro_Z_Raw;
float Gx;
float Gy;
float Gz;
float angle_deg;
int8_t msg1;
int8_t msg2;
int8_t msg3;
int8_t msg4;
int8_t msg5;
int8_t msg6;
uint8_t buffer[BUFFER_SIZE];

//#define KALMAN_FILTER_ON
//// +++ kalman filter ++++++++++++++++++++++
//#ifdef KALMAN_FILTER_ON
//  double kf_A = 1;            // State transition matrix
//  double kf_H = 1;            // Measurement matrix
//  double kf_Q = 0.1;         // Process noise covariance
//  double kf_R = 1;         // Measurement noise covariance
//  double kf_X = 0;            // Initial state estimate
//  double kf_P = 1;            // Initial state covariance
//#endif

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void can_transmit(CAN_HandleTypeDef* hcan1, uint16_t id, int8_t msg1, int8_t msg2, int8_t msg3);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void can_transmit(CAN_HandleTypeDef* hcan, uint16_t id, int8_t msg1, int8_t msg2, int8_t msg3){
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             buffer[8];
    uint32_t            pTxMailbox;

    tx_header.StdId = id;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = CAN_DATA_SIZE;
    tx_header.TransmitGlobalTime = DISABLE;
    buffer[0] = msg1;
    buffer[1] = msg2;
    buffer[3] = msg3;
//    buffer[4] = msg4;
//    buffer[5] = msg5;
//    buffer[6] = msg6;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, buffer, &pTxMailbox) == HAL_OK){
        while (HAL_CAN_IsTxMessagePending(hcan, pTxMailbox));
    }

}


void MPU6050_Init (void)
{
	uint8_t check, Data;
	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, WHO_AM_I_REG,1,&check,1,1000);

	if( check == 104)
	{
		Data=0;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		Data=0x07;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		Data=0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		Data=0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}
	void MPU6050_Read_Accel(void)
	{
		uint8_t Rec_Data[6];
		HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Accel_X_Raw=(int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Accel_Y_Raw=(int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Accel_Z_Raw=(int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

		Ax=Accel_X_Raw/4096.0;
		Ay=Accel_Y_Raw/4096.0;
		Az=Accel_Z_Raw/4096.0;
	}

	void MPU6050_Read_Gyro(void)
	{
		uint8_t Rec_Data[6];
		HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Gyro_X_Raw=(int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Gyro_Y_Raw=(int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Gyro_Z_Raw=(int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

		Gx=Gyro_X_Raw/131.0;
		Gy=Gyro_Y_Raw/131.0;
		Gz=Gyro_Z_Raw/131.0;
	}

	float Accel_X_Angle(float Ax, float Ay, float Az) {
//	    float angle_rad = atan2(Ay/ (sqrt(Az * Az + Ax * Ax)));
//	    float angle_deg = angle_rad * 180.0 / M_PI;
		float angle_deg = (180/3.141592)*(atan(Az/Ax));
		return angle_deg;
	}
	float Accel_Y_Angle(float Ax, float Ay, float Az) {
//	    float angle_rad = atan2(Ax, sqrt(Az * Az + Ay * Ay));
//	    float angle_deg = angle_rad * 180.0 / M_PI;
		float angle_deg = ((180/3.141592) * atan(Ay / sqrt(pow(Ax, 2) + pow(Az, 2))))*100;
			    return angle_deg;
	}

	float Accel_Z_Angle(float Ax, float Ay, float Az) {
//	    float angle_rad = atan2(sqrt(Ax * Ax + Ay * Ay), Az);
//	    float angle_deg = angle_rad * 180.0 / M_PI;
		float angle_deg = (180/3.141592) * atan(sqrt((Ay) + (Ax)) / Az);

	    return angle_deg;
	}
//	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim12) {
//
////	    	RGB rgb;
////	    	MPU6050_Read_Accel();
////	    		  	MPU6050_Read_Gyro();
//	    	//		rgb=TCS34725_Get_RGBData();
//	    	//		RGB888=TCS34725_GetRGB888(rgb);
//
////	    			float angle_X = Accel_X_Angle(Ax, Ay, Az);
////	    			float angle_Y = Accel_Y_Angle(Ax, Ay, Az);
////	    			float angle_Z = Accel_Z_Angle(Ax, Ay, Az);
////
////	    			buffer[0]= angle_X;
////	    			buffer[1]= angle_Y;
////	    			buffer[2]= angle_Z;
//
////			buffer[3]=RGB888>>16 ;
////			buffer[4]=(RGB888>>8) & 0xff;
////			buffer[5]=(RGB888) & 0xff;
//			 can_transmit(&hcan1, 0x190, msg1, msg2, msg3);
//
//	    }



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	RGB rgb;
//	UDOUBLE RGB888=0;
//	UWORD RGB565=0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_CAN1_Init();
  MX_I2C2_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MPU6050_Init();
  HAL_TIM_Base_Start_IT(&htim12);
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	if(TCS34725_Init() != 0){
      printf("TCS34725 initialization error!!\n");
  }
	else{
  printf("TCS34725 initialization success!!\n");
	}
	uint8_t data[] = "HELLO WORLD \r\n";

	HAL_UART_Transmit (&huart1, data, sizeof (data), 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	MPU6050_Read_Accel();
//	  	MPU6050_Read_Gyro();
//		rgb=TCS34725_Get_RGBData();
//		RGB888=TCS34725_GetRGB888(rgb);

		float angle_X = Accel_X_Angle(Ax, Ay, Az);
		float angle_Y = Accel_Y_Angle(Ax, Ay, Az);
		float angle_Z = Accel_Z_Angle(Ax, Ay, Az);

		buffer[0]= angle_X;
		buffer[1]= angle_Y;
		buffer[2]= angle_Z;

//		HAL_TIM_Base_Start_IT(&htim12);
//		buffer[3]=RGB888>>16 ;
//		buffer[4]=(RGB888>>8) & 0xff;
//		buffer[5]=(RGB888) & 0xff;

		//=== kalmant filter  ====
//		    #ifdef KALMAN_FILTER_ON
//		      // prediction
//		      double x_pred = kf_A * kf_X;
//		      double P_pred = kf_A * kf_P * kf_A + kf_Q;
//		      // update
//		      double K = (P_pred * kf_H)/(kf_H * P_pred * kf_H + kf_R);
//		      kf_X = x_pred + K*( Ax - kf_H*x_pred);
//		      kf_P = (1 - K * kf_H)*P_pred;
//		      // -----------------------
//		      x = kf_X*100;
//		    #endif
//		can_transmit(&hcan1, 0x205, angle_X, msg2, msg3);
//		HAL_Delay(200);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
   //S.ID=0x191;
   sFilterConfig.FilterBank = 0;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = 0x205;
   sFilterConfig.FilterIdLow = 0x205;
   sFilterConfig.FilterMaskIdHigh = 0x0000;
   sFilterConfig.FilterMaskIdLow = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 14;
   if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
     	Error_Handler();
   }

   if (HAL_CAN_Start(&hcan1) != HAL_OK) {
     	Error_Handler();
   }

   if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
     	Error_Handler();
   }
  /* USER CODE END CAN1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 64000-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 2-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : PC8 */
//  GPIO_InitStruct.Pin = GPIO_PIN_8;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : INT_PIN_Pin */
//  GPIO_InitStruct.Pin = INT_PIN_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(INT_PIN_GPIO_Port, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}

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
