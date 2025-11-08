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
#include "PCA9685PW.h"
#include "CMR_CANFD.h"
#include "CMR_Servo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern int transmit_message(uint32_t ID, uint8_t* data, uint32_t frameType, uint32_t len, uint32_t error, uint32_t bitRateSwitch);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
FDCAN_HandleTypeDef hfdcan1;

uint32_t fdcanid = 0x1;  //change this to set board canid
//	uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
//	uint8_t data2[] = {0x05, 0x06, 0x07, 0x08};
FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

uint32_t fdcanid = 0x1;  //change this to set board canid
	uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
	uint8_t data2[] = {0x05, 0x06, 0x07, 0x08};


PCA9685 pca9685;
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;
Servo servo9;
Servo servo10;
Servo servo11;
Servo servo12;
Servo servo13;
Servo servo14;
Servo servo15;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int configFlag = 0;
int controlFlag = 0;
int feedbackFlag = 0;
int diagnosticFlag = 0;
uint8_t rxData[32];


void rxHandler0(uint32_t ID, uint64_t* data) {
    memcpy(rxData, data, 32);

    uint8_t msg_type = (rxData[0] >> 6) & 0x03;
    switch (msg_type) {
        case 0: configFlag = 1;     break;
        case 1: controlFlag = 1;    break;
        case 2: feedbackFlag = 1;   break;
        case 3: diagnosticFlag = 1; break;
    }
}

int rxFlag = 0;
int rxFlag2 = 0;

void rxHandler0(uint32_t ID, uint8_t* data) {
    if (memcmp(data, (uint8_t[]){0x1, 0x2, 0x3, 0x4}, (unsigned int)4) == 0) {
        rxFlag = 1;
    }
    if (memcmp(data, (uint8_t[]){0x5, 0x6, 0x7, 0x8}, (unsigned int)4) == 0) {
        rxFlag2 = 1;
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
	HAL_FDCAN_Start(&hfdcan1);


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  cmr_fdcan_init1();

  PCA9685_Init(&pca9685, &hi2c1);
  Servo_Init(&servo0, 0, 180, 1, 0);
  Servo_Init(&servo1, 1, 180, 1, 0);
  Servo_Init(&servo2, 2, 180, 1, 0);
  Servo_Init(&servo3, 3, 180, 1, 0);
  Servo_Init(&servo4, 4, 180, 1, 0);
  Servo_Init(&servo5, 5, 180, 1, 0);


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  cmr_fdcan_init1(fdcanid);

  PCA9685_Init(&pca9685, &hi2c1);
  Servo_Init(&servo0,  0, 210, 1, 0);
  Servo_Init(&servo1,  1, 180, 1, 0);
  Servo_Init(&servo2,  2, 180, 1, 0);
  Servo_Init(&servo3,  3, 180, 1, 0);
  Servo_Init(&servo4,  4, 180, 1, 0);
  Servo_Init(&servo5,  5, 180, 1, 0);
  Servo_Init(&servo6,  6, 180, 1, 0);
  Servo_Init(&servo7,  7, 180, 1, 0);
  Servo_Init(&servo8,  8, 180, 1, 0);
  Servo_Init(&servo9,  9, 180, 1, 0);
  Servo_Init(&servo10, 10, 180, 1, 0);
  Servo_Init(&servo11, 11, 180, 1, 0);
  Servo_Init(&servo12, 12, 180, 1, 0);
  Servo_Init(&servo13, 13, 180, 1, 0);
  Servo_Init(&servo14, 14, 180, 1, 0);
  Servo_Init(&servo15, 15, 180, 1, 0);


/*Test Cases for Control Message

 ControlMsg msg = {
     .msg_type      = 1,
     .servo_id      = 0,
     .home          = 0,
     .reset_home    = 1,
     .control_mode  = 0,
     .control_data  = 30,
     .clear_faults  = 0,
     .query_data    = 0,
     .reserved      = 0
 };

 Run_Ctrl_Msg(msg);

 HAL_Delay(2000);

 ControlMsg testMsg = {
	.msg_type = 1,
	.servo_id = 0,
	.home = 0,
	.reset_home = 0,
	.control_mode = 0,       
	.control_data = 0,      
	.clear_faults = 0,
	.query_data = 0,
	.reserved = 0
 };

 Run_Ctrl_Msg(testMsg);
 HAL_Delay(2000);

 ControlMsg go_msg = {
     .msg_type      = 1,
     .servo_id      = 0,
     .home          = 1,   
     .reset_home    = 0,
     .control_mode  = 0,
     .control_data  = 0,
     .clear_faults  = 0,
     .query_data    = 0,
     .reserved      = 0
 };
 Run_Ctrl_Msg(go_msg);
 HAL_Delay(2000);
 */

  //MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(&pca9685, &hi2c1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(500);
	  if (configFlag) {
	      configFlag = 0;
	      ConfigMsg msg;
	      Config_Msg_Init(&msg, rxData);
//	      HAL_GPIO_TogglePin(GPIOA, Debug_LED1_Pin);

	  } else if (controlFlag) {
	      controlFlag = 0;
	      ControlMsg msg;
	      Control_Msg_Init(&msg, rxData);
	      Run_Ctrl_Msg(msg);

	      uint8_t ack[32] = {0xAC};  // 0xAC = Acknowledge Code (arbitrary)
	      transmit_message(0x101, ack, FDCAN_DATA_FRAME, 32, 0, 1);
   	      HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_SET);

	  }
//	  else if (feedbackFlag) {
//	      feedbackFlag = 0;
//	      FeedbackMsg msg;
//	      Feedback_Msg_Init(&msg, rxData);
//	      HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_SET);
//	  }
//	    else if (diagnosticFlag) {
//	      diagnosticFlag = 0;
//	      DiagnosticMsg msg;
//	      Diagnostic_Msg_Init(&msg, rxData);
//	      HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_SET);
//	  }
  	  else {
//	      HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_RESET);
	  }

/*Set Servo Angles Directly
	  PCA9685_SetServoAngle(&servo0, 180);
	  PCA9685_SetServoAngle(&servo1, 180);
	  PCA9685_SetServoAngle(&servo2, 180);
	  PCA9685_SetServoAngle(&servo3, 180);
	  PCA9685_SetServoAngle(&servo4, 180);
	  PCA9685_SetServoAngle(&servo5, 180);
	  HAL_Delay(2000);
	  PCA9685_SetServoAngle(&servo0, 100);
	  PCA9685_SetServoAngle(&servo1, 0);
	  PCA9685_SetServoAngle(&servo2, 0);
	  PCA9685_SetServoAngle(&servo3, 0);
	  PCA9685_SetServoAngle(&servo4, 0);
	  PCA9685_SetServoAngle(&servo5, 0);
	  HAL_Delay(2000);

	  PCA9685_SetServoAngle(&servo, 0);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 10);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 20);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 30);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 40);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 50);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 60);
	  HAL_Delay(100);
	  PCA9685_SetServoAngle(&servo, 70);
	  HAL_Delay(100);
*/

/* Test CAN RX Handler
	  //HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_SET);
	  if (rxFlag) {
		  rxFlag = 0;  // reset it
		  HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_SET);
		  HAL_Delay(500);

	  } else if (rxFlag2){
		  rxFlag2 = 0;
		  HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin, GPIO_PIN_RESET);
		  HAL_Delay(500);
	  }

	  transmit_message(0x1, data, FDCAN_DATA_FRAME, sizeof(data), 0, 1);
	  HAL_Delay(500);
	  transmit_message(0x1, data2, FDCAN_DATA_FRAME, sizeof(data), 0, 1);
	  HAL_Delay(500);
	  */




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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 37;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 4;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c1.Init.Timing = 0x00D09BE3;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Debug_LED1_Pin|OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRST_Pin */
  GPIO_InitStruct.Pin = NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Debug_LED1_Pin OE_Pin */
  GPIO_InitStruct.Pin = Debug_LED1_Pin|OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_Pin */
  GPIO_InitStruct.Pin = BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
