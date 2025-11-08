/*
 * CMR_CANFD.c
 *
 *  Created on: Jan 28, 2025
 *      Author: David Bascom
 */

/* Includes */

#include "CMR_CANFD.h"
#include "main.h"


/* Private Variables*/

extern FDCAN_HandleTypeDef   hfdcan1;

FDCAN_FilterTypeDef   filter0;

FDCAN_RxHeaderTypeDef RxHeader;

FDCAN_TxHeaderTypeDef TxHeader;

uint64_t  RxData[64];



/* CANFD functions -------------------------------------------------------------*/
int cmr_fdcan_init1(uint32_t ID){

  HAL_FDCAN_MspInit(&hfdcan1);

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 29;
  hfdcan1.Init.NominalTimeSeg2 = 10;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK){
	  return -1;
  }

  if (cmr_filter_init(&filter0, ID) != HAL_OK){
	  return -1;
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
	  return -1;
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
	  return -1;
  }

  return 1;
}


HAL_StatusTypeDef cmr_filter_init(FDCAN_FilterTypeDef* filter, uint32_t fdcanid) {
    // Configure the filter properties for FDCAN
	filter->IdType = FDCAN_STANDARD_ID;
	filter->FilterIndex = 0;
	filter->FilterType = FDCAN_FILTER_MASK;
	filter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter->FilterID1 = fdcanid;
	filter->FilterID2 = 0x7FF;

    return HAL_FDCAN_ConfigFilter(&hfdcan1, filter);
}


int transmit_message(uint32_t ID, uint8_t* data, uint32_t frameType, uint32_t len, uint32_t error, uint32_t bitRateSwitch){

	// Deactivate notifications while the transmit is occurring
//	HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

	// CANFD header formation
	TxHeader.Identifier = ID;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = frameType;
	TxHeader.DataLength = len;
	TxHeader.ErrorStateIndicator = error;
//	TxHeader.BitRateSwitch = bitRateSwitch;
	TxHeader.BitRateSwitch  = FDCAN_BRS_OFF;
//	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.FDFormat       = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	// Send the message
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK) {
		return -1;
	}

	// Activate notifications again
//	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	return 1;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
//        HAL_GPIO_TogglePin(GPIOA, Debug_LED1_Pin); // Confirm interrupt is alive

//        HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

        FDCAN_RxHeaderTypeDef RxHeader_t;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader_t, RxData);
        rxHandler0(RxHeader_t.Identifier, RxData);

//        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}



/**
  * @brief This function handles CAN1 RX0 interrupt.
  */

void FDCAN1_IT0_IRQHandler(void)
{
  HAL_FDCAN_IRQHandler(&hfdcan1);

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hfdcan->Instance==FDCAN1)
  {

  /* Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /*FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  }

}


