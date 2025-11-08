/*
 * CMR_CANFD.h
 *
 *  Created on: Jan 28, 2025
 *      Author: David Bascom
 */

#ifndef INC_CMR_CANFD_H_
#define INC_CMR_CANFD_H_

/* Includes */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

/* Initialization */

/**
  * @brief  Enables CMR_CANFD to desired mode and activates filters and interrupts.
  * use rxhandler0 ????
  * @param  baddr representing the board address
  * @retval integer( 1 if passes, -1 if fails)
  */

int cmr_fdcan_init1();


HAL_StatusTypeDef cmr_filter_init(FDCAN_FilterTypeDef* filter, uint32_t fdcanid);

/**
  * @brief  Transmits CANFD message
  * @param  StdID: The ID of the message. This must match the ID of the board you are sending to.
  *
  *
  * @param data: Data array
  *
  * @param len: length of data array (maximum of 64 bytes)
  *
  * @retval integer( 1 if passes, -1 if fails)
  */

int transmit_message(uint32_t ID, uint8_t* data, uint32_t frameType, uint32_t len, uint32_t error, uint32_t bitRateSwitch);

/**
  * @brief  uses the message to do whatever the board needs to do
  * @param  StdID: ID of the CANFD message that was sent to you. This should match your board ID already.
  * Probably you will not need to do anything with the StdID
  *
  * @param data: data array
  *
  *
  * @retval void
  */

extern void rxHandler0(uint32_t ID, uint64_t* data);



#endif /* INC_CMR_CANFD_H_ */
