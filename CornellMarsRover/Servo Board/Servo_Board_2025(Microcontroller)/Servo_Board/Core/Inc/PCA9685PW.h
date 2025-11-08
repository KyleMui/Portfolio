/*
 * PCA9685PW.h
 *
 *  Created on: Dec 26, 2024
 *      Author: Kyle Mui
 */

#ifndef PCA9685_I2C_DRIVER_H
#define PCA9685_I2C_DRIVER_H

#include "stm32g4xx_hal.h" /* Needed for I2C */

/* DEFINES */
#define PCA9685_I2C_ADDR		(0xAA)		/* Programmable, default address for the breakout board */

/* REGISTERS (p. 10-13) */
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

/* STRUCT */
typedef struct {

	/* I2C HANDLE */
	I2C_HandleTypeDef *i2cHandle;
	uint16_t frequency;


} PCA9685;

typedef struct {

	uint8_t Channel;
	uint8_t deg_range;
	uint16_t home_flag;
	uint16_t home;
} Servo;

/* INITIALIZATION */
uint8_t PCA9685_Init(PCA9685 *dev, I2C_HandleTypeDef *i2cHandle);
void Servo_Init(Servo *dev, uint8_t Channel, uint8_t deg, uint16_t home_flag, uint16_t home);

/* LOW-LEVAL FUNCTIONS */
HAL_StatusTypeDef PCA9685_ReadRegister(uint8_t Register, uint8_t *data);
HAL_StatusTypeDef PCA9685_ReadRegisters(uint8_t Register, uint8_t *data, uint8_t Length);
HAL_StatusTypeDef PCA9685_WriteRegister(uint8_t Register, uint8_t *data);
HAL_StatusTypeDef PCA9685_WriteRegisters(uint8_t Register, uint8_t *data, uint8_t Length);

HAL_StatusTypeDef PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value);
void PCA9685_SetPWMFrequency(uint16_t frequency);

HAL_StatusTypeDef PCA9685_SetPWM(Servo* dev, uint16_t OnTime, uint16_t OffTime);
HAL_StatusTypeDef PCA9685_SetServoAngle(Servo* dev, float Angle);
float PCA9685_GetServoAngle(Servo* dev);

void velocity_control(Servo *servo, uint8_t control_data);

#endif /* PCA9685_I2C_DRIVER_H */
