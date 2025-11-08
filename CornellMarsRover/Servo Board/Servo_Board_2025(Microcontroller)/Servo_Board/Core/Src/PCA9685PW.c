/*
 * PCA9685PW.c
 *
 *  Created on: Dec 26, 2024
 *      Author: Kyle Mui
 */

#include "PCA9685PW.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t PCA9685_Init(PCA9685 *dev, I2C_HandleTypeDef *i2cHandle) {

	/* Set up struct params */
	dev->i2cHandle		=i2cHandle;
	dev->frequency      =50;


	/* Store number of transaction errors (to be returned at the end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	PCA9685_SetPWMFrequency(dev->frequency); // 50 Hz for servo
	PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}


void Servo_Init(Servo *dev, uint8_t Channel, uint8_t deg, uint16_t home_flag, uint16_t home) {

	dev->Channel		=Channel;
	dev->deg_range		=deg;
	dev->home_flag		=home_flag;
	if (home_flag == 1){
		dev->home		=home;
	}
}


/* LOW-LEVEL FUNCTIONS */

HAL_StatusTypeDef PCA9685_ReadRegister(uint8_t Register, uint8_t *data) {
	return HAL_I2C_Mem_Read(&hi2c1,PCA9685_I2C_ADDR, Register, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


HAL_StatusTypeDef PCA9685_ReadRegisters(uint8_t Register, uint8_t *data, uint8_t Length) {
	return HAL_I2C_Mem_Read(&hi2c1,PCA9685_I2C_ADDR, Register, I2C_MEMADD_SIZE_8BIT, data, Length, HAL_MAX_DELAY);
}


HAL_StatusTypeDef PCA9685_WriteRegister(uint8_t Register, uint8_t *data) {
	return HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDR, Register, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


HAL_StatusTypeDef PCA9685_WriteRegisters(uint8_t Register, uint8_t *data, uint8_t Length) {
	return HAL_I2C_Mem_Write(&hi2c1, PCA9685_I2C_ADDR, Register, I2C_MEMADD_SIZE_8BIT, data, Length, HAL_MAX_DELAY);
}


// Read all 8 bits and set only one bit to 0/1 (Value) and write all 8 bits back
HAL_StatusTypeDef PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value) {
  uint8_t readValue;
  if (PCA9685_ReadRegister(Register, &readValue) != HAL_OK) return HAL_ERROR;

  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);

  if (PCA9685_WriteRegister(Register, &readValue) != HAL_OK) return HAL_ERROR;
  HAL_Delay(1);
  return HAL_OK;
}



void PCA9685_SetPWMFrequency(uint16_t frequency) {
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;      //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency); // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  PCA9685_WriteRegister(PCA9685_PRE_SCALE, &prescale);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}


HAL_StatusTypeDef PCA9685_SetPWM(Servo* dev, uint16_t OnTime, uint16_t OffTime) {
  uint8_t registerAddress = PCA9685_LED0_ON_L + (4 * dev->Channel);
  uint8_t pwm[4] = {
    OnTime & 0xFF,
    OnTime >> 8,
    OffTime & 0xFF,
    OffTime >> 8
  };

  return PCA9685_WriteRegisters(registerAddress, pwm, 4);
}



HAL_StatusTypeDef PCA9685_SetServoAngle(Servo* dev, float Angle) {
  float Value = (Angle * (511.9f - 102.4f) / dev->deg_range) + 102.4f;

  // Clamp value to valid range (optional safety)
  if (Value < 102.4f) Value = 102.4f;
  if (Value > 511.9f) Value = 511.9f;

  return PCA9685_SetPWM(dev, 0, (uint16_t)Value);
}


float PCA9685_GetServoAngle(Servo* dev) {
  uint8_t registerAddress;
  uint8_t pwm[4];
  uint16_t offTime;

  // Calculate the base register for this channel
  registerAddress = PCA9685_LED0_ON_L + (4 * dev->Channel);

  // Read 4 bytes: ON_L, ON_H, OFF_L, OFF_H
  if (PCA9685_ReadRegisters(registerAddress, pwm, 4) != HAL_OK) {
    return -1;  // Indicate an error
  }

  // Combine OFF_L and OFF_H to get the current PWM off-time
  offTime = (pwm[3] << 8) | pwm[2];

  // Reverse the formula used in PCA9685_SetServoAngle
  float angle = ((float)(offTime - 102.4) * dev->deg_range) / (511.9 - 102.4);

  // Clamp the angle to valid range
  if (angle < 0.0f) angle = 0.0f;
  if (angle > dev->deg_range) angle = (float)dev->deg_range;

  return angle;
}


void velocity_control(Servo *servo, uint8_t control_data) {
	int current_angle = PCA9685_GetServoAngle(servo); // Get the current angle of the servo
	int target_angle = (control_data >> 2) & 0x3F; // Use bits 7:2 for target angle (6 bits)
	uint8_t speed_option = control_data & 0x03;    // Use bits 1:0 for speed option

	// Clamp target angle within servo's range
	if (target_angle < 0) {
		target_angle = 0;
	} else if (target_angle > servo->deg_range) {
		target_angle = servo->deg_range;
	}

	int step = (target_angle > current_angle) ? 1 : -1; // Determine the direction of movement

	// Map speed options to delay values
	int speed_delay;
	switch (speed_option) {
		case 0: speed_delay  = 10; break;
		case 1: speed_delay  = 20; break;
		case 2: speed_delay  = 50; break;
		case 3: speed_delay  = 100; break;
		default: speed_delay = 50; break;
	}

	while (current_angle != target_angle) {
		current_angle += step; // Increment or decrement the angle
		PCA9685_SetServoAngle(servo, current_angle); // Update the servo position
		HAL_Delay(speed_delay); // Delay to control the speed
	}
}
