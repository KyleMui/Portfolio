/*
 * CMR_Servo.h
 *
 *  Created on: Feb 3, 2025
 *      Author: Kyle Mui
 */

#ifndef INC_CMR_SERVO_H_
#define INC_CMR_SERVO_H_

//Includes
#include "stm32g4xx_hal.h"
#include "PCA9685PW.h"


/*
 * Servo Definition specific to this board
*/

extern Servo servo0;
extern Servo servo1;
extern Servo servo2;
extern Servo servo3;
extern Servo servo4;
extern Servo servo5;
extern Servo servo6;
extern Servo servo7;
extern Servo servo8;
extern Servo servo9;
extern Servo servo10;
extern Servo servo11;
extern Servo servo12;
extern Servo servo13;
extern Servo servo14;
extern Servo servo15;

//defines

//ENUMS
typedef enum {
    CONFIG = 0x00,
    CONTROL = 0x01,
    FEEDBACK = 0x02,
    DIAGNOSTIC = 0x03
} CANFD_MessageType;


//STRUCTS
typedef struct {
    uint16_t msg_type : 2;    // 2 bits for Message Type
    uint16_t servo_id : 4;    // 4 bits for Servo ID
    uint16_t deg_range : 11;  // 11 bits for Degree Range
    uint16_t home_flag : 1;   // 1 bit for Home Flag
    uint16_t home_pos : 12;   // 12 bits for Home Position
    uint16_t reserved : 2;    // 2 bits reserved for future use (ensures 32-bit alignment)
} ConfigMsg;

typedef struct {
    uint16_t msg_type : 2;      // 2 bits (Message Type)
    uint16_t servo_id : 4;      // 4 bits (Servo ID)
    uint16_t home : 1;          // 1 bit (Send to Home)
    uint16_t reset_home : 1;    // 1 bit (Set Home)
    uint16_t control_mode : 2;  // 2 bits (Position, Velocity, PWM, Stop)
    uint16_t control_data : 12; // 12 bits (Control Data)
    uint16_t clear_faults : 1;  // 1 bit (Clear Faults)
    uint16_t query_data : 1;    // 1 bit (Query Data)
    uint16_t reserved : 2;      // 2 bits reserved (Ensures 32-bit alignment)
} ControlMsg;

typedef struct {
    uint16_t msg_type : 2;       // 2 bits (Message Type)
    uint16_t servo_id : 4;       // 4 bits (Servo ID)
    uint16_t control_data : 12;  // 12 bits (Control Data for current control method)
    uint16_t current_data : 12;  // 12 bits (Current Data, 0-20A in 1mA steps)
    uint16_t temp_data : 8;      // 8 bits (Temperature Data, -40 to 125°C in 0.5°C steps)
    uint16_t reserved : 2;       // 2 bits reserved (Ensures 32-bit alignment)
} FeedbackMsg;

typedef struct {
    //yet to be implemented
} DiagnosticMsg;


//Initialization
void Config_Msg_Init(ConfigMsg *Msg, uint8_t RxData[32]);
void Control_Msg_Init(ControlMsg *Msg, uint8_t RxData[32]);
void Feedback_Msg_Init(ControlMsg *Msg, uint8_t RxData[32]);
void Diagnostic_Msg_Init(ControlMsg *Msg, uint8_t RxData[32]);

//Functions
HAL_StatusTypeDef Run_Ctrl_Msg(ControlMsg msg);




#endif /* INC_CMR_SERVO_H_ */
