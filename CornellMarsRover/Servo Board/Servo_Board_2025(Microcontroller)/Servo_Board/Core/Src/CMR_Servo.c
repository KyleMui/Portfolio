/*
 * CMR_Servo.c
 *
 *  Created on: Feb 6, 2025
 *      Author: Kyle Mui
 */


//Includes
#include "CMR_Servo.h"
#include "PCA9685PW.h"
#include "main.h"

//Private Variables
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


//Initialization Functions
void Config_Msg_Init(ConfigMsg *Msg, uint8_t RxData[32]) {
    Msg->msg_type  = (RxData[0] >> 6) & 0x03;
    Msg->servo_id  = (RxData[0] >> 2) & 0x0F;
    Msg->deg_range = ((RxData[1] << 3) | (RxData[0] & 0x07));
    Msg->home_flag = (RxData[2] >> 7) & 0x01;
    Msg->home_pos  = ((RxData[4] << 4) | (RxData[3] >> 4));
    Msg->reserved  = 0;

    Servo *servo;

    switch (Msg->servo_id) {
        case 0:  servo = &servo0; break;
        case 1:  servo = &servo1; break;
        case 2:  servo = &servo2; break;
        case 3:  servo = &servo3; break;
        case 4:  servo = &servo4; break;
        case 5:  servo = &servo5; break;
        case 6:  servo = &servo6; break;
        case 7:  servo = &servo7; break;
        case 8:  servo = &servo8; break;
        case 9:  servo = &servo9; break;
        case 10: servo = &servo10; break;
        case 11: servo = &servo11; break;
        case 12: servo = &servo12; break;
        case 13: servo = &servo13; break;
        case 14: servo = &servo14; break;
        case 15: servo = &servo15; break;
        default: return;
    }

    servo->deg_range = Msg->deg_range;
    if (Msg->home_flag == 1) {
        servo->home_flag = 1;
        servo->home = Msg->home_pos;
        PCA9685_SetServoAngle(servo, Msg->home_pos);
    }
}

void Control_Msg_Init(ControlMsg *Msg, uint8_t RxData[32]) {
	Msg->msg_type 	  = ( RxData[0] >> 6) & 0x03;
	Msg->servo_id 	  = ( RxData[0] >> 2) & 0x0F;
	Msg->home 		  = ( RxData[0] >> 1) & 0x01;
	Msg->reset_home   =   RxData[0] & 0x01;
	Msg->control_mode = ( RxData[1] >> 6) & 0x03;
	Msg->control_data = ((RxData[2] << 4) | (RxData[1] & 0x0F));
	Msg->clear_faults = ( RxData[3] >> 7) & 0x01;
	Msg->query_data   = ( RxData[3] >> 6) & 0x01;
	Msg->reserved 	  = 0;
}

/* TO BE IMPLEMENTED if NEEDED
//Initialization Functions
void Config_Msg_Init(ConfigMsg *Msg, uint8_t RxData[32]) {
	Msg->msg_type = (RxData[0] >> 6) & 0x03;  // Extract bits 7:6 from RxData[0]
	Msg->servo_id = (RxData[0] >> 2) & 0x0F;  // Extract bits 5:2 from RxData[0]
	Msg->deg_range = (((RxData[0] & 0x03) << 9 ) | (RxData[1] << 1) | (RxData[2] >> 7)); // Combine bits 1:0 from RxData[0], bits 7:0 from RxData[1], bit 7 from RxData[2]
	Msg->home_flag = (RxData[2] >> 6) & 0x01;  // Extract bit 6 from RxData[2]
	Msg->home_pos = (((RxData[3] & 0x3f) << 6 ) | (RxData[4] >> 2));  // Combine bits 5:0 from RxData[3] and bits 7:2 from RxData[4]
	Msg->reserved = 0;

	Servo_Init(&servo[Msg->servo_id], Msg->servo_id, Msg->deg_range, Msg->home_flag, Msg->home_pos);
}

void Feedback_Msg_Init(FeedbackMsg *Msg, uint8_t RxData[32]){
    Msg->msg_type = (RxData[0] >> 6) & 0x03;
    Msg->servo_id = (RxData[0] >> 2) & 0x0F;
    Msg->control_data = (((RxData[0] & 0x03) << 10 ) | (RxData[1] << 2) | (RxData[2] >> 6));
    Msg->current_data = (((RxData[2] & 0x3f) << 6 ) | (RxData[3] >> 2));
    Msg->temp_data = (((RxData[3] & 0x03) << 6) | (RxData[4] >> 2));
    Msg->reserved = 0;
}

void Diagnostic_Msg_Init(DiagnosticMsg *Msg, uint8_t Rxdata[32]) {
	//Yet to be implemented
}
*/

HAL_StatusTypeDef Run_Ctrl_Msg(ControlMsg msg) {
    Servo* servo;

    switch (msg.servo_id) {
        case 0:  servo = &servo0; break;
        case 1:  servo = &servo1; break;
        case 2:  servo = &servo2; break;
        case 3:  servo = &servo3; break;
        case 4:  servo = &servo4; break;
        case 5:  servo = &servo5; break;
        case 6:  servo = &servo6; break;
		case 7:  servo = &servo7; break;
		case 8:  servo = &servo8; break;
		case 9:  servo = &servo9; break;
		case 10:  servo = &servo10; break;
		case 11:  servo = &servo11; break;
		case 12:  servo = &servo12; break;
		case 13:  servo = &servo13; break;
		case 14:  servo = &servo14; break;
		case 15:  servo = &servo15; break;

        default: return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;

    if (msg.home == 1) {
        status = PCA9685_SetServoAngle(servo, servo->home);
    } else if (msg.reset_home == 1) {
        servo->home = (float)msg.control_data;
    } else {
        switch (msg.control_mode) {
            case 0:
                status = PCA9685_SetServoAngle(servo, (float)msg.control_data);
                break;
            case 1:
                velocity_control(servo, msg.control_data);
                break;
            case 2:
                status = PCA9685_SetPWM(servo, 0, 0);
                break;
            case 3:
                break;
            default:
                return HAL_ERROR;
        }
    }

    if (status != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;


//Functions
void parse_message(uint8_t RxData[32]) {
	int Msg_Type = (RxData[0] >> 6) & 0x03;
	if (Msg_Type == 0) {
		ConfigMsg Msg;
		Config_Msg_Init(&Msg, RxData);
	} else if (Msg_Type == 1) {
		ControlMsg Msg;
		Control_Msg_Init(&Msg, RxData);
	} else if (Msg_Type == 2) {
		FeedbackMsg Msg;
		Feedback_Msg_Init(&Msg, RxData);
	} else if (Msg_Type == 3) {
		DiagnosticMsg Msg;
		Diagnostic_Msg_Init(&Msg, RxData);
	}
}













