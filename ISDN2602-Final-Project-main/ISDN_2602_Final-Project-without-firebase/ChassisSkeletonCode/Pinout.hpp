#ifndef Init_H_
#define Init_H_


/*Wifi Coneection Setup hpp file*/

/*Own Code*/
#include "MotorControl.hpp"
#include "Sensor.hpp"
#include "LineTracking.hpp"
#include "MFRC522_I2C.hpp"
#include "IMU.h"

/*Include library for FireBase*/


//PWM Channel 


/*Define PWM Channels*/
#define PWM_CH0 1
#define PWM_CH1 2
#define PWM_CH2 3  
#define PWM_CH3 4  
#define PWM_CH4 5

/*2 Motors with encoder 
Motor 1 (Right)
*/
#define Motor_R_IN1 14
#define Motor_R_IN2 21
#define Motor_R_Encoder_A 47
#define Motor_R_Encoder_B 48

//Motor 2 
#define Motor_L_IN1 11
#define Motor_L_IN2 12
#define Motor_L_Encoder_A 10
#define Motor_L_Encoder_B 9

//IR Sensor 
#define IR_L 5
#define IR_R 2
#define IR_M 4 

//ICM-42688-P Pin 
#define IMU_SDA 17
#define IMU_SCL 18 
#define IMU_DRDY 7

//RFID Reader 
#define RFID_RST 36
#define RFID_SCL 35
#define RFID_SDA 41
#define RFID_IRQ 42

//Ultrasonic Pin 
#define Tirg 16 
#define Echo 15

//Servo Motor Pin 
#define Servo_Pin 6

//GPIO LED Pin 
#define LED1 37

#endif