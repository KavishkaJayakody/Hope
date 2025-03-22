// config.h
//change this to robot configurations. 
// we can have a nother maze config file also
#pragma once
#include "esp_system.h"


const int loopTime = 18500; //in micro seconds

//**************************************************ENCODER CONFIG**************************************************************************
const int LeftFrontEncoderPin1 = 36;//48;
const int LeftFrontEncoderPin2 = 37;//35;

const int LeftBackEncoderPin1 = 48;//36;  // You'll need to assign the actual pin numbers
const int LeftBackEncoderPin2 = 35;//37;  // You'll need to assign the actual pin numbers

const int RightFrontEncoderPin1 = 41;
const int RightFrontEncoderPin2 = 40;

const int RightBackEncoderPin1 = 39;  // You'll need to assign the actual pin numbers
const int RightBackEncoderPin2 = 38;  // You'll need to assign the actual pin numbers


//**************************************************Robot CONFIG**************************************************************************
const float wheelDiameter = 48.0; //in mm

const float RADIANS_PER_DEGREE = PI/180;
const float WHEEL_GAP = 155.0; // distance between the wheels in mm
const float WHEEL_BASE = 100.0;
const float ROBOT_X_RADIUS =  WHEEL_GAP/2;
const float ROBOT_Y_RADIUS = WHEEL_BASE/2;

const float MM_PER_ROTATION = PI*wheelDiameter; //  pi*wheel diameter .......d=34mm
const float DEG_PER_MM_DIFFERENCE = 180.0/(2 * ROBOT_X_RADIUS * PI);

//*****************************************************COMMUNICATIONS************************************************************************
const char* SSID = "Oshani";//"SLT-ADSL-92776";//   // network credentials
const char* PASSWORD = "12345678";//"J1234567890";//
const int LOCAL_PORT = 12345;  // UDP port to listen on
const bool WIFI_ENABLE = true;
const char* REMOTE_IP = "192.168.83.13";// "192.168.1.157";
const int REMOTE_PORT = 5005;

//***************************************************MOTOR CONFIG***************************************************************************/
//Motor configurations
const float MAX_MOTOR_PERCENTAGE_SEARCH = 90;
const float MAX_MOTOR_PERCENTAGE_FINAL = 100;

const int MIN_MOTOR_PERCENTAGE = 5; // when the given percentage is below this value, percentage is set to zero to damp oscillations
const int MIN_MOTOR_BIAS = 10;// miinimum percentage that should be given for the motors to spin
const int PWM_RESOLUTION_BITS = 8;
const int PWM_RESOLUTION = 256; //2^8 use a suitable code to automate this
const int PULSES_PER_ROTATION = 1430;//2205;//1430

const float MOTOR_BALANCE = 0;    //The Percentage fed into the left(add) and right(deduct) motors to math the motor performance 
const int M_BALNCE_PWM = MAX_MOTOR_PERCENTAGE_SEARCH*PWM_RESOLUTION*MOTOR_BALANCE/10000;

// Left Front Motor
const int LEFT_FRONT_MOTOR_IN1 = 47;
const int LEFT_FRONT_MOTOR_IN2 = 21;

// Left Back Motor
const int LEFT_BACK_MOTOR_IN1 = 13;
const int LEFT_BACK_MOTOR_IN2 = 14;

// Right Front Motor
const int RIGHT_FRONT_MOTOR_IN1 = 16;
const int RIGHT_FRONT_MOTOR_IN2 = 15;

// Right Back Motor
const int RIGHT_BACK_MOTOR_IN1 = 42;
const int RIGHT_BACK_MOTOR_IN2 = 2;

//change motor directions to make the motors spin forward when both motors are forward commanded
#define MOTOR_LEFT_FRONT_POLARITY (-1)
#define MOTOR_LEFT_BACK_POLARITY (-1)
#define MOTOR_RIGHT_FRONT_POLARITY (1)
#define MOTOR_RIGHT_BACK_POLARITY (1)

//PD parameters 

const float FWD_KP = 0.80;    
const float FWD_KD = 0.9;
const float ROT_KP = 6.0;//0.54;   // measured for(90,360,0,3600)   @7.4V battery
const float ROT_KD = 0.5;

const float LINEAR_ACC = 100;
const float ANGULAR_ACC = 3.14;

const float MAX_SPEED = 100.0;
const float MAX_ANGULAR_SPEED = 0.5 ;


const float NOMINAL_BATTERY_V = 8.0;

//**************************************************REPORTING CONFIG**************************************************************************
uint8_t broadcastAddress[] = { 0xEC, 0xDA, 0x3B, 0x51, 0xA5, 0x84 }; // RECEIVER MAC Address



//****************************************************Sensor config************************/************************** */ */
const int BUTTON_PIN = 0;
const int LED_PIN = 48; 
