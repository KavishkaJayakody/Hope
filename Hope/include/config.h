// config.h
//change this to robot configurations. 
// we can have a nother maze config file also
#pragma once
#include "esp_system.h"


const int loopTime = 18500; //in micro seconds

//**************************************************ENCODER CONFIG**************************************************************************
const int LeftEncoderPin1 = 48;
const int LeftEncoderPin2 = 35;

const int RightEncoderPin1 = 36;
const int RightEncoderPin2 = 37;  // Assuming you might have another pin for Right Encoder


//**************************************************SENSOR CONFIG**************************************************************************



#define LSM6DS3_ADDRESS 0x6B // I2C address of LSM6DS3




//**************************************************Robot CONFIG**************************************************************************
const int wheelDiameter = 34; //in mm

const float RADIANS_PER_DEGREE = PI/180;
const float WHEEL_GAP = 101; // distance between the wheels in mm
const float MOUSE_RADIUS =  WHEEL_GAP/2;

const float MM_PER_ROTATION = PI*wheelDiameter; //  pi*wheel diameter .......d=34mm
const float DEG_PER_MM_DIFFERENCE = 180.0/(2 * MOUSE_RADIUS * PI);

//*****************************************************COMMUNICATIONS************************************************************************
const char* SSID = "SLT-4G_WataNandun";//"SLT-ADSL-92776";//   // network credentials
const char* PASSWORD = "Nwata@#com";//"J1234567890";//
const int LOCAL_PORT = 12345;  // UDP port to listen on
const bool WIFI_ENABLE = true;
const char* REMOTE_IP = "192.168.1.157";
const int REMOTE_PORT = 5005;

//***************************************************MOTOR CONFIG***************************************************************************/
//Left and Right Motor configurations
const float MAX_MOTOR_PERCENTAGE_SEARCH = 90;
const float MAX_MOTOR_PERCENTAGE_FINAL = 100;

const int MIN_MOTOR_PERCENTAGE = 5; // when the given percentage is below this value, percentage is set to zero to damp oscillations
const int MIN_MOTOR_BIAS = 10;// miinimum percentage that should be given for the motors to spin
const int PWM_RESOLUTION_BITS = 8;
const int PWM_RESOLUTION = 256; //2^8 use a suitable code to automate this
const int PULSES_PER_ROTATION = 2205;//1430

const float MOTOR_BALANCE = 0;    //The Percentage fed into the left(add) and right(deduct) motors to math the motor performance 
const int M_BALNCE_PWM = MAX_MOTOR_PERCENTAGE_SEARCH*PWM_RESOLUTION*MOTOR_BALANCE/10000;


const int LEFT_MOTOR_PWM = 47;   //left is motor A
const int LEFT_MOTOR_IN1 = 41;
const int LEFT_MOTOR_IN2 = 42;
const int RIGHT_MOTOR_IN2 = 42; //right is motor B
const int RIGHT_MOTOR_IN1 = 42;
const int RIGHT_MOTOR_PWM = 20;


//change motor directions to make the motors spin forward when both motors are forward commanded
#define MOTOR_LEFT_POLARITY (-1)
#define MOTOR_RIGHT_POLARITY (1)

//PD parameters 
const float FWD_KP_FINAL = 0.5;
const float FWD_KD_FINAL = 0.9;
const float ROT_KP_FINAL = 2.4;
const float ROT_KD_FINAL = 0.9;

const float FWD_KP_SMALL = 0.5;    
const float FWD_KD_SMALL = 0.9;
const float ROT_KP_90 = 2.4;   // measured for(90,360,0,3600)   @7.4V battery
const float ROT_KD_90 = 0.5;



const float STEERING_KP_SEARCH_FAST = 0.6;//0.3;
const float STEERING_KD_SEARCH_FAST = 11;//8;

const float STEERING_KP_FINAL_FAST = 0.9;
const float STEERING_KD_FINAL_FAST = 18;

const float STEERING_KP_SEARCH_SLOW = 1.4;//1.4
const float STEERING_KD_SEARCH_SLOW = 18; //18

const float STEERING_KP_FINAL_SLOW = 0.2; //0.3   9  good   - 0.25 14  somewhat oscillatory  0.2 9 - good but somewht osicallatory
const float STEERING_KD_FINAL_SLOW = 13.5;

const float STEERING_ADJUST_LIMIT = 10.0;

const float NOMINAL_BATTERY_V = 8.0;

//**************************************************REPORTING CONFIG**************************************************************************
uint8_t broadcastAddress[] = { 0xEC, 0xDA, 0x3B, 0x51, 0xA5, 0x84 }; // RECEIVER MAC Address

