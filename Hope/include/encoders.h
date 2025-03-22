#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "config.h"
#include <math.h>

class Encoders;

extern Encoders encoders;

class Encoders
{
public:
    void begin()
    {
        pinMode(LeftFrontEncoderPin1, INPUT_PULLUP);
        pinMode(LeftFrontEncoderPin2, INPUT_PULLUP);
        pinMode(LeftBackEncoderPin1, INPUT_PULLUP);
        pinMode(LeftBackEncoderPin2, INPUT_PULLUP);
        pinMode(RightFrontEncoderPin1, INPUT_PULLUP);
        pinMode(RightFrontEncoderPin2, INPUT_PULLUP);
        pinMode(RightBackEncoderPin1, INPUT_PULLUP);
        pinMode(RightBackEncoderPin2, INPUT_PULLUP);

        attachInterrupt(digitalPinToInterrupt(LeftFrontEncoderPin1), updateLeftFrontEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(LeftFrontEncoderPin2), updateLeftFrontEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(LeftBackEncoderPin1), updateLeftBackEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(LeftBackEncoderPin2), updateLeftBackEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RightFrontEncoderPin1), updateRightFrontEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RightFrontEncoderPin2), updateRightFrontEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RightBackEncoderPin1), updateRightBackEncoderISR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(RightBackEncoderPin2), updateRightBackEncoderISR, CHANGE);

        reset();
    }
    

    void reset()
    {
        noInterrupts();
        encoderCounterLeftFront = 0;
        encoderCounterLeftBack = 0;
        encoderCounterRightFront = 0;
        encoderCounterRightBack = 0;
        robot_distance = 0;
        robot_angle = 0;
        lateral_distance = 0;
        interrupts();
    }

    static void updateLeftFrontEncoderISR()
    {
        encoders.updateLeftFrontEncoder();
    }

    static void updateLeftBackEncoderISR()
    {
        encoders.updateLeftBackEncoder();
    }

    static void updateRightFrontEncoderISR()
    {
        encoders.updateRightFrontEncoder();
    }

    static void updateRightBackEncoderISR()
    {
        encoders.updateRightBackEncoder();
    }

    void updateLeftFrontEncoder()
    {
        int MSB = digitalRead(LeftFrontEncoderPin1);
        int LSB = digitalRead(LeftFrontEncoderPin2);
        int encoded = (MSB << 1) | LSB;
        int sum = (lastEncodedLeftFront << 2) | encoded;
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        {
            encoderCounterLeftFront--;
        }
        else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        {
            encoderCounterLeftFront++;
        }
        lastEncodedLeftFront = encoded;
    }

    void updateLeftBackEncoder()
    {
        int MSB = digitalRead(LeftBackEncoderPin1);
        int LSB = digitalRead(LeftBackEncoderPin2);
        int encoded = (MSB << 1) | LSB;
        int sum = (lastEncodedLeftBack << 2) | encoded;
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        {
            encoderCounterLeftBack--;
        }
        else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        {
            encoderCounterLeftBack++;
        }
        lastEncodedLeftBack = encoded;
    }

    void updateRightFrontEncoder()
    {
        int MSB = digitalRead(RightFrontEncoderPin1);
        int LSB = digitalRead(RightFrontEncoderPin2);
        int encoded = (MSB << 1) | LSB;
        int sum = (lastEncodedRightFront << 2) | encoded;
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        {
            encoderCounterRightFront--;
        }
        else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        {
            encoderCounterRightFront++;
        }
        lastEncodedRightFront = encoded;
    }

    void updateRightBackEncoder()
    {
        int MSB = digitalRead(RightBackEncoderPin1);
        int LSB = digitalRead(RightBackEncoderPin2);
        int encoded = (MSB << 1) | LSB;
        int sum = (lastEncodedRightBack << 2) | encoded;
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        {
            encoderCounterRightBack--;
        }
        else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        {
            encoderCounterRightBack++;
        }
        lastEncodedRightBack = encoded;
    }

    void update()
    {
        unsigned long currentTime = micros();
        left_front_delta = 0;
        left_back_delta = 0;
        right_front_delta = 0;
        right_back_delta = 0;
        
        time_change_1 = prevTime;
        time_change_u = currentTime-prevTime;
        if (time_change_u==0){
            time_change_u = 1;
        }
    
        prevTime = currentTime;

        // Make sure values don't change while being read
        noInterrupts();
        left_front_delta = encoderCounterLeftFront;
        left_back_delta = encoderCounterLeftBack;
        right_front_delta = encoderCounterRightFront;
        right_back_delta = encoderCounterRightBack;
        encoderCounterLeftFront = 0;
        encoderCounterLeftBack = 0;
        encoderCounterRightFront = 0;
        encoderCounterRightBack = 0;
        interrupts();

        float left_front_change = (float)left_front_delta * MM_PER_ROTATION / PULSES_PER_ROTATION;
        float left_back_change = (float)left_back_delta * MM_PER_ROTATION / PULSES_PER_ROTATION;
        float right_front_change = (float)right_front_delta * MM_PER_ROTATION / PULSES_PER_ROTATION;
        float right_back_change = (float)right_back_delta * MM_PER_ROTATION / PULSES_PER_ROTATION;

        // Average of all four wheels for forward motion
        fwd_change = 0.25 * (right_front_change + right_back_change + left_front_change + left_back_change);
        robot_distance += fwd_change;

        lateral_distance += loopTime_s()*robot_lateral_speed();

        // For rotation, average the difference between right and left sides
        float left_side_change = 0.5 * (left_front_change + left_back_change);
        float right_side_change = 0.5 * (right_front_change + right_back_change);
        rot_change = (right_side_change - left_side_change) * DEG_PER_MM_DIFFERENCE;
        robot_angle += loopTime_s()*robot_omega();

        
    }

    inline int loopTime_us(){
        int looptime = time_change_u;
        return looptime;
    }

    inline float loopTime_s(){
        float time = (float)time_change_u/1000000.0;
        return time;
    }
    
    inline float robotDistance()
    {
        float distance;

        noInterrupts();
        distance = robot_distance; //in mm
        interrupts();

        return distance;
    }
    inline float robotLateralDistance()
    {
        float distance;

        noInterrupts();
        distance = lateral_distance; //in mm
        interrupts();

        return distance;
    }
    inline float robotAngle()
    {
        float angle;

        noInterrupts();
        angle = robot_angle;
        interrupts();

        return angle;
    }

    inline float robot_speed(){
        float speed ;

                speed=(leftFrontSpeed() + leftBackSpeed() + 
                                 rightFrontSpeed() + rightBackSpeed()) / 4.0;

        return speed;
    }

    inline float robot_omega(){  /// Note thas this is for side velocities. Not actually omega
        float omega;

            omega = (-(encoders.leftFrontSpeed() + encoders.leftBackSpeed())+
                                  (encoders.rightFrontSpeed() + encoders.rightBackSpeed())) / (4.0*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS));
            

        return omega;
    }
    inline float robot_lateral_speed(){
        float lateral_speed = ((encoders.leftFrontSpeed() - encoders.leftBackSpeed()) + 
                              (encoders.rightBackSpeed() - encoders.rightFrontSpeed())) / 4.0;

        return lateral_speed;
    }

    inline float robot_fwd_change()
    {
        float distance;

        noInterrupts();
        distance = fwd_change;
        interrupts();

        return distance;
    }

    inline float robot_rot_change()
    {
        float distance;
        
        noInterrupts();
        distance = rot_change;
        interrupts();

        return distance;
    }

    inline float leftFrontSpeed(){
        float spd;
        noInterrupts();
        spd = ((float)left_front_delta * MM_PER_ROTATION / (PULSES_PER_ROTATION * time_change_u)) * 1000000;
        interrupts();
        return spd;
    }

    inline float leftBackSpeed(){
        float spd;
        noInterrupts();
        spd = ((float)left_back_delta * MM_PER_ROTATION / (PULSES_PER_ROTATION * time_change_u)) * 1000000;
        interrupts();
        return spd;
    }

    inline float rightFrontSpeed(){
        float spd;
        noInterrupts();
        spd = ((float)right_front_delta * MM_PER_ROTATION / (PULSES_PER_ROTATION * time_change_u)) * 1000000;
        interrupts();
        return spd;
    }

    inline float rightBackSpeed(){
        float spd;
        noInterrupts();
        spd = ((float)right_back_delta * MM_PER_ROTATION / (PULSES_PER_ROTATION * time_change_u)) * 1000000;
        interrupts();
        return spd;
    }

    inline int leftFrontRPS(){
        int rps;
        noInterrupts();
        rps = ((float)left_front_delta/time_change_u) * 1000000.0;
        interrupts();
        return rps;
    }

    inline int leftBackRPS(){
        int rps;
        noInterrupts();
        rps = ((float)left_back_delta/time_change_u) * 1000000.0;
        interrupts();
        return rps;
    }

    inline int rightFrontRPS(){
        int rps;
        noInterrupts();
        rps = ((float)right_front_delta/time_change_u) * 1000000.0;
        interrupts();
        return rps;
    }

    inline int rightBackRPS(){
        int rps;
        noInterrupts();
        rps = ((float)right_back_delta/time_change_u) * 1000000.0;
        interrupts();
        return rps;
    }

private:
    volatile long encoderCounterLeftFront;
    volatile long lastEncodedLeftFront;
    volatile long encoderCounterLeftBack;
    volatile long lastEncodedLeftBack;
    volatile long encoderCounterRightFront;
    volatile long lastEncodedRightFront;
    volatile long encoderCounterRightBack;
    volatile long lastEncodedRightBack;

    int left_front_delta;
    int left_back_delta;
    int right_front_delta;
    int right_back_delta;

    volatile float robot_distance;
    volatile float robot_angle;
    volatile float lateral_distance = 0;

    unsigned long prevTime;
    float fwd_change;
    float rot_change;
    float time_change_u;
    int time_change_1;
    int time_change_2;
    int time_change_3;
};