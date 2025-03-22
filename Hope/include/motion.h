#pragma once

#include "config.h"
#include "Arduino.h"
#include "encoders.h"
#include "profiler.h"
#include "motors.h"
#include <math.h>

class Motion;
extern Motion motion;

class Motion {
public:
    void forward(float distance){
        if(distance>0){
            profiler.setTarget(MAX_SPEED,0,0);
            float Target_distance = encoders.robotDistance()+distance;
            Serial.println(distance);
            while(Target_distance>encoders.robotDistance()){
                delay(5);
            }
            profiler.stop();
        }
        else{
            profiler.setTarget(-MAX_SPEED,0,0);
            float Target_distance = encoders.robotDistance()+distance;
            Serial.println(distance);
            while(Target_distance<encoders.robotDistance()){
                delay(5);
            }
            profiler.stop();
        }
    }

    void side(float distance){
        if(distance>0){
            profiler.setTarget(0,MAX_SPEED,0);
            float Target_distance = encoders.robotLateralDistance()+distance;
            Serial.println(distance);
            while(Target_distance>encoders.robotLateralDistance()){
                delay(5);
            }
            profiler.stop();
        }
        else{
            profiler.setTarget(0,-MAX_SPEED,0);
            float Target_distance = encoders.robotLateralDistance()+distance;
            Serial.println(distance);
            while(Target_distance<encoders.robotLateralDistance()){
                delay(5);
            }
            profiler.stop();
        }
    }

    void rotate(float angle){  // angle in radians
        if(angle>0){
            profiler.setTarget(0,0,MAX_ANGULAR_SPEED);  // ~0.314 rad/s = 18 deg/s
            float Target_angle = encoders.robotAngle()+angle;
            Serial.println(angle);
            while(Target_angle>encoders.robotAngle()){
                delay(5);
            }
            profiler.stop();
        }
        else{
            profiler.setTarget(0,0,-MAX_ANGULAR_SPEED);  // ~-0.314 rad/s = -18 deg/s
            float Target_angle = encoders.robotAngle()+angle;
            Serial.println(angle);
            while(Target_angle<encoders.robotAngle()){
                delay(5);
            }
            profiler.stop();
        }
    }

    // Helper function to convert degrees to radians
    float deg2rad(float deg) {
        return deg * (M_PI / 180.0);
    }

    // Helper function to convert radians to degrees
    float rad2deg(float rad) {
        return rad * (180.0 / M_PI);
    }

private:
    const float DEFAULT_LINEAR_SPEED = 100;    // mm/s
    const float DEFAULT_ANGULAR_SPEED = 0.314;  // rad/s (~18 deg/s)
};