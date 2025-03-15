#ifndef HOPE_H
#define HOPE_H

#include <Arduino.h>
#include "motors.h"

class Hope {
public:
    enum TestState {
        TEST_FORWARD,
        TEST_BACKWARD,
        TEST_ROTATE_CW,
        TEST_ROTATE_CCW,
        TEST_INDIVIDUAL,
        TEST_DIAGONAL,
        TEST_STOP
    };

    Hope(Encoders& encoders, Motors& motors) 
        : encoders_(encoders), motors_(motors),
          currentTest_(TEST_FORWARD), testStartTime_(0),
          target_vel_lf_(0), target_vel_lb_(0),
          target_vel_rf_(0), target_vel_rb_(0) {}

    void begin() {
        testStartTime_ = millis();
    }

    void printEncoderData() {
        Serial.println("Encoder Data:");
        Serial.print("LF: "); Serial.print(encoders_.leftFrontRPS());
        Serial.print(" LB: "); Serial.print(encoders_.leftBackRPS());
        Serial.print(" RF: "); Serial.print(encoders_.rightFrontRPS());
        Serial.print(" RB: "); Serial.print(encoders_.rightBackRPS());
        Serial.println();
        
        Serial.print("Speeds (mm/s) - LF: "); Serial.print(encoders_.leftFrontSpeed());
        Serial.print(" LB: "); Serial.print(encoders_.leftBackSpeed());
        Serial.print(" RF: "); Serial.print(encoders_.rightFrontSpeed());
        Serial.print(" RB: "); Serial.print(encoders_.rightBackSpeed());
        Serial.println();
    }

    void printMotorOutputs() {
        Serial.println("Motor Outputs (%):");
        Serial.print("LF: "); Serial.print(motors_.getLeftFrontPercentage());
        Serial.print(" LB: "); Serial.print(motors_.getLeftBackPercentage());
        Serial.print(" RF: "); Serial.print(motors_.getRightFrontPercentage());
        Serial.print(" RB: "); Serial.print(motors_.getRightBackPercentage());
        Serial.println();
    }

    void updateMotorsBasedOnTest() {
        switch(currentTest_) {
            case TEST_FORWARD:
                target_vel_lf_ = 100;
                target_vel_lb_ = 100;
                target_vel_rf_ = 100;
                target_vel_rb_ = 100;
                Serial.println("Testing: Forward Movement");
                break;
                
            case TEST_BACKWARD:
                target_vel_lf_ = -100;
                target_vel_lb_ = -100;
                target_vel_rf_ = -100;
                target_vel_rb_ = -100;
                Serial.println("Testing: Backward Movement");
                break;
                
            case TEST_ROTATE_CW:
                target_vel_lf_ = 100;
                target_vel_lb_ = 100;
                target_vel_rf_ = -100;
                target_vel_rb_ = -100;
                Serial.println("Testing: Clockwise Rotation");
                break;
                
            case TEST_ROTATE_CCW:
                target_vel_lf_ = -100;
                target_vel_lb_ = -100;
                target_vel_rf_ = 100;
                target_vel_rb_ = 100;
                Serial.println("Testing: Counter-Clockwise Rotation");
                break;
                
            case TEST_INDIVIDUAL:
                static int activeMotor = 0;
                if (millis() - testStartTime_ > TEST_DURATION/4) {
                    testStartTime_ = millis();
                    activeMotor = (activeMotor + 1) % 4;
                }
                
                target_vel_lf_ = (activeMotor == 0) ? 100 : 0;
                target_vel_lb_ = (activeMotor == 1) ? 100 : 0;
                target_vel_rf_ = (activeMotor == 2) ? 100 : 0;
                target_vel_rb_ = (activeMotor == 3) ? 100 : 0;
                Serial.print("Testing: Individual Motor "); Serial.println(activeMotor);
                break;
                
            case TEST_DIAGONAL:
                static bool diagonal_pattern = true;
                if (millis() - testStartTime_ > TEST_DURATION/2) {
                    testStartTime_ = millis();
                    diagonal_pattern = !diagonal_pattern;
                }
                
                if (diagonal_pattern) {
                    target_vel_lf_ = 100;
                    target_vel_lb_ = 0;
                    target_vel_rf_ = 0;
                    target_vel_rb_ = 100;
                    Serial.println("Testing: Diagonal (FL-BR)");
                } else {
                    target_vel_lf_ = 0;
                    target_vel_lb_ = 100;
                    target_vel_rf_ = 100;
                    target_vel_rb_ = 0;
                    Serial.println("Testing: Diagonal (FR-BL)");
                }
                break;
                
            case TEST_STOP:
                target_vel_lf_ = 0;
                target_vel_lb_ = 0;
                target_vel_rf_ = 0;
                target_vel_rb_ = 0;
                Serial.println("Testing: Stop");
                break;
        }
    }

    void update() {
        // Change test state every TEST_DURATION milliseconds
        if (millis() - testStartTime_ >= TEST_DURATION) {
            testStartTime_ = millis();
            currentTest_ = static_cast<TestState>((static_cast<int>(currentTest_) + 1) % 7);
            
            // Reset encoders at the start of each test
            encoders_.reset();
        }
        
        // Update target velocities based on current test
        updateMotorsBasedOnTest();
    }

    // Getters for target velocities
    float getTargetVelLF() const { return target_vel_lf_; }
    float getTargetVelLB() const { return target_vel_lb_; }
    float getTargetVelRF() const { return target_vel_rf_; }
    float getTargetVelRB() const { return target_vel_rb_; }

private:
    Encoders& encoders_;
    Motors& motors_;
    TestState currentTest_;
    unsigned long testStartTime_;
    static const unsigned long TEST_DURATION = 3000; // 3 seconds per test

    // Target velocities for each motor
    float target_vel_lf_;
    float target_vel_lb_;
    float target_vel_rf_;
    float target_vel_rb_;
};

#endif // HOPE_H 