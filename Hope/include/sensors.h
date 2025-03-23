#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config.h"




class Sensors;

extern Sensors sensors;


// Enum for different steering states
enum
{
    STEER_NORMAL,
    STEERING_OFF,
};


class Sensors
{
public:



    void begin()
    {   
        pinMode(BUTTON_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPressISR, CHANGE);

        // Setup ultrasonic sensor pins
        pinMode(TRIG_PIN, OUTPUT);
        pinMode(ECHO_PIN, INPUT);
        digitalWrite(TRIG_PIN, LOW);  // Ensure trigger starts LOW
    }

    // Get distance from ultrasonic sensor in cm
    float getDistance() {
        // Clear the trigger pin
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        
        // Send 10us pulse to trigger
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        
        // Read echo pin (timeout after MAX_DISTANCE)
        unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
        
        // Convert time to distance (speed of sound = 343m/s = 0.0343cm/us)
        // Divide by 2 because sound travels to object and back
        float distance = (duration * 0.0343) / 2;
        
        // Check if measurement is valid
        if (distance == 0 || distance > MAX_DISTANCE) {
            return MAX_DISTANCE;
        }
        
        // Apply simple filter to reduce noise
        filtered_distance = FILTER_ALPHA * distance + (1 - FILTER_ALPHA) * filtered_distance;
        return filtered_distance;
    }

    // Check if obstacle is detected within threshold
    bool isObstacleDetected(float threshold = OBSTACLE_THRESHOLD) {
        float distance = getDistance();
        return distance < threshold && distance > 0;
    }

    // Print sensor data
    void printDistance() {
        Serial.print("DISTANCE:");
        Serial.println(filtered_distance);
    }

    static void handleButtonPressISR(){
        sensors.handleButtonPress();
    }
    void handleButtonPress(){
        button_pressed = true;
    }
    void reset_button(){  // Call this fuction whenever using the button functionality before calling any other button functions
        button_pressed = false;
    }
    bool is_button_pressed(){
        bool button_state;
        noInterrupts();
        button_state = button_pressed;
        interrupts();
        return button_state;
    }

    void led_indicator(bool state){
        if (state){
            digitalWrite(LED_PIN, HIGH);
        }
        else{
            digitalWrite(LED_PIN, LOW);
        }
    }
    void wait_till_button(){
        reset_button();
        while(not is_button_pressed()){
            led_indicator(1);
            delay(250);
            led_indicator(0);
            delay(250);

        }

    
   

 

}
private:
    // variables for steering
    float last_steering_error = 0;
    volatile float m_cross_track_error;
    volatile float m_steering_adjustment;
    volatile bool button_pressed;
    bool left_state;
    bool right_state;
    bool no_line;

    // Ultrasonic sensor variables
    float filtered_distance = MAX_DISTANCE;
    const float MAX_DISTANCE = 400.0;      // Maximum measurable distance in cm
    const float OBSTACLE_THRESHOLD = 20.0;  // Default obstacle detection threshold in cm
    const unsigned long TIMEOUT_US = 23200; // Timeout for 400cm (MAX_DISTANCE)
    const float FILTER_ALPHA = 0.2;        // Filter coefficient (0-1)
};
