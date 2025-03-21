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
};
