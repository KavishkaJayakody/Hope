#include <Arduino.h>
#include "motors.h"
#include "Ticker.h"


Encoders encoders;
Motors motors;

Ticker updateTicker;


void setup(){
      Serial.begin(115200);  // For ESP32-S3 USB CDC
      delay(1000);     // Give time for Serial to initialize
      Serial.println("ESP32-S3 Starting...");
      digitalWrite(21,LOW);
      digitalWrite(19,LOW);
      encoders.begin();
      encoders.reset();
      motors.begin();
      updateTicker.attach(0.02, []()
                    {
                      encoders.update();
                      motors.update(100, 0, 0);
                    }
        );

}

void loop(){
  Serial.print("Left Encoder :");
  Serial.print(encoders.leftRPS());
  Serial.print(" Right Encoder :");
  Serial.print(encoders.rightRPS());
  Serial.print("   SPEED    :");
  Serial.println(encoders.robot_speed());
  delay(100);  // Add a small delay
}