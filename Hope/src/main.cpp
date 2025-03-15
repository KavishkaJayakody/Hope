#include <Arduino.h>
#include "motors.h"
#include "Ticker.h"
#include "hope.h"

Encoders encoders;
Motors motors;
Ticker updateTicker;
Hope hope(encoders, motors);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Motor and Encoder Test Starting...");
  
  encoders.begin();
  encoders.reset();
  motors.begin();
  motors.enable_controllers();
  hope.begin();
  
  // Update encoders and motors every 20ms
  updateTicker.attach(0.02, []() {
    encoders.update();
    motors.update(hope.getTargetVelLF(), hope.getTargetVelLB(), 
                 hope.getTargetVelRF(), hope.getTargetVelRB());
  });
}

void loop() {
  // Update test states and target velocities
  hope.update();
  
  // Print debug information every 100ms
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();
    
    Serial.println("\n----------------------------------------");
    hope.printEncoderData();
    hope.printMotorOutputs();
    Serial.println("----------------------------------------\n");
  }
  
  delay(10);  // Small delay to prevent overwhelming the serial output
}