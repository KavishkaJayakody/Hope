#include <Arduino.h>
#include "motors.h"
#include "Ticker.h"
#include "hope.h"
#include "profiler.h"
#include "MPU6050.h" // Include the MPU6050 header

Encoders encoders;
Motors motors;
Ticker updateTicker;
Hope hope(encoders, motors);
Profiler profiler;
MPU6050 mpu; // Create an instance of the MPU6050 class

void updateSystem()
{
  encoders.update();   // Update encoder data
  mpu.update();        // Update MPU6050 data
  profiler.update();   // Update profiler
  motors.updateMotors( // Update motor outputs
      profiler.Y_Velocity(),
      profiler.X_Velocity(),
      profiler.Omega(),
      0,
      0);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Motor and Encoder Test Starting...");

  encoders.begin();
  encoders.reset();
  motors.begin();
  motors.enable_controllers();
  hope.begin();
  mpu.begin(); // Initialize the MPU6050

  // Update encoders, motors, profiler, and MPU6050 every 20ms
  updateTicker.attach(0.02, updateSystem);
}

void loop()
{
  // Update test states and target velocities
  // hope.update();
  delay(1000);
  profiler.setTarget(200, 0, 0);
  delay(5000);
  profiler.stop();
  delay(1000);
  profiler.setTarget(0, 200, 0);
  delay(5000);
  profiler.stop();
  delay(1000);
  profiler.setTarget(0, 0, 1);
  delay(5000);
  profiler.stop();
  delay(1000);
  profiler.setTarget(50, 50, 0.1);
  delay(5000);
  profiler.stop();

  // Print debug information every 100ms
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 100)
  {
    lastPrintTime = millis();

    Serial.println("\n----------------------------------------");
    hope.printEncoderData();
    hope.printMotorOutputs();

    // Print MPU6050 data
    Serial.print("Angular X Velocity: ");
    Serial.print(mpu.getAngularVelocityX());
    Serial.print("\tLinear Y Velocity: ");
    Serial.print(mpu.getLinearVelocityY());
    Serial.print("\tLinear Z Velocity: ");
    Serial.println(mpu.getLinearVelocityZ());

    Serial.println("----------------------------------------\n");
  }

  delay(10); // Small delay to prevent overwhelming the serial output
}