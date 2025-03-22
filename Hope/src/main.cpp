#include <Arduino.h>
#include "motors.h"
#include "Ticker.h"
#include "hope.h"
#include "profiler.h"
#include "communications.h"
#include "sensors.h"
#include "motion.h"

Encoders encoders;
Motors motors;
Ticker updateTicker;
Hope hope(encoders, motors);
Profiler profiler;
Communications communications;
Sensors sensors;
Motion motion;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Motor and Encoder Test Starting...");
  //communications.begin();
  sensors.begin();
  //motion.begin();
  encoders.begin();
  encoders.reset();
  motors.begin();
  motors.enable_controllers();
  hope.begin();
  
  // Update encoders and motors every 20ms
  updateTicker.attach(0.02, []() {
    encoders.update();
    profiler.update();
    //motion.update();
    communications.check();
    //communications.send_velocity();
    motors.updateMotors(profiler.Y_Velocity(), profiler.X_Velocity(), profiler.Omega(), 0, 0);

  });
  profiler.setTarget(0,0,0);
  delay(1000);
  profiler.setTarget(0, 0, 0.314);
  delay(10);
  // for (int i = -100; i<101;i++){
  //   motors.set_left_front_motor_percentage(i);
  //   motors.set_right_front_motor_percentage(i);
  //   motors.set_left_back_motor_percentage(i);
  //   motors.set_right_back_motor_percentage(i);
  //   delay(200);
  //       // Print individual motor velocities
  //   //Serial.print("MOTORS:");
  //   //Serial.print(m_left_front_velocity);   // Left Front target velocity
  //   Serial.print(i);
  //   Serial.print(",");
  //   Serial.print(encoders.leftFrontSpeed());  // Left Front actual velocity
  //   Serial.print(",");
  //   //Serial.print(m_right_front_velocity);  // Right Front target velocity
  //   //Serial.print(",");
  //   Serial.print(encoders.rightFrontSpeed()); // Right Front actual velocity
  //   Serial.print(",");
  //   //Serial.print(m_left_back_velocity);    // Left Back target velocity
  //   //Serial.print(",");
  //   Serial.print(encoders.leftBackSpeed());   // Left Back actual velocity
  //   Serial.print(",");
  //   //Serial.print(m_right_back_velocity);   // Right Back target velocity
  //   //Serial.print(",");
  //   Serial.print(encoders.rightBackSpeed());  // Right Back actual velocity
  //   Serial.println();
  //}

}

void loop() {
  // Update test states and target velocities
  //hope.update();



  profiler.stop();
  motion.forward(1000);

  delay(2000);

  motion.forward(-1000);

  delay(2000);

  motion.rotate(3.14);

  delay(2000);

  motion.rotate(-3.14);

  delay(2000);

  motion.side(100);

  delay(2000);

  motion.side(-100);

  delay(2000);

  // profiler.setTarget(-100,0,0);
  // // delay(5000);
  // float distance = encoders.robotDistance();
  // Serial.println(distance);
  // while(distance-1000<encoders.robotDistance()){
  //   Serial.println(distance);
  //   delay(5);
  // }
  // profiler.stop();
  // delay(5000);
  // profiler.setTarget( 0,0,-0.314);
  // distance = encoders.robotAngle();
  // Serial.println(distance);
  // while(distance-3.14 <encoders.robotAngle()){
  //   Serial.println(distance);
  //   delay(5);
  // }
  // profiler.stop();
  // delay(5000);

  // profiler.setTarget(0, 0, -0.314);
  // delay(10000);
  // profiler.stop();
  // delay(1000);
  // profiler.setTarget(0,0,0.1);
  // delay(5000);
  // profiler.stop();


  // delay(1000);
  // profiler.setTarget(50,50,0.05);
  // delay(5000);
  // profiler.stop();
  
  // Print debug information every 100ms


  // static unsigned long lastPrintTime = 0;
  // if (millis() - lastPrintTime >= 100) {
  //   lastPrintTime = millis();
    
  //   Serial.println("\n----------------------------------------");
  //   hope.printEncoderData();
  //   hope.printMotorOutputs();
  //   Serial.println("----------------------------------------\n");
  // }
  
  //delay(10);  // Small delay to prevent overwhelming the serial output
}