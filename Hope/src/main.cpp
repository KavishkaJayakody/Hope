#include <Arduino.h>
#include "motors.h"
#include "Ticker.h"


Encoders encoders;
Motors motors;

Ticker updateTicker;


void setup(){
      Serial.begin(115200, SERIAL_8N1);
      encoders.begin();
      encoders.reset();
      motors.begin();
      updateTicker.attach(0.002, []()
                    {
                      encoders.update();
                      motors.update(0, 0, 0);
                    }
        );

}

void loop(){
  Serial.print("Left Encoder :");
  Serial.print(encoders.leftRPS());
  Serial.print(" Right Encoder :");
  Serial.print(encoders.rightRPS());
  

}