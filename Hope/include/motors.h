#pragma once

#include <Arduino.h>
#include "config.h"
#include "encoders.h"
//#include "analog.h"

class Motors;
// testng
extern Motors motors;

enum {
    LEFT_FRONT_IN1_CHANNEL,
    LEFT_FRONT_IN2_CHANNEL,
    LEFT_BACK_IN1_CHANNEL,
    LEFT_BACK_IN2_CHANNEL,
    RIGHT_FRONT_IN1_CHANNEL,
    RIGHT_FRONT_IN2_CHANNEL,
    RIGHT_BACK_IN1_CHANNEL,
    RIGHT_BACK_IN2_CHANNEL
};

class Motors
{
public:
  // remove after testing
  float fwdKp = FWD_KP_SMALL;
  float fwdKd = FWD_KD_SMALL;
  float rotKp = ROT_KP_90;
  float rotKd = ROT_KD_90;

  float maxMotorPercentage = MAX_MOTOR_PERCENTAGE_FINAL;
  // remove

  void begin()
  {
    pinMode(LEFT_FRONT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_FRONT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_BACK_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_BACK_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_FRONT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_FRONT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_BACK_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_BACK_MOTOR_IN2, OUTPUT);

    digitalWrite(LEFT_FRONT_MOTOR_IN1, 0);
    digitalWrite(LEFT_FRONT_MOTOR_IN2, 0);
    digitalWrite(LEFT_BACK_MOTOR_IN1, 0);
    digitalWrite(LEFT_BACK_MOTOR_IN2, 0);
    digitalWrite(RIGHT_FRONT_MOTOR_IN1, 0);
    digitalWrite(RIGHT_FRONT_MOTOR_IN2, 0);
    digitalWrite(RIGHT_BACK_MOTOR_IN1, 0);
    digitalWrite(RIGHT_BACK_MOTOR_IN2, 0);
    ///////////////////////////////////////////////////////////////////////////////////setupPWM();
  }

  float getLeftFrontSpeed() { return left_front_speed; }
  float getLeftBackSpeed() { return left_back_speed; }
  float getRightFrontSpeed() { return right_front_speed; }
  float getRightBackSpeed() { return right_back_speed; }

  float getLeftFrontPercentage() { return left_front_output; }
  float getLeftBackPercentage() { return left_back_output; }
  float getRightFrontPercentage() { return right_front_output; }
  float getRightBackPercentage() { return right_back_output; }

  void reset_controllers()
  {
    m_left_front_error = 0;
    m_left_back_error = 0;
    m_right_front_error = 0;
    m_right_back_error = 0;
    m_previous_left_front_error = 0;
    m_previous_left_back_error = 0;
    m_previous_right_front_error = 0;
    m_previous_right_back_error = 0;
  }

  void stop()
  {
    set_left_front_motor_percentage(0);
    set_left_back_motor_percentage(0);
    set_right_front_motor_percentage(0);
    set_right_back_motor_percentage(0);
  }

  float left_front_controller()
  {
    float increment = m_left_front_velocity * encoders.loopTime_s();
    m_left_front_error += increment - encoders.leftFrontSpeed() * encoders.loopTime_s();
    float diff = m_left_front_error - m_previous_left_front_error;
    m_previous_left_front_error = m_left_front_error;
    return fwdKp * m_left_front_error + fwdKd * diff;
  }

  float left_back_controller()
  {
    float increment = m_left_back_velocity * encoders.loopTime_s();
    m_left_back_error += increment - encoders.leftBackSpeed() * encoders.loopTime_s();
    float diff = m_left_back_error - m_previous_left_back_error;
    m_previous_left_back_error = m_left_back_error;
    return fwdKp * m_left_back_error + fwdKd * diff;
  }

  float right_front_controller()
  {
    float increment = m_right_front_velocity * encoders.loopTime_s();
    m_right_front_error += increment - encoders.rightFrontSpeed() * encoders.loopTime_s();
    float diff = m_right_front_error - m_previous_right_front_error;
    m_previous_right_front_error = m_right_front_error;
    return fwdKp * m_right_front_error + fwdKd * diff;
  }

  float right_back_controller()
  {
    float increment = m_right_back_velocity * encoders.loopTime_s();
    m_right_back_error += increment - encoders.rightBackSpeed() * encoders.loopTime_s();
    float diff = m_right_back_error - m_previous_right_back_error;
    m_previous_right_back_error = m_right_back_error;
    return fwdKp * m_right_back_error + fwdKd * diff;
  }

  void update(float left_front_velocity, float left_back_velocity, float right_front_velocity, float right_back_velocity)
  {
    m_left_front_velocity = left_front_velocity;
    m_left_back_velocity = left_back_velocity;
    m_right_front_velocity = right_front_velocity;
    m_right_back_velocity = right_back_velocity;

    left_front_output = left_front_controller();
    left_back_output = left_back_controller();
    right_front_output = right_front_controller();
    right_back_output = right_back_controller();

    left_front_speed = m_left_front_velocity;
    left_back_speed = m_left_back_velocity;
    right_front_speed = m_right_front_velocity;
    right_back_speed = m_right_back_velocity;

    float left_front_ff = feed_forward_percentage(left_front_speed);
    float left_back_ff = feed_forward_percentage(left_back_speed);
    float right_front_ff = feed_forward_percentage(right_front_speed);
    float right_back_ff = feed_forward_percentage(right_back_speed);

    if (m_feedforward_enabled)
    {
      left_front_output += left_front_ff;
      left_back_output += left_back_ff;
      right_front_output += right_front_ff;
      right_back_output += right_back_ff;
    }

    if (m_controller_output_enabled)
    {
      set_left_front_motor_percentage(left_front_output);
      set_left_back_motor_percentage(left_back_output);
      set_right_front_motor_percentage(right_front_output);
      set_right_back_motor_percentage(right_back_output);
    }
  }

  float feed_forward_percentage(float velocity)
  {
    if (velocity > 0) {
      return 5 + 0.14 * velocity;
    } else {
      return -5 + 0.14 * velocity;
    }
  }

  void set_left_front_motor_percentage(float percentage)
  {
    percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
    if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
    {
      percentage = 0;
    }
    m_left_front_motor_percentage = percentage;
    int pwm = calculate_pwm(percentage);
    set_left_front_motor_pwm(pwm);
  }

  void set_left_back_motor_percentage(float percentage)
  {
    percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
    if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
    {
      percentage = 0;
    }
    m_left_back_motor_percentage = percentage;
    int pwm = calculate_pwm(percentage);
    set_left_back_motor_pwm(pwm);
  }

  void set_right_front_motor_percentage(float percentage)
  {
    percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
    if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
    {
      percentage = 0;
    }
    m_right_front_motor_percentage = percentage;
    int pwm = calculate_pwm(percentage);
    set_right_front_motor_pwm(pwm);
  }

  void set_right_back_motor_percentage(float percentage)
  {
    percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
    if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
    {
      percentage = 0;
    }
    m_right_back_motor_percentage = percentage;
    int pwm = calculate_pwm(percentage);
    set_right_back_motor_pwm(pwm);
  }

  void set_left_front_motor_pwm(int pwm)
  {
    pwm = MOTOR_LEFT_FRONT_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm + M_BALNCE_PWM);
      // ledcWrite(LEFT_FRONT_IN1_CHANNEL, pwm);
      // ledcWrite(LEFT_FRONT_IN2_CHANNEL, 0);

      digitalWrite(LEFT_FRONT_IN1_CHANNEL,HIGH);
      digitalWrite(LEFT_FRONT_IN2_CHANNEL,LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm + M_BALNCE_PWM);
      // ledcWrite(LEFT_FRONT_IN1_CHANNEL, 0);
      // ledcWrite(LEFT_FRONT_IN2_CHANNEL, pwm);

      digitalWrite(LEFT_FRONT_IN1_CHANNEL,HIGH);
      digitalWrite(LEFT_FRONT_IN2_CHANNEL,LOW);
    }
  }

  void set_left_back_motor_pwm(int pwm)
  {
    pwm = MOTOR_LEFT_BACK_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm + M_BALNCE_PWM);
      // ledcWrite(LEFT_BACK_IN1_CHANNEL, pwm);
      // ledcWrite(LEFT_BACK_IN2_CHANNEL, 0);

      digitalWrite(LEFT_BACK_IN1_CHANNEL,HIGH);
      digitalWrite(LEFT_BACK_IN2_CHANNEL,LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm + M_BALNCE_PWM);
      // ledcWrite(LEFT_BACK_IN1_CHANNEL, 0);
      // ledcWrite(LEFT_BACK_IN2_CHANNEL, pwm);

      digitalWrite(LEFT_BACK_IN1_CHANNEL,HIGH);
      digitalWrite(LEFT_BACK_IN2_CHANNEL,LOW);
    }
  }

  void set_right_front_motor_pwm(int pwm)
  {
    pwm = MOTOR_RIGHT_FRONT_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm - M_BALNCE_PWM);
      // ledcWrite(RIGHT_FRONT_IN1_CHANNEL, pwm);
      // ledcWrite(RIGHT_FRONT_IN2_CHANNEL, 0);

      digitalWrite(RIGHT_FRONT_IN1_CHANNEL,HIGH);
      digitalWrite(RIGHT_FRONT_IN2_CHANNEL,LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm - M_BALNCE_PWM);
      // ledcWrite(RIGHT_FRONT_IN1_CHANNEL, 0);
      // ledcWrite(RIGHT_FRONT_IN2_CHANNEL, pwm);
      digitalWrite(RIGHT_FRONT_IN1_CHANNEL,HIGH);
      digitalWrite(RIGHT_FRONT_IN2_CHANNEL,LOW);
    }
  }

  void set_right_back_motor_pwm(int pwm)
  {
    pwm = MOTOR_RIGHT_BACK_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm - M_BALNCE_PWM);
      // ledcWrite(RIGHT_BACK_IN1_CHANNEL, pwm);
      // ledcWrite(RIGHT_BACK_IN2_CHANNEL, 0);

      digitalWrite(RIGHT_BACK_IN1_CHANNEL,HIGH);
      digitalWrite(RIGHT_BACK_IN2_CHANNEL,LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm - M_BALNCE_PWM);
      // ledcWrite(RIGHT_BACK_IN1_CHANNEL, 0);
      // ledcWrite(RIGHT_BACK_IN2_CHANNEL, pwm);

      digitalWrite(RIGHT_BACK_IN1_CHANNEL,HIGH);
      digitalWrite(RIGHT_BACK_IN2_CHANNEL,LOW);
    }
  }

  int calculate_pwm(float desired_percentage)
  {
    return maxMotorPercentage * PWM_RESOLUTION * desired_percentage / 10000;
  }

  void setupPWM()
  {
    ledcSetup(LEFT_FRONT_IN1_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_FRONT_MOTOR_IN1, LEFT_FRONT_IN1_CHANNEL);
    ledcSetup(LEFT_FRONT_IN2_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_FRONT_MOTOR_IN2, LEFT_FRONT_IN2_CHANNEL);

    ledcSetup(LEFT_BACK_IN1_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_BACK_MOTOR_IN1, LEFT_BACK_IN1_CHANNEL);
    ledcSetup(LEFT_BACK_IN2_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_BACK_MOTOR_IN2, LEFT_BACK_IN2_CHANNEL);

    ledcSetup(RIGHT_FRONT_IN1_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(RIGHT_FRONT_MOTOR_IN1, RIGHT_FRONT_IN1_CHANNEL);
    ledcSetup(RIGHT_FRONT_IN2_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(RIGHT_FRONT_MOTOR_IN2, RIGHT_FRONT_IN2_CHANNEL);

    ledcSetup(RIGHT_BACK_IN1_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(RIGHT_BACK_MOTOR_IN1, RIGHT_BACK_IN1_CHANNEL);
    ledcSetup(RIGHT_BACK_IN2_CHANNEL, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(RIGHT_BACK_MOTOR_IN2, RIGHT_BACK_IN2_CHANNEL);
  }

  int batteryCompPWM(int pwm) {
    float volts = 7.65; //analog.batteryVoltage();
    int adjustedPWM = pwm * NOMINAL_BATTERY_V / volts;
    if (adjustedPWM>PWM_RESOLUTION){
      adjustedPWM = PWM_RESOLUTION;
    }
    return adjustedPWM;
  }

  void enable_controllers()
  {
    m_controller_output_enabled = true;
  }

  void disable_controllers()
  {
    m_controller_output_enabled = false;
  }

private:
  float m_left_front_motor_percentage;
  float m_left_back_motor_percentage;
  float m_right_front_motor_percentage;
  float m_right_back_motor_percentage;

  float m_previous_left_front_error;
  float m_previous_left_back_error;
  float m_previous_right_front_error;
  float m_previous_right_back_error;

  float m_left_front_error;
  float m_left_back_error;
  float m_right_front_error;
  float m_right_back_error;

  float m_left_front_velocity;
  float m_left_back_velocity;
  float m_right_front_velocity;
  float m_right_back_velocity;

  float left_front_speed;
  float left_back_speed;
  float right_front_speed;
  float right_back_speed;

  float left_front_output;
  float left_back_output;
  float right_front_output;
  float right_back_output;

  bool m_feedforward_enabled = true;
  bool m_controller_output_enabled;
  unsigned long i = 0;
};