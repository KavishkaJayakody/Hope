#pragma once

#include <Arduino.h>
#include "config.h"
#include "encoders.h"
//#include "analog.h"

class Motors;
// testng
extern Motors motors;

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
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);

    digitalWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR_IN1, 0);
    digitalWrite(LEFT_MOTOR_IN2, 0);
    digitalWrite(RIGHT_MOTOR_IN2, 0);
    digitalWrite(RIGHT_MOTOR_IN1, 0);
    digitalWrite(RIGHT_MOTOR_PWM, 0);
    setupPWM();
  }

  float getLeftSpeed(){
    return left_speed;
  }

  float getRightSpeed(){
    return right_speed;
  }
    float getLeftPercentage(){
    return left_output;

  }  float getRightPercentage(){
    return right_output;
  }

  void reset_controllers()
  {
    m_fwd_error = 0;
    m_rot_error = 0;
    m_previous_fwd_error = 0;
    m_previous_rot_error = 0;
  }

  void stop()
  {
    set_left_motor_percentage(0);
    set_right_motor_percentage(0);
  }

  float position_controller()
  {
    float increment = m_velocity * encoders.loopTime_s();
    m_fwd_error += increment - encoders.robot_fwd_change();
    float diff = m_fwd_error - m_previous_fwd_error;
    m_previous_fwd_error = m_fwd_error;

    // change them to config kp kd later
    float output = fwdKp * m_fwd_error + fwdKd * diff;
    return output;
  }

  float angle_controller(float steering_adjustment)
  {
    float increment = m_omega * encoders.loopTime_s();
    m_rot_error += increment - encoders.robot_rot_change();
    float diff = m_rot_error - m_previous_rot_error;
    m_previous_rot_error = m_rot_error;
    m_rot_error -= steering_adjustment;

    // Serial.print("Steering  :");
    // Serial.print(steering_adjustment);

    // changethis kp kd to config kp kd later
    float output = rotKp * m_rot_error + rotKd * diff;
    return output;
  }

  void update(float velocity, float omega, float steering)
  {
    m_velocity = velocity;
    m_omega = omega;

    float pos_output = position_controller();
    float rot_output = angle_controller(steering);

    left_output = 0;
    right_output = 0;

    left_output = pos_output - rot_output;
    right_output = pos_output + rot_output;

    float tangent_speed = m_omega * MOUSE_RADIUS * RADIANS_PER_DEGREE;
    left_speed = m_velocity - tangent_speed;
    right_speed = m_velocity + tangent_speed;
    float left_ff = left_feed_forward_percentage(left_speed);
    float right_ff = right_feed_forward_percentage(right_speed);
    if (m_feedforward_enabled)
    {
      left_output += left_ff;
      right_output += right_ff;
    }
    if (m_controller_output_enabled)
    {
      set_left_motor_percentage(left_output);
      set_right_motor_percentage(right_output);


      // Serial.print("  left  : ");
      // Serial.print(left_output);

      // Serial.print("   right  : ");
      // Serial.println(right_output);
    }
  }

  float left_feed_forward_percentage(float left_feed_velocity)
  {
    ///////give the percentage required to acheive a given velocity--- |v|<500
    //int l_rps = (left_feed_velocity * PULSES_PER_ROTATION) / MM_PER_ROTATION;
    //Serial.print("   left feed velocity: ");
    //Serial.println(left_feed_velocity);

    float delta_left_feed_velocity = left_feed_velocity - previous_left_velocity;//for acceleration feed forward calculation
    previous_left_velocity = left_feed_velocity;
    if (left_feed_velocity>0){
        float l_feed_percentage = 5 + 0.14*left_feed_velocity  + 0.15*delta_left_feed_velocity;
        return l_feed_percentage;
    }
    else{
        float l_feed_percentage = -5 + 0.14*left_feed_velocity + 0.15*delta_left_feed_velocity;
        return l_feed_percentage;
    }
  }

  float right_feed_forward_percentage(float right_feed_velocity)
  {
    ///////give the percentage required to acheive a given velocity--- |v|<500
    //int r_rps = (left_feed_velocity * PULSES_PER_ROTATION) / MM_PER_ROTATION;
    //Serial.print("   right feed velocity: ");
    //Serial.println(right_feed_velocity);
    float delta_right_feed_velocity = right_feed_velocity - previous_right_velocity;//for acceleration feed forward calculation
    previous_right_velocity = right_feed_velocity;
    if (right_feed_velocity>0){
        float r_feed_percentage = 5 + 0.14*right_feed_velocity + 0.15*delta_right_feed_velocity;
        return r_feed_percentage;
    }
    else{
        float r_feed_percentage = -5 + 0.14*right_feed_velocity + 0.15*delta_right_feed_velocity;
        return r_feed_percentage;
    }
  }

  void set_left_motor_percentage(float percentage)
  {
      percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
      if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
      {
          percentage = 0;
      }

      m_left_motor_percentage = percentage;
      int left_pwm = calculate_pwm(m_left_motor_percentage);
      //Serial.print("   Left pwm percentage: ");
      //Serial.print(percentage);

      set_left_motor_pwm(left_pwm);
  }

  void set_left_motor_pwm(int pwm)
  {
    pwm = MOTOR_LEFT_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm + M_BALNCE_PWM);
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
      ledcWrite(0, pwm);
    }
    else
    {
      pwm = batteryCompPWM(pwm + M_BALNCE_PWM);
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
      ledcWrite(0, pwm);
    }
  }

  int calculate_pwm(float desired_percentage)
  {
    int pwm = maxMotorPercentage * PWM_RESOLUTION * desired_percentage / 10000;
    return pwm;
  }

  void set_right_motor_percentage(float percentage)
  {
      percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
      if (percentage >= -MIN_MOTOR_PERCENTAGE && percentage <= MIN_MOTOR_PERCENTAGE)
      {
          percentage = 0;
      }

      m_right_motor_percentage = percentage;
      int right_pwm = calculate_pwm(m_right_motor_percentage);

      //Serial.print("   right pwm percentage: ");
      //Serial.println(percentage);
      
      set_right_motor_pwm(right_pwm);
  }

  void set_right_motor_pwm(int pwm)
  {
    pwm = MOTOR_RIGHT_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm - M_BALNCE_PWM);
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
      ledcWrite(1, pwm);
    }
    else
    {
      pwm = batteryCompPWM(pwm - M_BALNCE_PWM);
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      digitalWrite(RIGHT_MOTOR_IN2, HIGH);
      ledcWrite(1, pwm);
    }
  }

  void setupPWM()
  {
    ledcSetup(0, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(LEFT_MOTOR_PWM, 0);
    ledcSetup(1, 5000, PWM_RESOLUTION_BITS);
    ledcAttachPin(RIGHT_MOTOR_PWM, 1);
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
  float m_left_motor_percentage;
  float m_right_motor_percentage;

  float m_previous_fwd_error;
  float m_previous_rot_error;
  float m_fwd_error;
  float m_rot_error;

  float previous_left_velocity;
  float previous_right_velocity;

  float m_velocity;
  float m_omega;

  float left_speed;
  float right_speed;

  float left_output;
  float right_output;

  bool m_feedforward_enabled = true;
  bool m_controller_output_enabled;
  unsigned long i = 0;
};