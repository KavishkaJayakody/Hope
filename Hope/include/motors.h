#pragma once

#include <Arduino.h>
#include "config.h"
#include "encoders.h"
//#include "analog.h"

class Motors;
extern Motors motors;

// PWM Channel definitions
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

// Control mode selection
enum ControlMode {
    INDIVIDUAL_WHEELS,  // Original mode: control each wheel independently
    SYSTEM_CONTROL      // New mode: control robot as a whole system
};

class Motors
{
public:
  // Control parameters - remove after testing
  float fwdKp = FWD_KP;
  float fwdKd = FWD_KD;
  float rotKp = ROT_KP;
  float rotKd = ROT_KD;
  float sideKp = FWD_KP;  // Added for side movement control
  float sideKd = FWD_KD;  // Added for side movement control

  float maxMotorPercentage = MAX_MOTOR_PERCENTAGE_FINAL;
  
  // System control mode selection
  ControlMode controlMode = SYSTEM_CONTROL;

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
    setupPWM();
  }

  float getLeftFrontSpeed() { return left_front_speed; }
  float getLeftBackSpeed() { return left_back_speed; }
  float getRightFrontSpeed() { return right_front_speed; }
  float getRightBackSpeed() { return right_back_speed; }

  float getLeftFrontPercentage() { return left_front_output; }
  float getLeftBackPercentage() { return left_back_output; }
  float getRightFrontPercentage() { return right_front_output; }
  float getRightBackPercentage() { return right_back_output; }

  void setControlMode(ControlMode mode) { controlMode = mode; }
  ControlMode getControlMode() { return controlMode; }

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
    
    // Reset system control errors
    m_forward_error = 0;
    m_side_error = 0;
    m_rotation_error = 0;
    m_previous_forward_error = 0;
    m_previous_side_error = 0;
    m_previous_rotation_error = 0;
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

  // Original per-wheel update
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

  // System controllers for omnidirectional movement
  float forward_controller(float forward_velocity)
  {
    // Average of all wheel speeds to estimate robot forward movement
    float actual_forward_speed = (encoders.leftFrontSpeed() + encoders.leftBackSpeed() + 
                                 encoders.rightFrontSpeed() + encoders.rightBackSpeed()) / 4.0;
    
    float increment = forward_velocity * encoders.loopTime_s();
    m_forward_error += increment - actual_forward_speed * encoders.loopTime_s();
    float diff = m_forward_error - m_previous_forward_error;
    m_previous_forward_error = m_forward_error;
    
    return fwdKp * m_forward_error + fwdKd * diff;
  }

  float side_controller(float side_velocity)
  {
    // Estimate side movement from wheel encoders
    // For mecanum wheels, side movement causes diagonal wheels to move in same direction
    float actual_side_speed = ((encoders.leftFrontSpeed() - encoders.leftBackSpeed()) + 
                              (encoders.rightBackSpeed() - encoders.rightFrontSpeed())) / 4.0;
    
    float increment = side_velocity * encoders.loopTime_s();
    m_side_error += increment - actual_side_speed * encoders.loopTime_s();
    float diff = m_side_error - m_previous_side_error;
    m_previous_side_error = m_side_error;
    
    return sideKp * m_side_error + sideKd * diff;
  }

  float rotation_controller(float omega)
  {
    // Estimate rotation from wheel encoders
    // Left wheels positive, right wheels negative for clockwise rotation
    float actual_rotation_speed = (-(encoders.leftFrontSpeed() + encoders.leftBackSpeed())+
                                  (encoders.rightFrontSpeed() + encoders.rightBackSpeed())) / (4.0*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS));
    
    float increment = omega * encoders.loopTime_s();
    m_rotation_error += increment - actual_rotation_speed * encoders.loopTime_s();
    float diff = m_rotation_error - m_previous_rotation_error;
    m_previous_rotation_error = m_rotation_error;
    
    return rotKp * m_rotation_error + rotKd * diff;
  }

  // New system-level update for omnidirectional control
  void updateSystem(float forward_velocity, float side_velocity, float omega, 
                    float side_adjustment = 0, float rotation_adjustment = 0)
  {
    // Store desired velocities
    m_forward_velocity = forward_velocity;
    m_side_velocity = side_velocity;  //To convert to standard coordinate system
    m_omega = omega;
    
    // Calculate control outputs for each movement dimension
    float forward_output = forward_controller(forward_velocity);
    float side_output = side_controller(side_velocity) + side_adjustment;
    float rotation_output = rotation_controller(omega) + rotation_adjustment;
    
    // Convert to individual wheel velocities based on mecanum wheel kinematics
    // For mecanum wheels:
    // LF = forward + side + rotation
    // RF = forward - side - rotation
    // LB = forward - side + rotation
    // RB = forward + side - rotation
    
    m_left_front_velocity = forward_velocity + side_velocity - omega*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    m_right_front_velocity = forward_velocity - side_velocity + omega*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    m_left_back_velocity = forward_velocity - side_velocity - omega*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    m_right_back_velocity = forward_velocity + side_velocity + omega*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    
    // Calculate output for each wheel
    left_front_output = forward_output + side_output - rotation_output*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    right_front_output = forward_output - side_output + rotation_output*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    left_back_output = forward_output - side_output - rotation_output*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    right_back_output = forward_output + side_output + rotation_output*(ROBOT_X_RADIUS+ROBOT_Y_RADIUS);
    
    // Store speeds for telemetry
    left_front_speed = m_left_front_velocity;
    left_back_speed = m_left_back_velocity;
    right_front_speed = m_right_front_velocity;
    right_back_speed = m_right_back_velocity;
    
    // Apply feedforward
    if (m_feedforward_enabled)
    {
      left_front_output += feed_forward_percentage(left_front_speed);
      left_back_output += feed_forward_percentage(left_back_speed);
      right_front_output += feed_forward_percentage(right_front_speed);
      right_back_output += feed_forward_percentage(right_back_speed);
    }
    
    // Apply to motors
    if (m_controller_output_enabled)
    {
      set_left_front_motor_percentage(left_front_output);
      set_left_back_motor_percentage(left_back_output);
      set_right_front_motor_percentage(right_front_output);
      set_right_back_motor_percentage(right_back_output);
    }
  }
  
  // Main update function that selects the control mode
  void updateMotors(float arg1, float arg2, float arg3, float arg4, float arg5 = 0)
  {
    if (controlMode == INDIVIDUAL_WHEELS) {
      // In individual mode, use first 4 args as individual wheel velocities
      update(arg1, arg2, arg3, arg4);
    } else {
      // In system mode, args are: forward_velocity, side_velocity, omega, side_adjustment, rotation_adjustment
      updateSystem(arg1, arg2, arg3, arg4, arg5);
    }
  }

  float feed_forward_percentage(float velocity)
  {
    if (velocity > 5) {
      return 40.0+0.001*velocity*velocity;//5 + 0.14 * velocity;//
    }
    else if (velocity > -5) {
      return 0;//5 + 0.14 * velocity;
    }
     else {
      return -30-0.001*velocity*velocity;//-5 + 0.14 * velocity;//
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
      ledcWrite(LEFT_FRONT_IN1_CHANNEL, pwm);
      ledcWrite(LEFT_FRONT_IN2_CHANNEL, 0);
      // For digital control (commented out):
      // digitalWrite(LEFT_FRONT_MOTOR_IN1, HIGH);
      // digitalWrite(LEFT_FRONT_MOTOR_IN2, LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm + M_BALNCE_PWM);
      ledcWrite(LEFT_FRONT_IN1_CHANNEL, 0);
      ledcWrite(LEFT_FRONT_IN2_CHANNEL, pwm);
      // For digital control (commented out):
      // digitalWrite(LEFT_FRONT_MOTOR_IN1, LOW);
      // digitalWrite(LEFT_FRONT_MOTOR_IN2, HIGH);
    }
  }

  void set_left_back_motor_pwm(int pwm)
  {
    pwm = MOTOR_LEFT_BACK_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm + M_BALNCE_PWM);
      ledcWrite(LEFT_BACK_IN1_CHANNEL, pwm);
      ledcWrite(LEFT_BACK_IN2_CHANNEL, 0);
      // For digital control (commented out):
      // digitalWrite(LEFT_BACK_MOTOR_IN1, HIGH);
      // digitalWrite(LEFT_BACK_MOTOR_IN2, LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm + M_BALNCE_PWM);
      ledcWrite(LEFT_BACK_IN1_CHANNEL, 0);
      ledcWrite(LEFT_BACK_IN2_CHANNEL, pwm);
      // For digital control (commented out):
      // digitalWrite(LEFT_BACK_MOTOR_IN1, LOW);
      // digitalWrite(LEFT_BACK_MOTOR_IN2, HIGH);
    }
  }

  void set_right_front_motor_pwm(int pwm)
  {
    pwm = MOTOR_RIGHT_FRONT_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm - M_BALNCE_PWM);
      ledcWrite(RIGHT_FRONT_IN1_CHANNEL, pwm);
      ledcWrite(RIGHT_FRONT_IN2_CHANNEL, 0);
      // For digital control (commented out):
      // digitalWrite(RIGHT_FRONT_MOTOR_IN1, HIGH);
      // digitalWrite(RIGHT_FRONT_MOTOR_IN2, LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm - M_BALNCE_PWM);
      ledcWrite(RIGHT_FRONT_IN1_CHANNEL, 0);
      ledcWrite(RIGHT_FRONT_IN2_CHANNEL, pwm);
      // For digital control (commented out):
      // digitalWrite(RIGHT_FRONT_MOTOR_IN1, LOW);
      // digitalWrite(RIGHT_FRONT_MOTOR_IN2, HIGH);
    }
  }

  void set_right_back_motor_pwm(int pwm)
  {
    pwm = MOTOR_RIGHT_BACK_POLARITY * pwm;
    if (pwm < 0)
    {
      pwm = batteryCompPWM(-pwm - M_BALNCE_PWM);
      ledcWrite(RIGHT_BACK_IN1_CHANNEL, pwm);
      ledcWrite(RIGHT_BACK_IN2_CHANNEL, 0);
      // For digital control (commented out):
      // digitalWrite(RIGHT_BACK_MOTOR_IN1, HIGH);
      // digitalWrite(RIGHT_BACK_MOTOR_IN2, LOW);
    }
    else
    {
      pwm = batteryCompPWM(pwm - M_BALNCE_PWM);
      ledcWrite(RIGHT_BACK_IN1_CHANNEL, 0);
      ledcWrite(RIGHT_BACK_IN2_CHANNEL, pwm);
      // For digital control (commented out):
      // digitalWrite(RIGHT_BACK_MOTOR_IN1, LOW);
      // digitalWrite(RIGHT_BACK_MOTOR_IN2, HIGH);
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

  // Individual wheel control errors
  float m_previous_left_front_error;
  float m_previous_left_back_error;
  float m_previous_right_front_error;
  float m_previous_right_back_error;

  float m_left_front_error;
  float m_left_back_error;
  float m_right_front_error;
  float m_right_back_error;

  // Individual wheel velocities
  float m_left_front_velocity;
  float m_left_back_velocity;
  float m_right_front_velocity;
  float m_right_back_velocity;

  // System control velocities
  float m_forward_velocity;
  float m_side_velocity;
  float m_omega;
  
  // System control errors
  float m_forward_error;
  float m_side_error;
  float m_rotation_error;
  float m_previous_forward_error;
  float m_previous_side_error;
  float m_previous_rotation_error;

  // Speed and output tracking
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