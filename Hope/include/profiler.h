#ifndef PROFILER_H
#define PROFILER_H

#include "config.h"
#include "Arduino.h"
#include "encoders.h"


class Profiler;
extern Profiler profiler;

class Profiler {

public:


    void setTarget(float vy, float vx, float w) {
        target_x = vx;
        target_y = vy;
        target_omega = w;
        x_reached = false;
        y_reached = false;
        omega_reached = false;
    }

    void update() {
        // Update X velocity
        float dt = encoders.loopTime_s();
        if (!x_reached) {
            if (fabs(x_velocity - target_x) <= acceleration * dt) {
                x_velocity = target_x;
                x_reached = true;
            } else {
                x_velocity += (target_x > x_velocity ? acceleration : -acceleration) * dt;
            }
        }
        
        // Update Y velocity
        if (!y_reached) {
            if (fabs(y_velocity - target_y) <= acceleration * dt) {
                y_velocity = target_y;
                y_reached = true;
            } else {
                y_velocity += (target_y > y_velocity ? acceleration : -acceleration) * dt;
            }
        }
        
        // Update Omega velocity
        if (!omega_reached) {
            if (fabs(omega - target_omega) <= angular_acc * dt) {
                omega = target_omega;
                omega_reached = true;
            } else {
                omega += (target_omega > omega ? angular_acc : -angular_acc) * dt;
            }
        }
    }

    void stop() {
        x_velocity = 0;
        y_velocity = 0;
        omega = 0;
        x_reached = true;
        y_reached = true;
        omega_reached = true;
    }

    float X_Velocity() const { 
        return x_velocity; 
        }
    float Y_Velocity() const { 
        return y_velocity; 
        }
    float Omega() const { 
        return omega; 
        }


    private:
    float acceleration = LINEAR_ACC; // Acceleration value (same for all directions)
    float angular_acc = ANGULAR_ACC;
    float x_velocity = 0, y_velocity = 0, omega = 0; // Current velocities
    float target_x = 0, target_y = 0, target_omega = 0; // Target velocities
    bool x_reached = true, y_reached = true, omega_reached = true; // Flags for tracking targets
};
#endif // PROFILER_H