#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

class MPU6050
{
public:
    MPU6050(uint8_t address = 0x68); // Constructor with default I2C address
    void begin();                    // Initialize the sensor
    void calibrate();                // Calibrate the sensor
    void update();                   // Read and process sensor data
    float getAngularVelocityX();     // Get filtered angular velocity (X-axis)
    float getLinearVelocityY();      // Get linear velocity (Y-axis)
    float getLinearVelocityZ();      // Get linear velocity (Z-axis)

private:
    uint8_t _address;                                // I2C address of the MPU6050
    float gyroX, accY, accZ;                         // Raw sensor data
    float gyroXBias, accYBias, accZBias;             // Sensor biases
    float gyroXEstimate, accYEstimate, accZEstimate; // Kalman filter estimates
    float velocityY, velocityZ;                      // Linear velocities
    float dt;                                        // Time interval for updates

    // Kalman filter variables
    float gyroXEstimateError, gyroXProcessNoise, gyroXMeasurementNoise;
    float accYEstimateError, accYProcessNoise, accYMeasurementNoise;
    float accZEstimateError, accZProcessNoise, accZMeasurementNoise;

    // High-pass filter variables
    float alpha;
    float gyroXPrev, accYPrev, accZPrev;
    float gyroXFiltered, accYFiltered, accZFiltered;

    // Moving average filter variables
    static const int windowSize = 10;
    float gyroXBuffer[windowSize], accYBuffer[windowSize], accZBuffer[windowSize];
    int bufferIndex;

    void readRawData();                                              // Read raw data from the sensor
    void applyKalmanFilter();                                        // Apply Kalman filter
    void applyHighPassFilter();                                      // Apply high-pass filter
    void applyMovingAverageFilter();                                 // Apply moving average filter
    float movingAverage(float newValue, float buffer[], int &index); // Helper function
};

#endif