#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>

// MPU6050 Register Addresses
#define MPU6050_ADDR            0x68
#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_GYRO_XOUT_H    0x43
#define MPU6050_ACCEL_YOUT_H   0x3D
#define MPU6050_ACCEL_ZOUT_H   0x3F

class MPU6050 {
public:
    MPU6050(uint8_t address = MPU6050_ADDR) : _address(address) {
        // Initialize filter parameters
        alpha = 0.96f;
        dt = 0.02f; // 50Hz update rate

        // Initialize Kalman filter parameters
        gyroXEstimateError = 0.1f;
        gyroXProcessNoise = 0.001f;
        gyroXMeasurementNoise = 0.1f;

        accYEstimateError = 0.1f;
        accYProcessNoise = 0.001f;
        accYMeasurementNoise = 0.1f;

        accZEstimateError = 0.1f;
        accZProcessNoise = 0.001f;
        accZMeasurementNoise = 0.1f;

        // Initialize filter variables
        gyroXPrev = accYPrev = accZPrev = 0.0f;
        gyroXFiltered = accYFiltered = accZFiltered = 0.0f;
        bufferIndex = 0;

        // Initialize velocities
        velocityY = velocityZ = 0.0f;

        // Clear buffers
        for (int i = 0; i < windowSize; i++) {
            gyroXBuffer[i] = accYBuffer[i] = accZBuffer[i] = 0.0f;
        }
    }

    void begin() {
        Wire.begin();
        
        // Wake up the MPU6050
        writeByte(MPU6050_PWR_MGMT_1, 0x00);
        
        // Configure sample rate (50Hz)
        writeByte(MPU6050_SMPLRT_DIV, 0x13);
        
        // Configure gyroscope (±250°/s)
        writeByte(MPU6050_GYRO_CONFIG, 0x00);
        
        // Configure accelerometer (±2g)
        writeByte(MPU6050_ACCEL_CONFIG, 0x00);
        
        // Set digital low-pass filter
        writeByte(MPU6050_CONFIG, 0x03);
    }

    void calibrate() {
        const int numSamples = 100;
        float sumGyroX = 0, sumAccY = 0, sumAccZ = 0;

        // Collect samples
        for (int i = 0; i < numSamples; i++) {
            readRawData();
            sumGyroX += gyroX;
            sumAccY += accY;
            sumAccZ += accZ;
            delay(10);
        }

        // Calculate average biases
        gyroXBias = sumGyroX / numSamples;
        accYBias = sumAccY / numSamples;
        accZBias = sumAccZ / numSamples;

        // Initialize estimates
        gyroXEstimate = 0;
        accYEstimate = 0;
        accZEstimate = 0;
    }

    void update() {
        readRawData();
        applyKalmanFilter();
        applyHighPassFilter();
        applyMovingAverageFilter();

        // Update linear velocities using trapezoidal integration
        velocityY += (accYFiltered + accYPrev) * dt / 2.0f;
        velocityZ += (accZFiltered + accZPrev) * dt / 2.0f;

        // Store previous values
        accYPrev = accYFiltered;
        accZPrev = accZFiltered;
    }

    float getAngularVelocityX() { return gyroXFiltered; }
    float getLinearVelocityY() { return velocityY; }
    float getLinearVelocityZ() { return velocityZ; }

private:
    uint8_t _address;                                // I2C address of the MPU6050
    float gyroX, accY, accZ;                        // Raw sensor data
    float gyroXBias, accYBias, accZBias;            // Sensor biases
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

    void writeByte(uint8_t reg, uint8_t data) {
        Wire.beginTransmission(_address);
        Wire.write(reg);
        Wire.write(data);
        Wire.endTransmission();
    }

    int16_t readWord(uint8_t reg) {
        Wire.beginTransmission(_address);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(_address, (uint8_t)2);
        return (Wire.read() << 8) | Wire.read();
    }

    void readRawData() {
        // Read raw sensor data
        gyroX = readWord(MPU6050_GYRO_XOUT_H) / 131.0f - gyroXBias;  // Convert to deg/s
        accY = readWord(MPU6050_ACCEL_YOUT_H) / 16384.0f - accYBias; // Convert to g
        accZ = readWord(MPU6050_ACCEL_ZOUT_H) / 16384.0f - accZBias; // Convert to g

        // Convert accelerometer readings to m/s²
        accY *= 9.81f;
        accZ *= 9.81f;
    }

    void applyKalmanFilter() {
        // Gyro X Kalman filter
        float gyroXPrediction = gyroXEstimate;
        gyroXEstimateError += gyroXProcessNoise;

        float gyroXKalmanGain = gyroXEstimateError / (gyroXEstimateError + gyroXMeasurementNoise);
        gyroXEstimate = gyroXPrediction + gyroXKalmanGain * (gyroX - gyroXPrediction);
        gyroXEstimateError = (1 - gyroXKalmanGain) * gyroXEstimateError;

        // Similar process for accelerometer Y and Z
        float accYPrediction = accYEstimate;
        accYEstimateError += accYProcessNoise;

        float accYKalmanGain = accYEstimateError / (accYEstimateError + accYMeasurementNoise);
        accYEstimate = accYPrediction + accYKalmanGain * (accY - accYPrediction);
        accYEstimateError = (1 - accYKalmanGain) * accYEstimateError;

        float accZPrediction = accZEstimate;
        accZEstimateError += accZProcessNoise;

        float accZKalmanGain = accZEstimateError / (accZEstimateError + accZMeasurementNoise);
        accZEstimate = accZPrediction + accZKalmanGain * (accZ - accZPrediction);
        accZEstimateError = (1 - accZKalmanGain) * accZEstimateError;
    }

    void applyHighPassFilter() {
        // Apply high-pass filter to remove drift
        gyroXFiltered = alpha * (gyroXFiltered + gyroXEstimate - gyroXPrev);
        accYFiltered = alpha * (accYFiltered + accYEstimate - accYPrev);
        accZFiltered = alpha * (accZFiltered + accZEstimate - accZPrev);

        gyroXPrev = gyroXEstimate;
        accYPrev = accYEstimate;
        accZPrev = accZEstimate;
    }

    void applyMovingAverageFilter() {
        // Apply moving average filter to smooth the data
        gyroXFiltered = movingAverage(gyroXFiltered, gyroXBuffer, bufferIndex);
        accYFiltered = movingAverage(accYFiltered, accYBuffer, bufferIndex);
        accZFiltered = movingAverage(accZFiltered, accZBuffer, bufferIndex);

        bufferIndex = (bufferIndex + 1) % windowSize;
    }

    float movingAverage(float newValue, float buffer[], int &index) {
        buffer[index] = newValue;
        float sum = 0;
        for (int i = 0; i < windowSize; i++) {
            sum += buffer[i];
        }
        return sum / windowSize;
    }
};

#endif // MPU6050_H