#include "MPU6050.h"

MPU6050::MPU6050(uint8_t address) : _address(address)
{
    // Initialize variables
    gyroX = accY = accZ = 0.0f;
    gyroXBias = accYBias = accZBias = 0.0f;
    gyroXEstimate = accYEstimate = accZEstimate = 0.0f;
    velocityY = velocityZ = 0.0f;
    dt = 0.01f;

    // Kalman filter initialization
    gyroXEstimateError = accYEstimateError = accZEstimateError = 0.1f;
    gyroXProcessNoise = accYProcessNoise = accZProcessNoise = 0.01f;
    gyroXMeasurementNoise = accYMeasurementNoise = accZMeasurementNoise = 0.1f;

    // High-pass filter initialization
    alpha = 0.9f;
    gyroXPrev = accYPrev = accZPrev = 0.0f;
    gyroXFiltered = accYFiltered = accZFiltered = 0.0f;

    // Moving average filter initialization
    bufferIndex = 0;
    for (int i = 0; i < windowSize; i++)
    {
        gyroXBuffer[i] = accYBuffer[i] = accZBuffer[i] = 0.0f;
    }
}

void MPU6050::begin()
{
    Wire.begin();
    Wire.beginTransmission(_address);
    Wire.write(0x6B); // Power management register
    Wire.write(0);    // Wake up the MPU6050
    Wire.endTransmission(true);
    calibrate();
}

void MPU6050::calibrate()
{
    long gyroXSum = 0, accYSum = 0, accZSum = 0;
    int numSamples = 1000;

    for (int i = 0; i < numSamples; i++)
    {
        if (i < 200)
        {
            delay(10);
            readRawData();
            continue; // Skip the first 200 samples to allow the sensor to stabilize
        }
        delay(10);
        readRawData();
        gyroXSum += gyroX;
        accYSum += accY;
        accZSum += accZ;
    }

    gyroXBias = gyroXSum / 800.0;
    accYBias = accYSum / 800.0;
    accZBias = accZSum / 800.0;
}

void MPU6050::update()
{
    readRawData();
    applyMovingAverageFilter();
    applyHighPassFilter();
    applyKalmanFilter();

    // Integrate accelerometer data to get linear velocities
    velocityY += accYFiltered * dt;
    velocityZ += accZFiltered * dt;
}

float MPU6050::getAngularVelocityX()
{
    return gyroXEstimate;
}

float MPU6050::getLinearVelocityY()
{
    return velocityY;
}

float MPU6050::getLinearVelocityZ()
{
    return velocityZ;
}

void MPU6050::readRawData()
{
    Wire.beginTransmission(_address);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, 14, true);

    // Read accelerometer data
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    int16_t Tmp = Wire.read() << 8 | Wire.read();

    // Read gyroscope data (we only need X)
    int16_t GyX = Wire.read() << 8 | Wire.read();
    Wire.read(); // Skip GyY
    Wire.read();
    Wire.read(); // Skip GyZ
    Wire.read();

    // Convert raw gyroscope data to degrees per second
    gyroX = GyX / 131.0;

    // Convert raw accelerometer data to m/s²
    accY = AcY / 16384.0 * 9.81; // Convert to m/s²
    accZ = AcZ / 16384.0 * 9.81;
}

void MPU6050::applyKalmanFilter()
{
    // Kalman filter for gyroscope X
    float gyroXPrediction = gyroXEstimate;
    float gyroXPredictionError = gyroXEstimateError + gyroXProcessNoise;

    float gyroXKalmanGain = gyroXPredictionError / (gyroXPredictionError + gyroXMeasurementNoise);
    gyroXEstimate = gyroXPrediction + gyroXKalmanGain * (gyroXFiltered - gyroXPrediction);
    gyroXEstimateError = (1.0 - gyroXKalmanGain) * gyroXPredictionError;

    // Kalman filter for accelerometer Y
    float accYPrediction = accYEstimate;
    float accYPredictionError = accYEstimateError + accYProcessNoise;

    float accYKalmanGain = accYPredictionError / (accYPredictionError + accYMeasurementNoise);
    accYEstimate = accYPrediction + accYKalmanGain * (accYFiltered - accYPrediction);
    accYEstimateError = (1.0 - accYKalmanGain) * accYPredictionError;

    // Kalman filter for accelerometer Z
    float accZPrediction = accZEstimate;
    float accZPredictionError = accZEstimateError + accZProcessNoise;

    float accZKalmanGain = accZPredictionError / (accZPredictionError + accZMeasurementNoise);
    accZEstimate = accZPrediction + accZKalmanGain * (accZFiltered - accZPrediction);
    accZEstimateError = (1.0 - accZKalmanGain) * accZPredictionError;
}

void MPU6050::applyHighPassFilter()
{
    gyroXFiltered = alpha * (gyroX - gyroXPrev) + (1 - alpha) * gyroXFiltered;
    gyroXPrev = gyroX;

    accYFiltered = alpha * (accY - accYPrev) + (1 - alpha) * accYFiltered;
    accYPrev = accY;

    accZFiltered = alpha * (accZ - accZPrev) + (1 - alpha) * accZFiltered;
    accZPrev = accZ;
}

void MPU6050::applyMovingAverageFilter()
{
    gyroX = movingAverage(gyroX, gyroXBuffer, bufferIndex);
    accY = movingAverage(accY, accYBuffer, bufferIndex);
    accZ = movingAverage(accZ, accZBuffer, bufferIndex);
}

float MPU6050::movingAverage(float newValue, float buffer[], int &index)
{
    buffer[index] = newValue;
    index = (index + 1) % windowSize;

    float sum = 0.0;
    for (int i = 0; i < windowSize; i++)
    {
        sum += buffer[i];
    }
    return sum / windowSize;
}