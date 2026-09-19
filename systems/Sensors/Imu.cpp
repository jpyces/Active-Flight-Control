#include <SPI.h>
#include <cmath>

#include "Imu.h"
#include "config.h"

using namespace gnc;

Imu::Imu()
    : imu(), status(SensorStatus::UNINITIALIZED), consecutiveFailures(0), consecutiveSuccesses(0)
{
};

bool Imu::begin() {
    bool ok = imu.begin(kLsm6dsoImuAddress, IMU_WIRE);
    status = ok ? SensorStatus::NOMINAL : SensorStatus::FAILED;
    return ok;
};

SensorStatus Imu::checkHealth() {
    IMU_WIRE.beginTransmission(kLsm6dsoImuAddress);
    bool ack = (IMU_WIRE.endTransmission() == 0);

    if(ack) {
        consecutiveFailures = 0;
        consecutiveSuccesses++;
        if (consecutiveSuccesses >= FULL_RECOVERY_THRESHOLD)
            status = SensorStatus::NOMINAL;
        else if (consecutiveSuccesses >= RECOVERY_THRESHOLD)
            status = SensorStatus::DEGRADED;
    } else {
        consecutiveSuccesses = 0;
        consecutiveFailures++;
        if (status != SensorStatus::FAILED && consecutiveFailures >= DEGRADE_THRESHOLD)
            status = SensorStatus::DEGRADED;
        if (consecutiveFailures >= FAILURE_THRESHOLD)
            status = SensorStatus::FAILED;
    }

    return status;

};

SensorStatus Imu::getStatus() const {
    return status;
}

std::int16_t Imu::getRawAccelX() {
    return imu.readRawAccelX();
}

std::int16_t Imu::getRawAccelY() {
    return imu.readRawAccelY();
}

std::int16_t Imu::getRawAccelZ() {
    return imu.readRawAccelZ();
}

std::int16_t Imu::getRawGyroX() {
    return imu.readRawGyroX();
}

std::int16_t Imu::getRawGyroY() {
    return imu.readRawGyroY();
}

std::int16_t Imu::getRawGyroZ() {
    return imu.readRawGyroZ();
}

std::int16_t Imu::getRawTemperature() {
    return imu.readRawTemp();
}

Imu::RawSample Imu::getRawSample() {
    return {
        getRawAccelX(),
        getRawAccelY(),
        getRawAccelZ(),
        getRawGyroX(),
        getRawGyroY(),
        getRawGyroZ(),
        getRawTemperature()
    };
}

float Imu::getAccelXG() {
    return imu.readFloatAccelX();
}

float Imu::getAccelYG() {
    return imu.readFloatAccelY();
}

float Imu::getAccelZG() {
    return imu.readFloatAccelZ();
}

Vector3 Imu::getAccelG() {
    return {
        getAccelXG(),
        getAccelYG(),
        getAccelZG()
    };
}

float Imu::getAccelMagnitudeG() {
    Vector3 accel = getAccelG();
    return std::sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));
}

float Imu::getGyroXDps() {
    return imu.readFloatGyroX();
}

float Imu::getGyroYDps() {
    return imu.readFloatGyroY();
}

float Imu::getGyroZDps() {
    return imu.readFloatGyroZ();
}

Vector3 Imu::getGyroDps() {
    return {
        getGyroXDps(),
        getGyroYDps(),
        getGyroZDps()
    };
}

float Imu::getGyroMagnitudeDps() {
    Vector3 gyro = getGyroDps();
    return std::sqrt((gyro.x * gyro.x) + (gyro.y * gyro.y) + (gyro.z * gyro.z));
}

float Imu::getTemperatureC() {
    return imu.readTempC();
}

float Imu::getTemperatureF() {
    return imu.readTempF();
}

Imu::MotionSample Imu::getMotionSample() {
    Vector3 accel = getAccelG();
    Vector3 gyro = getGyroDps();

    return {
        accel,
        gyro,
        getTemperatureC(),
        std::sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z)),
        std::sqrt((gyro.x * gyro.x) + (gyro.y * gyro.y) + (gyro.z * gyro.z))
    };
}

std::uint8_t Imu::getDataReadyFlags() {
    return imu.listenDataReady();
}

bool Imu::isAccelDataReady() {
    return (getDataReadyFlags() & ACCEL_DATA_READY) != 0;
}

bool Imu::isGyroDataReady() {
    return (getDataReadyFlags() & GYRO_DATA_READY) != 0;
}

bool Imu::isTemperatureDataReady() {
    return (getDataReadyFlags() & TEMP_DATA_READY) != 0;
}

bool Imu::configureForFlight(std::uint8_t accelRangeG, std::uint16_t gyroRangeDps, std::uint16_t sampleRateHz) {
    bool ok = true;
    ok = imu.setAccelRange(accelRangeG) && ok;
    ok = imu.setGyroRange(gyroRangeDps) && ok;
    ok = imu.setAccelDataRate(sampleRateHz) && ok;
    ok = imu.setGyroDataRate(sampleRateHz) && ok;
    ok = imu.setBlockDataUpdate(true) && ok;
    ok = imu.setHighPerfAccel(true) && ok;
    ok = imu.setHighPerfGyro(true) && ok;

    if (!ok)
        status = SensorStatus::DEGRADED;

    return ok;
}

bool Imu::isAccelNearLimit(float marginG) {
    float limit = imu.getAccelRange();
    return getAccelMagnitudeG() >= (limit - marginG);
}

bool Imu::isGyroNearLimit(float marginDps) {
    float limit = imu.getGyroRange();
    return getGyroMagnitudeDps() >= (limit - marginDps);
}
