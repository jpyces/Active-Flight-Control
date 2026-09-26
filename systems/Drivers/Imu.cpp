#ifdef GNC_HARDWARE_BUILD

#include <SPI.h>
#include <cmath>

#include "Imu.h"
#include "config.h"

using namespace gnc;

namespace
{
    constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

    // Counts are reads. Staleness: at the 416 Hz ODR set by configureForFlight() a
    // new sample exists every ~2.4 ms, so 50 ms without one is a real fault.
    constexpr SensorHealthConfig kHealthConfig{
        2,      // degradeAfterFailures
        4,      // failAfterFailures
        9,      // recoverAfterSuccesses
        15,     // fullRecoverAfterSuccesses
        50000,  // staleDegradeUs
        200000, // staleFailUs
    };

    constexpr std::uint8_t kAccelGyroReady = ACCEL_DATA_READY | GYRO_DATA_READY;
}

Imu::Imu()
    : imu(), m_health(kHealthConfig)
{
}

bool Imu::begin()
{
    bool ok = imu.begin(kLsm6dsoImuAddress, IMU_WIRE);
    m_health.begin(ok, micros());
    return ok;
}

SensorStatus Imu::checkHealth()
{
    getMotionSample();
    return m_health.status();
}

SensorStatus Imu::getStatus() const
{
    return m_health.status();
}

bool Imu::isFresh() const
{
    return m_health.isFresh();
}

std::int16_t Imu::getRawAccelX()
{
    return imu.readRawAccelX();
}

std::int16_t Imu::getRawAccelY()
{
    return imu.readRawAccelY();
}

std::int16_t Imu::getRawAccelZ()
{
    return imu.readRawAccelZ();
}

std::int16_t Imu::getRawGyroX()
{
    return imu.readRawGyroX();
}

std::int16_t Imu::getRawGyroY()
{
    return imu.readRawGyroY();
}

std::int16_t Imu::getRawGyroZ()
{
    return imu.readRawGyroZ();
}

std::int16_t Imu::getRawTemperature()
{
    return imu.readRawTemp();
}

Imu::RawSample Imu::getRawSample()
{
    return {
        getRawAccelX(),
        getRawAccelY(),
        getRawAccelZ(),
        getRawGyroX(),
        getRawGyroY(),
        getRawGyroZ(),
        getRawTemperature()};
}

float Imu::getAccelXG()
{
    return imu.readFloatAccelX();
}

float Imu::getAccelYG()
{
    return imu.readFloatAccelY();
}

float Imu::getAccelZG()
{
    return imu.readFloatAccelZ();
}

// NOTE: reads X/Y/Z as three separate I2C transactions. This is only
// torn-sample-safe because configureForFlight() enables Block Data
// Update, which holds all axis registers frozen until every byte has
// been read. Don't remove that setBlockDataUpdate(true) call without
// revisiting this.
Vector3 Imu::getAccelG()
{
    return {
        getAccelXG(),
        getAccelYG(),
        getAccelZG()};
}

float Imu::getAccelMagnitudeG()
{
    Vector3 accel = getAccelG();
    return std::sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));
}

float Imu::getGyroXDps()
{
    return imu.readFloatGyroX();
}

float Imu::getGyroYDps()
{
    return imu.readFloatGyroY();
}

float Imu::getGyroZDps()
{
    return imu.readFloatGyroZ();
}

// Same BDU-dependent atomicity note as getAccelG() applies here.
Vector3 Imu::getGyroDps()
{
    return {
        getGyroXDps(),
        getGyroYDps(),
        getGyroZDps()};
}

float Imu::getGyroMagnitudeDps()
{
    Vector3 gyro = getGyroDps();
    return std::sqrt((gyro.x * gyro.x) + (gyro.y * gyro.y) + (gyro.z * gyro.z));
}

float Imu::getGyroXRadPerSec()
{
    return getGyroXDps() * kDegToRad;
}

float Imu::getGyroYRadPerSec()
{
    return getGyroYDps() * kDegToRad;
}

float Imu::getGyroZRadPerSec()
{
    return getGyroZDps() * kDegToRad;
}

Vector3 Imu::getGyroRadPerSec()
{
    return {
        getGyroXRadPerSec(),
        getGyroYRadPerSec(),
        getGyroZRadPerSec()};
}

float Imu::getGyroMagnitudeRadPerSec()
{
    return getGyroMagnitudeDps() * kDegToRad;
}

float Imu::getTemperatureC()
{
    return imu.readTempC();
}

float Imu::getTemperatureF()
{
    return imu.readTempF();
}

// Magnitudes recomputed by hand here rather than via
// getAccelMagnitudeG()/getGyroMagnitudeDps() deliberately -- calling
// those would re-trigger getAccelG()/getGyroDps() and cost a second,
// redundant round of I2C reads. Reusing the local accel/gyro already
// fetched above avoids that.
Imu::MotionSample Imu::getMotionSample()
{
    // readRegister() rather than listenDataReady(): the latter returns 0xFF on a
    // bus error, which reads as "all data ready".
    std::uint8_t flags = 0;
    const bool readOk = imu.readRegister(&flags, STATUS_REG) == IMU_SUCCESS;
    const bool newData = readOk && (flags & kAccelGyroReady) == kAccelGyroReady;
    m_health.record(readOk, newData, micros());

    if (!readOk)
    {
        const Vector3 nanVec{NAN, NAN, NAN};
        return {nanVec, nanVec, NAN, NAN, NAN};
    }

    Vector3 accel = getAccelG();
    Vector3 gyro = getGyroDps();

    return {
        accel,
        gyro,
        getTemperatureC(),
        std::sqrt((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z)),
        std::sqrt((gyro.x * gyro.x) + (gyro.y * gyro.y) + (gyro.z * gyro.z))};
}

std::uint8_t Imu::getDataReadyFlags()
{
    return imu.listenDataReady();
}

bool Imu::isAccelDataReady()
{
    return (getDataReadyFlags() & ACCEL_DATA_READY) != 0;
}

bool Imu::isGyroDataReady()
{
    return (getDataReadyFlags() & GYRO_DATA_READY) != 0;
}

bool Imu::isTemperatureDataReady()
{
    return (getDataReadyFlags() & TEMP_DATA_READY) != 0;
}

bool Imu::configureForFlight(std::uint8_t accelRangeG, std::uint16_t gyroRangeDps, std::uint16_t sampleRateHz)
{
    bool ok = true;
    ok = imu.setAccelRange(accelRangeG) && ok;
    ok = imu.setGyroRange(gyroRangeDps) && ok;
    ok = imu.setAccelDataRate(sampleRateHz) && ok;
    ok = imu.setGyroDataRate(sampleRateHz) && ok;
    ok = imu.setBlockDataUpdate(true) && ok;
    ok = imu.setHighPerfAccel(true) && ok;
    ok = imu.setHighPerfGyro(true) && ok;

    if (!ok)
        m_health.degrade();

    return ok;
}

bool Imu::isAccelNearLimit(float marginG)
{
    float limit = imu.getAccelRange();
    return getAccelMagnitudeG() >= (limit - marginG);
}

bool Imu::isGyroNearLimit(float marginDps)
{
    float limit = imu.getGyroRange();
    return getGyroMagnitudeDps() >= (limit - marginDps);
}

#endif