/*
Handles IMU interaction stuff directly with declarations to make stuff public
*/

#pragma once

#include <cstdint>

#include "SparkFunLSM6DSO.h"
#include "SensorStatus.h"
#include "Vector3.h"

namespace gnc
{

    inline constexpr std::uint8_t kLsm6dsoImuAddress = 0x6A; // can be 0x6A with a specific hardware setup

    class Imu
    {
    public:
        // One-shot raw register readout for logging, calibration, and sanity checks.
        struct RawSample
        {
            std::int16_t accelX;
            std::int16_t accelY;
            std::int16_t accelZ;
            std::int16_t gyroX;
            std::int16_t gyroY;
            std::int16_t gyroZ;
            std::int16_t temperature;
        };

        // Coherent converted sample for flight-control loops and telemetry packets.
        struct MotionSample
        {
            Vector3 accelG;
            Vector3 gyroDps;
            float temperatureC;
            float accelMagnitudeG;
            float gyroMagnitudeDps;
        };

    private:
        // Sensor (third-party type — stays unqualified, it's not yours to namespace)
        LSM6DSO imu;

        // Meta
        SensorStatus status;
        std::uint8_t consecutiveFailures;
        std::uint8_t consecutiveSuccesses;
        static constexpr std::uint8_t FAILURE_THRESHOLD = 4;
        static constexpr std::uint8_t RECOVERY_THRESHOLD = 9;
        static constexpr std::uint8_t FULL_RECOVERY_THRESHOLD = 15;
        static constexpr std::uint8_t DEGRADE_THRESHOLD = 2;

    public:
        Imu();
        bool begin();
        SensorStatus checkHealth();
        SensorStatus getStatus() const;

        std::int16_t getRawAccelX();
        std::int16_t getRawAccelY();
        std::int16_t getRawAccelZ();
        std::int16_t getRawGyroX();
        std::int16_t getRawGyroY();
        std::int16_t getRawGyroZ();
        std::int16_t getRawTemperature();
        RawSample getRawSample();

        float getAccelXG();
        float getAccelYG();
        float getAccelZG();
        Vector3 getAccelG();
        float getAccelMagnitudeG();

        float getGyroXDps();
        float getGyroYDps();
        float getGyroZDps();
        Vector3 getGyroDps();
        float getGyroMagnitudeDps();
        float getGyroXRadPerSec();
        float getGyroYRadPerSec();
        float getGyroZRadPerSec();
        Vector3 getGyroRadPerSec();
        float getGyroMagnitudeRadPerSec();

        float getTemperatureC();
        float getTemperatureF();

        MotionSample getMotionSample();

        std::uint8_t getDataReadyFlags();
        bool isAccelDataReady();
        bool isGyroDataReady();
        bool isTemperatureDataReady();

        bool configureForFlight(std::uint8_t accelRangeG = 16, std::uint16_t gyroRangeDps = 2000, std::uint16_t sampleRateHz = 416);

        bool isAccelNearLimit(float marginG = 0.5F);
        bool isGyroNearLimit(float marginDps = 50.0F);
    };

} // namespace gnc