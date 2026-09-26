/*
Handles IMU interaction stuff directly with declarations to make stuff public
*/

#pragma once

#ifdef GNC_HARDWARE_BUILD

#include <cstdint>

#include "SparkFunLSM6DSO.h"
#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"
#include "Vector3.h"

namespace gnc
{

    inline constexpr std::uint8_t kLsm6dsoImuAddress = 0x6A; // can be 0x6A with a specific hardware setup

    class Imu : public SensorInterface
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

        // Health + freshness, updated only by getMotionSample()
        SensorHealth m_health;

    public:
        Imu();
        bool begin();

        // Delegates to getMotionSample() (the one read path that updates health and
        // freshness) and discards the sample. Call one or the other once per tick,
        // not both: a second read in the same tick sees no new data.
        SensorStatus checkHealth();
        SensorStatus getStatus() const override;

        // True if the last getMotionSample() returned a new accel+gyro sample
        // (both STATUS_REG data-ready bits set).
        bool isFresh() const override;

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

        // The per-tick read: checks STATUS_REG for new data, records health and
        // freshness, then reads accel/gyro/temperature. Returns NaN fields if the
        // status read fails. The individual getters above are raw passthroughs for
        // debugging and do not update health or freshness. Call this before any of
        // them in a tick: reading the output registers clears the data-ready bits.
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

#endif