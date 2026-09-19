/*
altimeter stuff
*/

#pragma once

#include <cstdint>

#include <Adafruit_BMP280.h>

#include "SensorStatus.h"

namespace gnc
{
    class Altimeter
    {
    private:
        // Sensor
        Adafruit_BMP280 bmp;

        // Meta
        SensorStatus status;
        std::uint8_t consecutiveSuccesses;                          // helper to count how many status checks succeeded
        std::uint8_t consecutiveFailures;                           // helper to count how many status checks failed
        static constexpr std::uint8_t FAILURE_THRESHOLD = 8;        // 40ms at 100Hz
        static constexpr std::uint8_t RECOVERY_THRESHOLD = 6;       // 60ms at 100Hz
        static constexpr std::uint8_t DEGRADE_THRESHOLD = 4;        // 40ms at 100Hz
        static constexpr std::uint8_t FULL_RECOVERY_THRESHOLD = 16; // 160ms at 100Hz

    public:
        // constructor declaration
        Altimeter();

        // Method declarations
        bool begin();
        SensorStatus checkHealth();
        SensorStatus getStatus() const;

        // getters
        float getTemperature();
        float getPressure();
        float getAltitude(float seaLevelhPa = (1013.25F));
    };

}