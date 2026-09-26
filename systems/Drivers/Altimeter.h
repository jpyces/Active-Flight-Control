/*
altimeter stuff
*/

#pragma once

#ifdef GNC_HARDWARE_BUILD

#include <cstdint>

#include <Adafruit_BMP280.h>

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"

namespace gnc
{
    class Altimeter : public SensorInterface
    {
    public:
        // Coherent sample for the flight loop: temperature and pressure are read once
        // each and altitude is derived from that same pressure reading.
        struct BaroSample
        {
            float temperatureC;
            float pressurePa;
            float altitudeM;
            bool valid;
        };

    private:
        // Sensor
        Adafruit_BMP280 bmp;

        // Health + freshness, updated only by getBaroSample()
        SensorHealth m_health;
        std::uint32_t m_lastNewSampleUs;

    public:
        // constructor declaration
        Altimeter();

        // Method declarations
        bool begin();

        // Delegates to getBaroSample() (the one read path that updates health and
        // freshness) and discards the sample. Call one or the other once per tick,
        // not both.
        SensorStatus checkHealth();
        SensorStatus getStatus() const override;

        // True if the last getBaroSample() is guaranteed to hold a conversion that
        // no earlier fresh sample contained. See getBaroSample() for how.
        bool isFresh() const override;

        // The per-tick read: bus ACK check, then one temperature and one pressure
        // read. valid is false (and fields NaN) if the sensor does not ACK.
        BaroSample getBaroSample(float seaLevelhPa = 1013.25F);

        // Raw passthrough getters for debugging; they do not update health or
        // freshness.
        float getTemperature();
        float getPressure();
        float getAltitude(float seaLevelhPa = (1013.25F));
    };

}
#endif
