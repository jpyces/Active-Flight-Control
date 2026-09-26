#ifdef GNC_HARDWARE_BUILD

#include <SPI.h>
#include <cmath>
#include "Altimeter.h"
#include "config.h"

using namespace gnc;

// Scanner detected at 0x7E, but 0x76 working???
static constexpr auto ALTIMETER_BMP280_ADDRESS = 0x76;

namespace
{
    // Counts are reads.
    constexpr SensorHealthConfig kHealthConfig{
        4,      // degradeAfterFailures
        8,      // failAfterFailures
        6,      // recoverAfterSuccesses
        16,     // fullRecoverAfterSuccesses
        100000, // staleDegradeUs
        500000, // staleFailUs
    };

    // Normal mode with the begin() settings (x1 temperature, x1 pressure, shortest
    // standby) produces a conversion every t_measure + t_standby: ~6.4 ms max +
    // 0.5 ms per the datasheet. The BMP280 has no reliable "new data" flag at loop
    // rate, so a sample counts as new only if at least this long has passed since
    // the last new one, which guarantees a fresh conversion without ever counting
    // one twice. Revisit if oversampling or standby in begin() changes.
    constexpr std::uint32_t kConversionPeriodUs = 8000;
}

// Constructor
Altimeter::Altimeter()
    : bmp(&ALTI_WIRE), m_health(kHealthConfig), m_lastNewSampleUs(0)
{
}

// Altimeter meta functioning
bool Altimeter::begin()
{
    bool ok = bmp.begin(ALTIMETER_BMP280_ADDRESS);
    m_lastNewSampleUs = micros();
    m_health.begin(ok, m_lastNewSampleUs);
    // Parameters: mode, temp oversampling, pressure oversampling, filter, standby time
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,   // Temperature
                    Adafruit_BMP280::SAMPLING_X1,  // Pressure
                    Adafruit_BMP280::FILTER_OFF,    // <--- IIR Filter Disabled
                    Adafruit_BMP280::STANDBY_MS_1); // Standby time
    return ok;
}

SensorStatus Altimeter::checkHealth()
{
    getBaroSample();
    return m_health.status();
}

SensorStatus Altimeter::getStatus() const
{
    return m_health.status();
}

bool Altimeter::isFresh() const
{
    return m_health.isFresh();
}

Altimeter::BaroSample Altimeter::getBaroSample(float seaLevelhPa)
{
    const std::uint32_t now = micros();

    ALTI_WIRE.beginTransmission(ALTIMETER_BMP280_ADDRESS);
    const bool ack = (ALTI_WIRE.endTransmission() == 0);

    BaroSample sample{NAN, NAN, NAN, false};
    if (ack)
    {
        // readPressure() reads temperature internally for compensation, so read
        // pressure first and temperature once after, instead of the three
        // separate getters (which re-read pressure for altitude).
        sample.pressurePa = bmp.readPressure();
        sample.temperatureC = bmp.readTemperature();
        // Same barometric formula as Adafruit_BMP280::readAltitude().
        sample.altitudeM = 44330.0F * (1.0F - powf((sample.pressurePa / 100.0F) / seaLevelhPa, 0.1903F));
        sample.valid = std::isfinite(sample.pressurePa) && std::isfinite(sample.altitudeM);
    }

    const bool newData = sample.valid && (now - m_lastNewSampleUs) >= kConversionPeriodUs;
    if (newData)
        m_lastNewSampleUs = now;

    m_health.record(sample.valid, newData, now);
    return sample;
}


// altimeter data getters

float Altimeter::getTemperature()
{
    if (m_health.status() == SensorStatus::FAILED)
        return NAN;
    return bmp.readTemperature();
}

float Altimeter::getPressure()
{
    if (m_health.status() == SensorStatus::FAILED)
        return NAN;
    return bmp.readPressure();
}

float Altimeter::getAltitude(float seaLevelhPa)
{
    if (m_health.status() == SensorStatus::FAILED)
        return NAN;
    return bmp.readAltitude(seaLevelhPa);
}

#endif