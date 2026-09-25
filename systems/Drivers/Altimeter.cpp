#ifdef GNC_HARDWARE_BUILD

#include <SPI.h>
#include "Altimeter.h"
#include "config.h"

using namespace gnc;

// Scanner detected at 0x7E, but 0x76 working???
static constexpr auto ALTIMETER_BMP280_ADDRESS = 0x76;

// Constructor
Altimeter::Altimeter()
    : bmp(&ALTI_WIRE), status(SensorStatus::UNINITIALIZED), consecutiveSuccesses(0), consecutiveFailures(0)
{
}

// Altimeter meta functioning
bool Altimeter::begin()
{
    bool ok = bmp.begin(ALTIMETER_BMP280_ADDRESS);
    status = ok ? SensorStatus::NOMINAL : SensorStatus::FAILED;
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
    ALTI_WIRE.beginTransmission(ALTIMETER_BMP280_ADDRESS);
    bool ack = (ALTI_WIRE.endTransmission() == 0);

    if (ack)
    {
        consecutiveFailures = 0;
        consecutiveSuccesses++;
        if (consecutiveSuccesses >= FULL_RECOVERY_THRESHOLD)
            status = SensorStatus::NOMINAL;
        else if (consecutiveSuccesses >= RECOVERY_THRESHOLD)
            status = SensorStatus::DEGRADED;
    }
    else
    {
        consecutiveSuccesses = 0;
        consecutiveFailures++;
        if (status != SensorStatus::FAILED && consecutiveFailures >= DEGRADE_THRESHOLD)
            status = SensorStatus::DEGRADED;
        if (consecutiveFailures >= FAILURE_THRESHOLD)
            status = SensorStatus::FAILED;
    }

    return status;
}

SensorStatus Altimeter::getStatus() const
{
    return status;
}


// altimeter data getters

float Altimeter::getTemperature()
{
    if (status == SensorStatus::FAILED)
        return NAN;
    return bmp.readTemperature();
}

float Altimeter::getPressure()
{
    if (status == SensorStatus::FAILED)
        return NAN;
    return bmp.readPressure();
}

float Altimeter::getAltitude(float seaLevelhPa)
{
    if (status == SensorStatus::FAILED)
        return NAN;
    return bmp.readAltitude(seaLevelhPa);
}

#endif