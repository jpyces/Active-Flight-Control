/*
Handles magnetometer interaction stuff directly.
*/

#include "magnetometer.h"

#include <cmath>

constexpr float Magnetometer::SENS_XY_UT_PER_LSB[8][4];
constexpr float Magnetometer::SENS_Z_UT_PER_LSB[8][4];

Magnetometer::Magnetometer()
    : mag(),
      status(SensorStatus::UNINITIALIZED),
      consecutiveSuccesses(0),
      consecutiveFailures(0) {}

bool Magnetometer::begin(TwoWire *wireBus)
{
    bool ok = mag.begin_I2C(MLX90393_MAG_ADDRESS, wireBus);

    if (ok)
    {
        status = SensorStatus::NOMINAL;
        consecutiveSuccesses = 0;
        consecutiveFailures = 0;
    }
    else
    {
        status = SensorStatus::FAILED;
        consecutiveFailures = FAILURE_THRESHOLD;
        consecutiveSuccesses = 0;
    }

    return ok;
}

SensorStatus Magnetometer::checkHealth()
{
    getMagSample();
    return status;
}

void Magnetometer::recordReadResult(bool ok)
{
    if (ok)
    {
        consecutiveSuccesses++;
        consecutiveFailures = 0;
    }
    else
    {
        consecutiveFailures++;
        consecutiveSuccesses = 0;
    }

    // Debounced state machine: a run of failures degrades and eventually fails the sensor;
    // a subsequent run of clean reads climbs back up rather than snapping straight to nominal.
    switch (status)
    {
    case SensorStatus::NOMINAL:
        if (consecutiveFailures >= FAILURE_THRESHOLD)
        {
            status = SensorStatus::FAILED;
        }
        else if (consecutiveFailures >= DEGRADE_THRESHOLD)
        {
            status = SensorStatus::DEGRADED;
        }
        break;

    case SensorStatus::DEGRADED:
        if (consecutiveFailures >= FAILURE_THRESHOLD)
        {
            status = SensorStatus::FAILED;
        }
        else if (consecutiveSuccesses >= FULL_RECOVERY_THRESHOLD)
        {
            status = SensorStatus::NOMINAL;
        }
        break;

    case SensorStatus::FAILED:
        if (consecutiveSuccesses >= RECOVERY_THRESHOLD)
        {
            status = SensorStatus::DEGRADED;
            consecutiveSuccesses = 0;
        }
        break;

    case SensorStatus::UNINITIALIZED:
    default:
        // checkHealth() before begin(); leave as-is until begin() runs.
        break;
    }

    return;
}

SensorStatus Magnetometer::getStatus() const
{
    return status;
}

bool Magnetometer::reset()
{
    return mag.reset();
}

bool Magnetometer::exitMode()
{
    return mag.exitMode();
}

bool Magnetometer::startSingleMeasurement()
{
    return mag.startSingleMeasurement();
}

float Magnetometer::getFieldXUT()
{
    return getFieldUT().x;
}

float Magnetometer::getFieldYUT()
{
    return getFieldUT().y;
}

float Magnetometer::getFieldZUT()
{
    return getFieldUT().z;
}

Magnetometer::Vector3 Magnetometer::getFieldUT()
{
    return getMagSample().fieldUT;
}

float Magnetometer::getFieldMagnitudeUT()
{
    return getMagSample().fieldMagnitudeUT;
}

Magnetometer::MagSample Magnetometer::getMagSample()
{
    MagSample sample{};
    sample.valid = mag.readData(&sample.fieldUT.x, &sample.fieldUT.y, &sample.fieldUT.z);
    recordReadResult(sample.valid);

    if (!sample.valid)
    {
        sample.fieldUT = {NAN, NAN, NAN};
        sample.fieldMagnitudeUT = NAN;
        return sample;
    }

    sample.fieldMagnitudeUT = sqrtf(sample.fieldUT.x * sample.fieldUT.x +
                                    sample.fieldUT.y * sample.fieldUT.y +
                                    sample.fieldUT.z * sample.fieldUT.z);
    return sample;
}

bool Magnetometer::configureForFlight(mlx90393_gain gain,
                                      mlx90393_resolution resolution,
                                      mlx90393_oversampling oversampling,
                                      mlx90393_filter filter)
{
    bool ok = true;

    ok &= mag.setGain(gain);
    ok &= mag.setResolution(MLX90393_X, resolution);
    ok &= mag.setResolution(MLX90393_Y, resolution);
    ok &= mag.setResolution(MLX90393_Z, resolution);
    ok &= mag.setOversampling(oversampling);
    ok &= mag.setFilter(filter);

    return ok;
}

float Magnetometer::fullScaleUT(mlx90393_axis axis)
{
    std::uint8_t gainIdx = static_cast<std::uint8_t>(mag.getGain());
    std::uint8_t resIdx = static_cast<std::uint8_t>(mag.getResolution(axis));

    float sensUtPerLsb = (axis == MLX90393_Z)
                             ? SENS_Z_UT_PER_LSB[gainIdx][resIdx]
                             : SENS_XY_UT_PER_LSB[gainIdx][resIdx];

    return sensUtPerLsb * static_cast<float>(FULL_SCALE_COUNTS);
}

bool Magnetometer::isFieldNearLimit(float marginUT)
{
    return isFieldNearLimit(getMagSample(), marginUT);
}

bool Magnetometer::isFieldNearLimit(const MagSample &sample, float marginUT)
{
    if (!sample.valid)
        return false;

    float limitX = fullScaleUT(MLX90393_X) - marginUT;
    float limitY = fullScaleUT(MLX90393_Y) - marginUT;
    float limitZ = fullScaleUT(MLX90393_Z) - marginUT;

    return fabsf(sample.fieldUT.x) >= limitX || fabsf(sample.fieldUT.y) >= limitY || fabsf(sample.fieldUT.z) >= limitZ;
}

mlx90393_gain Magnetometer::getGain()
{
    return mag.getGain();
}

mlx90393_resolution Magnetometer::getResolution(mlx90393_axis axis)
{
    return mag.getResolution(axis);
}

mlx90393_oversampling Magnetometer::getOversampling()
{
    return mag.getOversampling();
}

mlx90393_filter Magnetometer::getFilter()
{
    return mag.getFilter();
}

bool Magnetometer::setGain(mlx90393_gain gain)
{
    return mag.setGain(gain);
}

bool Magnetometer::setResolution(mlx90393_axis axis, mlx90393_resolution resolution)
{
    return mag.setResolution(axis, resolution);
}

bool Magnetometer::setOversampling(mlx90393_oversampling oversampling)
{
    return mag.setOversampling(oversampling);
}

bool Magnetometer::setFilter(mlx90393_filter filter)
{
    return mag.setFilter(filter);
}

bool Magnetometer::setTrigInt(bool state)
{
    return mag.setTrigInt(state);
}
