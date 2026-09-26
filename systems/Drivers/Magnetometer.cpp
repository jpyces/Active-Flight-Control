/*
Handles magnetometer interaction stuff directly.
*/
#ifdef GNC_HARDWARE_BUILD

#include <cmath>
#include "Magnetometer.h"
#include "config.h"

using namespace gnc;

constexpr float Magnetometer::SENS_XY_UT_PER_LSB[8][4];
constexpr float Magnetometer::SENS_Z_UT_PER_LSB[8][4];

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

    // Wait tconv * factor + extra before collecting a conversion. Waiting longer
    // costs nothing now that nothing blocks. The library's blocking readData()
    // adds a flat 10 ms ("doesn't always work" without it, cause undocumented);
    // if bench reads fail at this margin, raise it before suspecting anything else.
    constexpr float kConvMarginFactor = 1.25F;
    constexpr std::uint32_t kConvMarginUs = 1000;

    constexpr Magnetometer::MagSample kInvalidSample{{NAN, NAN, NAN}, NAN, false};
}

Magnetometer::Magnetometer()
    : mag(), m_health(kHealthConfig), m_convState(ConversionState::Idle),
      m_convStartUs(0), m_convTimeUs(0), m_lastSample(kInvalidSample) {}

bool Magnetometer::begin(TwoWire *wireBus)
{
    bool ok = mag.begin_I2C(MLX90393_MAG_ADDRESS, wireBus);
    m_health.begin(ok, micros());
    updateConversionTime();
    restartConversions();
    return ok;
}

bool Magnetometer::startConversion(std::uint32_t nowUs)
{
    const bool ok = mag.startSingleMeasurement();
    m_convState = ok ? ConversionState::Converting : ConversionState::Idle;
    m_convStartUs = nowUs;
    return ok;
}

void Magnetometer::restartConversions()
{
    m_convState = ConversionState::Idle;
    m_lastSample = kInvalidSample;
}

void Magnetometer::updateConversionTime()
{
    // mlx90393_tconv (ms) is the library's copy of the datasheet table, indexed
    // [digital filter 0-7][oversampling 0-3]; the getters return cached settings.
    const auto filt = static_cast<std::uint8_t>(mag.getFilter());
    const auto osr = static_cast<std::uint8_t>(mag.getOversampling());
    const float tconvMs = mlx90393_tconv[filt & 0x07][osr & 0x03];
    m_convTimeUs = static_cast<std::uint32_t>(tconvMs * 1000.0F * kConvMarginFactor) + kConvMarginUs;
}

SensorStatus Magnetometer::checkHealth()
{
    getMagSample();
    return m_health.status();
}

SensorStatus Magnetometer::getStatus() const
{
    return m_health.status();
}

bool Magnetometer::isFresh() const
{
    return m_health.isFresh();
}

bool Magnetometer::reset()
{
    restartConversions();
    return mag.reset();
}

bool Magnetometer::exitMode()
{
    restartConversions();
    return mag.exitMode();
}

const Magnetometer::MagSample &Magnetometer::getLastSample() const
{
    return m_lastSample;
}

float Magnetometer::getFieldXUT() const
{
    return m_lastSample.fieldUT.x;
}

float Magnetometer::getFieldYUT() const
{
    return m_lastSample.fieldUT.y;
}

float Magnetometer::getFieldZUT() const
{
    return m_lastSample.fieldUT.z;
}

Vector3 Magnetometer::getFieldUT() const
{
    return m_lastSample.fieldUT;
}

float Magnetometer::getFieldMagnitudeUT() const
{
    return m_lastSample.fieldMagnitudeUT;
}

Magnetometer::MagSample Magnetometer::getMagSample()
{
    const std::uint32_t now = micros();

    if (m_convState == ConversionState::Idle)
    {
        // Nothing in flight: the start command is this tick's bus transaction.
        const bool ok = startConversion(now);
        m_health.record(ok, false, now);
        return m_lastSample;
    }

    if ((now - m_convStartUs) < m_convTimeUs)
    {
        // Conversion still running; the sensor is deliberately not touched.
        m_health.idle(now);
        return m_lastSample;
    }

    // Conversion done: collect it.
    MagSample sample{};
    sample.valid = mag.readMeasurement(&sample.fieldUT.x, &sample.fieldUT.y, &sample.fieldUT.z);
    m_health.record(sample.valid, sample.valid, now);

    if (sample.valid)
    {
        sample.fieldMagnitudeUT = sqrtf(sample.fieldUT.x * sample.fieldUT.x +
                                        sample.fieldUT.y * sample.fieldUT.y +
                                        sample.fieldUT.z * sample.fieldUT.z);
        m_lastSample = sample;
    }
    else
    {
        m_lastSample = kInvalidSample; // never keep serving a value the sensor failed to confirm
    }

    // Start the next conversion right away so it runs while the loop does other
    // work. If the start fails, the state drops to Idle and the next tick retries
    // it (and records that attempt), keeping one health record per tick.
    startConversion(now);
    return m_lastSample;
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

    updateConversionTime();
    restartConversions();
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
    return isFieldNearLimit(m_lastSample, marginUT);
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
    restartConversions();
    return mag.setGain(gain);
}

bool Magnetometer::setResolution(mlx90393_axis axis, mlx90393_resolution resolution)
{
    restartConversions();
    return mag.setResolution(axis, resolution);
}

bool Magnetometer::setOversampling(mlx90393_oversampling oversampling)
{
    restartConversions();
    const bool ok = mag.setOversampling(oversampling);
    updateConversionTime();
    return ok;
}

bool Magnetometer::setFilter(mlx90393_filter filter)
{
    restartConversions();
    const bool ok = mag.setFilter(filter);
    updateConversionTime();
    return ok;
}

bool Magnetometer::setTrigInt(bool state)
{
    restartConversions();
    return mag.setTrigInt(state);
}

#endif