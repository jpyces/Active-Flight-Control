/*
Handles magnetometer interaction stuff directly with declarations to make stuff public
*/

#pragma once

#ifdef GNC_HARDWARE_BUILD

#include <cstdint>

#include <Wire.h>
#include "Vector3.h"
#include "Adafruit_MLX90393.h"

#include "SensorHealth.h"
#include "SensorInterface.h"
#include "SensorStatus.h"


#define MLX90393_MAG_ADDRESS MLX90393_DEFAULT_ADDR // 0x0C; can differ with A0/A1 strapping

namespace gnc
{

    class Magnetometer : public SensorInterface
    {
    public:

        // Coherent converted sample for flight-control loops and telemetry packets.
        struct MagSample
        {
            Vector3 fieldUT;
            float fieldMagnitudeUT;
            bool valid;
        };

    private:
        // Sensor
        Adafruit_MLX90393 mag;

        // Health + freshness, updated only by getMagSample()
        SensorHealth m_health;

        // Non-blocking conversion cycle driven by getMagSample(): start a single
        // measurement on one tick, collect it on the first tick after the conversion
        // time has passed, and start the next one immediately.
        enum class ConversionState : std::uint8_t
        {
            Idle,      // no conversion in flight (after begin/config change/failed start)
            Converting // started at m_convStartUs, result due after m_convTimeUs
        };
        ConversionState m_convState;
        std::uint32_t m_convStartUs;
        std::uint32_t m_convTimeUs; // from the datasheet tconv for the current filter/OSR, plus margin
        MagSample m_lastSample;     // latest collected sample, held between conversions

        bool startConversion(std::uint32_t nowUs);
        void restartConversions(); // discard any in-flight conversion (settings changed)
        void updateConversionTime();

        // MLX90393 output is a 16-bit 2's-complement value (span ±2^15 counts); multiplying
        // that span by the datasheet SENS_XY/SENS_Z (µT/LSB) figure for the active
        // gain/resolution gives the axis full-scale used by isFieldNearLimit().
        static constexpr std::int32_t FULL_SCALE_COUNTS = 32768;

        // Datasheet Table 3 sensitivities (µT/LSB), indexed [gain 0-7][resolution 0-3].
        // Gain index matches mlx90393_gain_t's declared order (GAIN_5X=0 ... GAIN_1X=7);
        // resolution index matches mlx90393_resolution_t (RES_16=0 ... RES_19=3).
        static constexpr float SENS_XY_UT_PER_LSB[8][4] = {
            {0.805F, 1.610F, 3.220F, 6.440F},
            {0.644F, 1.288F, 2.576F, 5.152F},
            {0.483F, 0.966F, 1.932F, 3.864F},
            {0.403F, 0.805F, 1.610F, 3.220F},
            {0.322F, 0.644F, 1.288F, 2.576F},
            {0.268F, 0.537F, 1.073F, 2.147F},
            {0.215F, 0.429F, 0.859F, 1.717F},
            {0.161F, 0.322F, 0.644F, 1.288F},
        };
        static constexpr float SENS_Z_UT_PER_LSB[8][4] = {
            {1.468F, 2.936F, 5.872F, 11.744F},
            {1.174F, 2.349F, 4.698F, 9.395F},
            {0.881F, 1.762F, 3.523F, 7.046F},
            {0.734F, 1.468F, 2.936F, 5.872F},
            {0.587F, 1.174F, 2.349F, 4.698F},
            {0.489F, 0.979F, 1.957F, 3.915F},
            {0.391F, 0.783F, 1.566F, 3.132F},
            {0.294F, 0.587F, 1.174F, 2.349F},
        };

        // Full-scale magnitude (µT) for one axis at the currently configured gain/resolution.
        float fullScaleUT(mlx90393_axis axis);

    public:
        // Creates the driver wrapper; call begin() before reading data.
        Magnetometer();

        // Starts the MLX90393 at the configured I2C address.
        // Defaults to Wire1 to match this project's wiring (MLX90393 magnetometer on Wire1,
        // per config.h / hardware docs) — pass &Wire or &Wire2 explicitly if that ever changes.
        bool begin(TwoWire *wireBus = &Wire1);

        // Delegates to getMagSample() and discards the sample. Call one or the other
        // once per tick, not both: each call advances the conversion cycle.
        SensorStatus checkHealth();

        // Returns the last health state calculated by begin() or getMagSample().
        SensorStatus getStatus() const override;

        // True if the last getMagSample() collected a newly completed conversion.
        bool isFresh() const override;

        // Resets the sensor and re-applies its default power-on configuration.
        bool reset();

        // Exits the sensor's current acquisition mode (burst/single/wake-on-change).
        bool exitMode();

        // The per-tick read, never blocks. Advances the conversion cycle by one step:
        //   - conversion still running: returns the held last sample (not fresh)
        //   - conversion done: reads it (fresh on success), then starts the next one
        //   - nothing in flight: starts a conversion, returns the held last sample
        // Call exactly once per tick. The held sample is NaN/invalid until the first
        // conversion completes, and after a failed read.
        MagSample getMagSample();

        // Accessors for the held sample (latest completed conversion). They do not
        // touch the sensor or advance the cycle.
        const MagSample &getLastSample() const;
        float getFieldXUT() const;
        float getFieldYUT() const;
        float getFieldZUT() const;
        Vector3 getFieldUT() const;
        float getFieldMagnitudeUT() const; // useful for hard-iron sanity checks

        // Configures broad-range, high-performance sampling defaults for rockets/drones.
        bool configureForFlight(mlx90393_gain gain = MLX90393_GAIN_1X,
                                mlx90393_resolution resolution = MLX90393_RES_16,
                                mlx90393_oversampling oversampling = MLX90393_OSR_0,
                                mlx90393_filter filter = MLX90393_FILTER_5);

        // Detects readings close to the configured full-scale range so saturation can be flagged.
        bool isFieldNearLimit(const MagSample &sample, float marginUT = 5.0F);
        bool isFieldNearLimit(float marginUT = 5.0F); // checks the held sample

        // getters
        mlx90393_gain getGain();
        mlx90393_resolution getResolution(mlx90393_axis axis);
        mlx90393_oversampling getOversampling();
        mlx90393_filter getFilter();

        // setters -- each discards any conversion in flight, so no sample taken
        // under the old settings is ever collected
        bool setGain(mlx90393_gain gain);
        bool setResolution(mlx90393_axis axis, mlx90393_resolution resolution);
        bool setOversampling(mlx90393_oversampling oversampling);
        bool setFilter(mlx90393_filter filter);
        bool setTrigInt(bool state);
    };

}

#endif