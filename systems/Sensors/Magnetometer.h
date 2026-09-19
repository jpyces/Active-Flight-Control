/*
Handles magnetometer interaction stuff directly with declarations to make stuff public
*/

#pragma once

#include <cstdint>

#include <Wire.h>

#include "Adafruit_MLX90393.h"

#include "SensorStatus.h"


#define MLX90393_MAG_ADDRESS MLX90393_DEFAULT_ADDR // 0x0C; can differ with A0/A1 strapping

namespace gnc
{

    class Magnetometer
    {
    public:
        // Generic 3-axis container used for magnetic field vectors.
        struct Vector3
        {
            float x;
            float y;
            float z;
        };

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

        // Meta
        SensorStatus status;
        std::uint8_t consecutiveSuccesses;                          // helper to count how many status checks succeeded
        std::uint8_t consecutiveFailures;                           // helper to count how many status checks failed
        static constexpr std::uint8_t FAILURE_THRESHOLD = 8;        // 40ms at 100Hz
        static constexpr std::uint8_t RECOVERY_THRESHOLD = 6;       // 60ms at 100Hz
        static constexpr std::uint8_t DEGRADE_THRESHOLD = 4;        // 40ms at 100Hz
        static constexpr std::uint8_t FULL_RECOVERY_THRESHOLD = 16; // 160ms at 100Hz

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
        void recordReadResult(bool ok);

    public:
        // Creates the driver wrapper; call begin() before reading data.
        Magnetometer();

        // Starts the MLX90393 at the configured I2C address.
        // Defaults to Wire1 to match this project's wiring (MLX90393 magnetometer on Wire1,
        // per config.h / hardware docs) — pass &Wire or &Wire2 explicitly if that ever changes.
        bool begin(TwoWire *wireBus = &Wire1);

        // Updates the health state using a measurement-read check and debounce thresholds.
        SensorStatus checkHealth();

        // Returns the last health state calculated by begin() or checkHealth().
        SensorStatus getStatus() const;

        // Resets the sensor and re-applies its default power-on configuration.
        bool reset();

        // Exits the sensor's current acquisition mode (burst/single/wake-on-change).
        bool exitMode();

        // Kicks off a single on-sensor conversion; pair with readField() once ready.
        bool startSingleMeasurement();

        // Converted magnetic field in microtesla for each axis.
        float getFieldXUT();
        float getFieldYUT();
        float getFieldZUT();

        // Converted magnetic field vector in microtesla.
        Vector3 getFieldUT();

        // Vector magnitude of magnetic field in microtesla; useful for hard-iron sanity checks.
        float getFieldMagnitudeUT();

        // Reads converted field and vector magnitude together.
        MagSample getMagSample();

        // Configures broad-range, high-performance sampling defaults for rockets/drones.
        bool configureForFlight(mlx90393_gain gain = MLX90393_GAIN_1X,
                                mlx90393_resolution resolution = MLX90393_RES_16,
                                mlx90393_oversampling oversampling = MLX90393_OSR_0,
                                mlx90393_filter filter = MLX90393_FILTER_5);

        // Detects readings close to the configured full-scale range so saturation can be flagged.
        bool isFieldNearLimit(const MagSample &sample, float marginUT = 5.0F);
        bool isFieldNearLimit(float marginUT = 5.0F);

        // getters
        mlx90393_gain getGain();
        mlx90393_resolution getResolution(mlx90393_axis axis);
        mlx90393_oversampling getOversampling();
        mlx90393_filter getFilter();

        // setters
        bool setGain(mlx90393_gain gain);
        bool setResolution(mlx90393_axis axis, mlx90393_resolution resolution);
        bool setOversampling(mlx90393_oversampling oversampling);
        bool setFilter(mlx90393_filter filter);
        bool setTrigInt(bool state);
    };

}