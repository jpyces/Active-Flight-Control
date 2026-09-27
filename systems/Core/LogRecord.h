#pragma once
#include <cstdint>
#include <type_traits>

namespace gnc
{
    // Plain-data mirrors of the math types, so the log layout is independent
    // of Vector3/Quaternion.
    struct LogVec3
    {
        float x, y, z;
    };

    struct LogQuaternion
    {
        float w, x, y, z;
    };

    struct LogEuler
    {
        float roll, pitch, yaw; // rad
    };

    struct LogHeader
    {
        uint32_t tick;        // FlightCore tick count
        uint32_t timestampUs; // stamped by main.cpp / SIM after tick()
        uint32_t loopExecUs;  // execution time of the previous tick
        float dt;             // s
        uint16_t schemaVersion;
        uint8_t reserved[2]; // explicit padding
    }; // 20 B

    struct LogSensors
    {
        LogVec3 accel;      // g, body frame
        LogVec3 gyro;       // rad/s, body frame
        LogVec3 mag;        // uT, body frame
        float baroAltitude; // m

        // SensorStatus values
        uint8_t accelStatus;
        uint8_t gyroStatus;
        uint8_t magStatus;
        uint8_t baroStatus;

        // 1 = new sample this tick, 0 = held value
        uint8_t accelFresh;
        uint8_t gyroFresh;
        uint8_t magFresh;
        uint8_t baroFresh;

        // GNSS: add in a later schema version as plain values + status/fresh bytes
    }; // 48 B

    struct LogEstimate
    {
        LogQuaternion quaternion;
        LogEuler eulerAngles;
        float altitude;         // m
        float verticalVelocity; // m/s
        float accelBias;        // vertical accel bias from the KF

        uint8_t ahrsSource;    // AhrsSource value
        uint8_t yawObservable; // 1 = yaw observable
        uint8_t reserved[2];   // explicit padding
    }; // 44 B

    struct LogMode
    {
        // Setpoints, flattened -- match these to the fields in Setpoints.h
        LogEuler spAttitude;
        float spAltitude; // m

        uint8_t flightState; // FlightState value
        uint8_t failsafe;    // Failsafe value
        uint8_t landed;
        uint8_t armed;
        uint8_t flightTrigger;
        uint8_t rcValid;
        uint8_t killed;      // 1 = kill latched (FsmOutputs::killed)
        uint8_t reserved[1]; // explicit padding
    }; // 24 B

    struct LogControl
    {
        float throttleN;   // collective thrust, N
        LogVec3 torque;    // pre-mix torque demand, N*m
        float motorCmd[4]; // normalized motor commands
    }; // 32 B

    struct LogRecord
    {
        static constexpr uint16_t kSchemaVersion = 1;
        LogHeader header;
        LogSensors sensors;
        LogEstimate estimate;
        LogMode mode;
        LogControl control;
    };

    static_assert(std::is_trivially_copyable_v<LogRecord>, "LogRecord must be memcpy-able");
    static_assert(sizeof(LogRecord) == 168, "LogRecord layout changed: bump kSchemaVersion and update the decoder");
}