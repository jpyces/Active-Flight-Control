#pragma once

namespace gnc
{

    enum class SensorStatus
    {
        UNINITIALIZED, // begin() never called / never succeeded
        NOMINAL,       // healthy, ACKing, data trustworthy
        DEGRADED,      // some failures but under threshold — still using data, flagged
        FAILED         // over threshold, data should not be trusted
    };

    // Helper function to print the status name
    inline void printStatus(SensorStatus c)
    {
        switch (c)
        {
        case SensorStatus::UNINITIALIZED:
            Serial.println("UNINITIALIZED");
            break;
        case SensorStatus::NOMINAL:
            Serial.println("NOMINAL");
            break;
        case SensorStatus::DEGRADED:
            Serial.println("DEGRADED");
            break;
        case SensorStatus::FAILED:
            Serial.println("FAILED");
            break;
        default:
            Serial.println("UNKNOWN");
            break;
        }
    }

}