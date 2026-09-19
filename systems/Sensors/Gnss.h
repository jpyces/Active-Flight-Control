/*
Handles reading directly from the GPS with declarations to make stuff public
*/

#pragma once

#include <cstdint>

#include <SparkFun_u-blox_GNSS_v3.h>

#include "SensorStatus.h"

namespace gnc
{

    struct GnssData
    {
        // Position
        double latitude;   // degrees
        double longitude;  // degrees
        float altitudeMSL; // meters

        // Velocity, NED frame
        float velNorth; // m/s
        float velEast;  // m/s
        float velDown;  // m/s

        // Accuracy estimates
        float horizontalAcc; // meters
        float verticalAcc;   // meters
        float speedAcc;      // m/s

        // Fix quality / metadata
        uint8_t fixType;
        uint8_t satellites;
        float pdop;

        uint32_t timeOfWeekMs;
        bool valid;

        void print() const
        {
            if (valid)
            {
                Serial.println("--- GNSS Data ---");

                // Position (Access members directly)
                Serial.print("Lat: ");
                Serial.println(latitude, 7);
                Serial.print("Lon: ");
                Serial.println(longitude, 7);
                Serial.print("Alt (MSL): ");
                Serial.print(altitudeMSL, 2);
                Serial.println(" m");

                // Velocity (NED)
                Serial.print("Vel N: ");
                Serial.print(velNorth, 2);
                Serial.print(" | E: ");
                Serial.print(velEast, 2);
                Serial.print(" | D: ");
                Serial.print(velDown, 2);
                Serial.println(" m/s");

                // Accuracy
                Serial.print("Acc Horz: ");
                Serial.print(horizontalAcc, 2);
                Serial.print(" m | Vert: ");
                Serial.print(verticalAcc, 2);
                Serial.print(" m | Speed: ");
                Serial.print(speedAcc, 2);
                Serial.println(" m/s");

                // Fix Quality
                Serial.print("Fix: ");
                Serial.print(fixType);
                Serial.print(" | Sats: ");
                Serial.print(satellites);
                Serial.print(" | PDOP: ");
                Serial.println(pdop, 2);

                // Timing
                Serial.print("TimeOfWeek: ");
                Serial.println(timeOfWeekMs);

                Serial.println("-----------------");
            }
            else
            {
                Serial.println("GNSS Data: INVALID");
            }
        }
    };

    class GNSS
    {
    private:
        // Sensor
        SFE_UBLOX_GNSS_SERIAL gnss;
        bool gnc_DEBUG;

        // Meta
        SensorStatus status;
        std::uint8_t consecutiveSuccesses; // consecutive fresh PVT messages with a good (>=3D) fix
        std::uint8_t consecutiveFailures;  // consecutive fresh PVT messages WITHOUT a good fix
        unsigned long lastPvtMillis;       // millis() timestamp of the last observed NAV-PVT iTOW update
        uint32_t lastTimeOfWeekMs;
        bool hasTimeOfWeek;

        // Cold-start fix acquisition (can legitimately take 20-30s+ depending on sky visibility)
        // is not a health problem — NAV-PVT messages flow at the full configured rate the whole
        // time, just with fixType<3, so without this the "bad-fix" DEGRADE_THRESHOLD below would
        // trip ~0.3s after begin() and sit in DEGRADED for the entire acquisition window on every
        // single boot. hasEverFixed + firstFixDeadlineMillis give the module a generous, one-time
        // grace period to get its FIRST fix before "no good fix yet" starts counting against it.
        // Losing a fix it already had is a different, real signal and is never grace-windowed.
        bool hasEverFixed;
        bool hasValidFix;
        unsigned long firstFixDeadlineMillis;
        static constexpr unsigned long FIRST_FIX_GRACE_MS = 45000; // generous vs. typical cold-start lock times

        // SparkFun's PVT getters clear the library's internal "fresh" flags as they are read, so
        // getPVT()'s own return value is not a stable freshness signal once other getters (e.g.
        // inside getData()) have also touched the same message's bookkeeping. iTOW (a field in the
        // payload itself, immune to how many times other getters were called) is the real source of
        // truth for "a new NAV-PVT solution arrived" — see getData()'s use of lastTimeOfWeekMs above.
        static constexpr std::uint8_t DEGRADE_THRESHOLD = 3;       // 3 consecutive bad-fix PVT messages (~0.3s at the module's real 10Hz)
        static constexpr std::uint8_t RECOVERY_THRESHOLD = 3;      // 3 consecutive good fixes -> degraded (partial recovery)
        static constexpr std::uint8_t FULL_RECOVERY_THRESHOLD = 5; // 5 consecutive good fixes -> nominal
        static constexpr unsigned long SILENCE_DEGRADE_MS = 300;   // no NAV-PVT message at all for 0.3s -> degraded
        static constexpr unsigned long SILENCE_FAILURE_MS = 2500;  // no NAV-PVT message at all for 2.5s -> failed

        void recordFreshPvt(bool fixOk, unsigned long now);
        void recordPvtSilence(unsigned long now);

    public:
        // Snapshot of what checkHealth() is actually seeing, for debugging a status verdict
        // that doesn't match what getData() prints (getData()'s fields are plain cached
        // getters — they return the module's last known values whether or not anything
        // fresh has arrived, so "the printed data looks fine" doesn't by itself mean
        // checkHealth() is wrong; this exposes the actual freshness/counters behind it).
        struct HealthDiagnostics
        {
            bool hasEverFixed;
            unsigned long msSinceLastPvt;
            std::uint8_t consecutiveSuccesses;
            std::uint8_t consecutiveFailures;
        };

        // constructor declaration
        GNSS();

        // Method declarations
        bool begin();

        // Thin wrapper around getData() — the only method that actually touches the receiver and
        // updates health state (see getData()'s doc comment). Calling checkHealth() AND getData()
        // independently in the same tick used to each partially consume the SparkFun library's
        // internal per-message "queried" flags, causing one to see stale/missing data the other
        // had already read — that's what this delegation avoids: there is now exactly one
        // hardware-touching path, matching Magnetometer::checkHealth()'s delegation through
        // getMagSample() for the same reason.
        SensorStatus checkHealth();
        SensorStatus getStatus() const;
        bool hasFix() const; // true iff the most recent read had fixType>=3 and getGnssFixOk() —
                             // data VALIDITY, independent of checkHealth()'s health/liveness verdict.
                             // A cold-start acquisition window can be status()==NOMINAL with hasFix()==false
                             // (the receiver is behaving correctly; there's just no usable fix yet) —
                             // consumers that need position data must check hasFix(), not getStatus().
        HealthDiagnostics getHealthDiagnostics() const;

        // getters
        //
        // The sole hardware-touching read path for this class. Determines freshness via iTOW
        // (timeOfWeekMs) advancing rather than getPVT()'s edge-triggered return value, and updates
        // health state (recordFreshPvt/recordPvtSilence) as a side effect of that determination —
        // so any code that reads GNSS data also, correctly, keeps the health state current.
        GnssData getData(GnssData d);
    };

}