#pragma once

#include <cstdint>
#include "SensorStatus.h"

namespace gnc
{
    // Debounce and staleness limits for one sensor. Counts are in record() calls
    // (one per read); times are in microseconds. A stale limit of 0 disables that
    // check.
    struct SensorHealthConfig
    {
        std::uint8_t degradeAfterFailures;      // NOMINAL -> DEGRADED
        std::uint8_t failAfterFailures;         // NOMINAL/DEGRADED -> FAILED
        std::uint8_t recoverAfterSuccesses;     // FAILED -> DEGRADED
        std::uint8_t fullRecoverAfterSuccesses; // DEGRADED -> NOMINAL
        std::uint32_t staleDegradeUs;           // no new sample for this long -> DEGRADED
        std::uint32_t staleFailUs;              // no new sample for this long -> FAILED
    };

    // Tracks two independent properties of a sensor:
    //   - health (status()): is the sensor responding and trustworthy? Debounced
    //     from read successes/failures, plus a staleness timeout so a sensor that
    //     still answers on the bus but has stopped producing new data is caught.
    //   - freshness (isFresh()): did the most recent read produce a new sample?
    //     A healthy sensor slower than the loop is NOMINAL but not fresh on most
    //     ticks; that is normal, not a fault.
    //
    // Hardware-independent: the caller passes the time in, so this is testable
    // under `native`. Each driver owns one and calls record() exactly once per
    // read, which makes "fresh" mean "new since the previous read".
    class SensorHealth
    {
    public:
        explicit SensorHealth(const SensorHealthConfig &cfg);

        // Result of the driver's begin(): NOMINAL on success, FAILED otherwise.
        // Clears the counters and starts the staleness clock at nowUs.
        void begin(bool ok, std::uint32_t nowUs);

        // One read attempt. readOk = the transaction succeeded; newData = it
        // returned a sample the sensor had not already delivered. Ignored until
        // begin() has run.
        void record(bool readOk, bool newData, std::uint32_t nowUs);

        // A tick on which the driver deliberately did not touch the sensor (e.g.
        // waiting out a conversion it started earlier). Clears freshness and runs
        // the staleness check, but counts as neither success nor failure, so it
        // cannot break a run of real read failures. Use instead of record() on
        // those ticks, still exactly once per tick.
        void idle(std::uint32_t nowUs);

        // Demotes NOMINAL to DEGRADED (e.g. a configuration write failed). Normal
        // debounced recovery applies afterwards.
        void degrade();

        SensorStatus status() const { return m_status; }
        bool isFresh() const { return m_fresh; }
        std::uint32_t lastSampleUs() const { return m_lastSampleUs; }

    private:
        void onSuccess();
        void onFailure();
        void checkStaleness(std::uint32_t nowUs);

        SensorHealthConfig m_cfg;
        SensorStatus m_status{SensorStatus::UNINITIALIZED};
        std::uint8_t m_successes{0};
        std::uint8_t m_failures{0};
        std::uint32_t m_lastSampleUs{0};
        bool m_fresh{false};
    };
}
