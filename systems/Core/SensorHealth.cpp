#include "SensorHealth.h"

namespace gnc
{
    namespace
    {
        // Saturating: a sensor that stays NOMINAL for more than 255 reads must
        // not wrap its success count back to 0.
        void increment(std::uint8_t &count)
        {
            if (count < UINT8_MAX)
                ++count;
        }
    }

    SensorHealth::SensorHealth(const SensorHealthConfig &cfg)
        : m_cfg(cfg)
    {
    }

    void SensorHealth::begin(bool ok, std::uint32_t nowUs)
    {
        m_status = ok ? SensorStatus::NOMINAL : SensorStatus::FAILED;
        m_successes = 0;
        m_failures = 0;
        m_lastSampleUs = nowUs;
        m_fresh = false;
    }

    void SensorHealth::record(bool readOk, bool newData, std::uint32_t nowUs)
    {
        if (m_status == SensorStatus::UNINITIALIZED)
        {
            m_fresh = false;
            return;
        }

        m_fresh = readOk && newData;

        if (!readOk)
        {
            onFailure();
        }
        else if (newData)
        {
            m_lastSampleUs = nowUs;
            onSuccess();
        }
        else
        {
            // The bus is fine and the sensor simply has nothing new yet. This
            // breaks a failure run but is not a success: recovery needs new data,
            // and only the staleness clock decides whether "nothing new" is a fault.
            m_failures = 0;
        }

        checkStaleness(nowUs);
    }

    void SensorHealth::idle(std::uint32_t nowUs)
    {
        m_fresh = false;
        if (m_status == SensorStatus::UNINITIALIZED)
            return;
        checkStaleness(nowUs);
    }

    void SensorHealth::degrade()
    {
        if (m_status == SensorStatus::NOMINAL)
        {
            m_status = SensorStatus::DEGRADED;
            m_successes = 0;
        }
    }

    void SensorHealth::onSuccess()
    {
        increment(m_successes);
        m_failures = 0;

        switch (m_status)
        {
        case SensorStatus::FAILED:
            if (m_successes >= m_cfg.recoverAfterSuccesses)
            {
                m_status = SensorStatus::DEGRADED;
                m_successes = 0; // full recovery counts from here, not from FAILED
            }
            break;
        case SensorStatus::DEGRADED:
            if (m_successes >= m_cfg.fullRecoverAfterSuccesses)
                m_status = SensorStatus::NOMINAL;
            break;
        default: // NOMINAL: nothing to recover
            break;
        }
    }

    void SensorHealth::onFailure()
    {
        increment(m_failures);
        m_successes = 0;

        switch (m_status)
        {
        case SensorStatus::NOMINAL:
            if (m_failures >= m_cfg.failAfterFailures)
                m_status = SensorStatus::FAILED;
            else if (m_failures >= m_cfg.degradeAfterFailures)
                m_status = SensorStatus::DEGRADED;
            break;
        case SensorStatus::DEGRADED:
            if (m_failures >= m_cfg.failAfterFailures)
                m_status = SensorStatus::FAILED;
            break;
        default: // FAILED: already at the bottom
            break;
        }
    }

    void SensorHealth::checkStaleness(std::uint32_t nowUs)
    {
        if (m_fresh)
            return;

        // Unsigned subtraction stays correct across the ~71 min micros() wrap.
        const std::uint32_t ageUs = nowUs - m_lastSampleUs;

        if (m_cfg.staleFailUs != 0 && ageUs >= m_cfg.staleFailUs)
        {
            if (m_status != SensorStatus::FAILED)
            {
                m_status = SensorStatus::FAILED;
                m_successes = 0;
            }
        }
        else if (m_cfg.staleDegradeUs != 0 && ageUs >= m_cfg.staleDegradeUs)
        {
            if (m_status == SensorStatus::NOMINAL)
            {
                m_status = SensorStatus::DEGRADED;
                m_successes = 0;
            }
        }
    }
}
