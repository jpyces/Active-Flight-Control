#pragma once

#include "SensorStatus.h"

namespace gnc
{
    // Contract every sensor driver implements. A driver must report both whether
    // the sensor is healthy and whether its latest read produced new data; bus
    // connectivity alone is not enough. Implementing this through a SensorHealth
    // member is the intended pattern.
    class SensorInterface
    {
    public:
        // Health: responding and trustworthy (debounced, includes staleness timeout).
        virtual SensorStatus getStatus() const = 0;

        // Freshness: the most recent read returned a sample not delivered before.
        // Drivers are read exactly once per loop tick, so this means "new this tick".
        virtual bool isFresh() const = 0;

    protected:
        // Never deleted through this type (no heap allocation), so the destructor
        // is protected and non-virtual.
        SensorInterface() = default;
        ~SensorInterface() = default;
        SensorInterface(const SensorInterface &) = default;
        SensorInterface &operator=(const SensorInterface &) = default;
    };
}
