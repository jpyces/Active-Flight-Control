#pragma once

namespace gnc {
    // Result of any externally-run check. The module running the check owns its
    // own timeout and reports Failed itself.
    enum class CheckResult
    {
        Pending, // still running (or not started)
        Passed,
        Failed
    };
}