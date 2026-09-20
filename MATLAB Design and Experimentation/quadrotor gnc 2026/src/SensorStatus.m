classdef SensorStatus
    %SENSORSTATUS Health status for a single sensor reading.
    %
    %   Mirrors the SensorStatus enum already used on the C++/rocket side
    %   (UNINITIALIZED/NOMINAL/DEGRADED/FAILED) so the same vocabulary
    %   carries across the MATLAB prototype and the eventual port.
    %
    %   Kept as a plain MATLAB enumeration for now. If/when this gets
    %   wired into Simulink bus objects, consider re-basing this on
    %   Simulink.IntEnumType instead -- not done here since that's a
    %   Simulink-integration concern, not part of the interface design.

    enumeration
        UNINITIALIZED
        NOMINAL
        DEGRADED
        FAILED
    end
end