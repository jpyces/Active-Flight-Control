function measurements = stateEstimateToControllerMeasurements(stateEstimate)
    measurements.angle    = stateEstimate.angles;
    measurements.rate     = stateEstimate.rates;
    measurements.altitude = stateEstimate.altitude;
    % verticalVelocity isn't consumed by AttitudeAltitudeController today
    % (altitude loop only reads .altitude) -- dropped here, not an oversight.
end