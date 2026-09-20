classdef StateEstimator < handle
    properties (SetAccess = {?StateEstimator, ?StateEstimatorTest})
        Quaternion
        AhrsSource          % "Seeding" | "Nominal" | "FallbackComplementary"
        MadgwickVariant     % "Full" | "NoMag"
        VerticalKFInstance
    end

    properties (Access = private)
        CompQuaternion
        MadgwickQuaternion
        MadgwickBeta
        CompAlpha           % weight on the complementary filter's gyro branch (e.g. 0.98)
        Gravity             % m/s^2, for verticalAccelFromBody's gravity compensation
        SeedingElapsed
        SeedingDuration
        InstabilityNormTol
    end

    methods
        function obj = StateEstimator(madgwickBeta, compAlpha, gravity, kfParams, ...
                seedingDuration, instabilityNormTol)
            obj.MadgwickBeta = madgwickBeta;
            obj.CompAlpha = compAlpha;
            obj.Gravity = gravity;
            obj.SeedingDuration = seedingDuration;
            obj.InstabilityNormTol = instabilityNormTol;

            obj.CompQuaternion = Quaternion.identity();
            obj.MadgwickQuaternion = Quaternion.identity();
            obj.Quaternion = Quaternion.identity();

            obj.AhrsSource = "Seeding";
            obj.MadgwickVariant = "Full";
            obj.SeedingElapsed = 0;

            obj.VerticalKFInstance = VerticalKF(kfParams{:});
        end

        function stateEstimate = update(obj, sensorReadings, dt)
            obj.CompQuaternion = complementaryFilterStep( ...
                obj.CompQuaternion, sensorReadings.gyro, sensorReadings.accel, dt, obj.CompAlpha);

            switch obj.AhrsSource
                case "Seeding"
                    obj.SeedingElapsed = obj.SeedingElapsed + dt;
                    if obj.SeedingElapsed >= obj.SeedingDuration
                        obj.MadgwickQuaternion = obj.CompQuaternion;
                        obj.AhrsSource = "Nominal";
                    end
                    obj.Quaternion = obj.CompQuaternion;

                case "Nominal"
                    magUsable = sensorReadings.magStatus == SensorStatus.NOMINAL;
                    if magUsable
                        obj.MadgwickVariant = "Full";
                        qNext = madgwickStepFull(obj.MadgwickQuaternion, ...
                            sensorReadings.gyro, sensorReadings.accel, sensorReadings.mag, ...
                            dt, obj.MadgwickBeta);
                    else
                        obj.MadgwickVariant = "NoMag";
                        qNext = madgwickNoMag(obj.MadgwickQuaternion, ...
                            sensorReadings.gyro, sensorReadings.accel, dt, obj.MadgwickBeta);
                    end

                    if StateEstimator.detectInstability(qNext, obj.InstabilityNormTol)
                        obj.AhrsSource = "FallbackComplementary";
                        obj.Quaternion = obj.CompQuaternion;
                    else
                        obj.MadgwickQuaternion = qNext;
                        obj.Quaternion = obj.MadgwickQuaternion;
                    end

                case "FallbackComplementary"
                    obj.Quaternion = obj.CompQuaternion;
            end

            aVert = verticalAccelFromBody(sensorReadings.accel, obj.Quaternion, obj.Gravity);
            obj.VerticalKFInstance.predict(aVert, dt);
            if sensorReadings.baroStatus == SensorStatus.NOMINAL
                obj.VerticalKFInstance.correct(sensorReadings.baroAltitude);
            end

            eulerAngles = obj.Quaternion.toEulerZYX();

            stateEstimate = StateEstimate( ...
                'angles',           eulerAngles, ...
                'rates',            sensorReadings.gyro, ...
                'altitude',         obj.VerticalKFInstance.x(1), ...
                'verticalVelocity', obj.VerticalKFInstance.x(2));
        end

        function tf = isConverged(obj)
            tf = obj.AhrsSource == "Nominal";
        end

        function reset(obj)
            obj.MadgwickQuaternion = obj.CompQuaternion;
            obj.AhrsSource = "Nominal";
            obj.SeedingElapsed = 0;
        end

        function tf = isDegraded(obj)
            tf = obj.AhrsSource == "FallbackComplementary";
        end
    end

    methods (Static)
        function tf = detectInstability(q, tol)
            qVec = [q.w, q.x, q.y, q.z];
            hasNonFinite = any(isnan(qVec)) || any(isinf(qVec));  % Infinity check
            normUnstable = abs(q.norm() - 1) > tol; % if normalized quaternion way off of acceptable tolerance
            tf = hasNonFinite || normUnstable;
        end
    end
end