classdef MockStateEstimator < handle
    properties
        Converged = false
        Degraded = false
        ResetCallCount = 0
    end
    methods
        function tf = isConverged(obj)
            tf = obj.Converged;
        end
        function tf = isDegraded(obj)
            tf = obj.Degraded;
        end
        function reset(obj)
            obj.ResetCallCount = obj.ResetCallCount + 1;
            obj.Degraded = false;
        end
    end
end