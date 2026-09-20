classdef MockController < handle
    properties
        ResetCallCount = 0
    end
    methods
        function reset(obj)
            obj.ResetCallCount = obj.ResetCallCount + 1;
        end
    end
end