function motorCmds = mixMotors(throttle, rollTorque, pitchTorque, ...
    yawTorque, mixMatrix ...
    )
    commandVector = [throttle; rollTorque; pitchTorque; yawTorque];
    motorCmds = mixMatrix * commandVector;
    
    % Physical motor commands must stay within [0, 1] (0% to 100% thrust).
    % Handle a command that dips below 0 first, by shifting ALL FOUR motors
    % up equally -- this preserves the differences between motors, which is
    % what actually encodes the roll/pitch/yaw ratio, rather than clipping
    % just the one offending motor and distorting that ratio.
    lowest = min(motorCmds);
    if lowest < 0
        motorCmds = motorCmds - lowest;  % lowest is negative, so this adds |lowest| to all four
    end
    
    % Then handle a command that's too high, by scaling ALL FOUR motors down
    % proportionally -- same reasoning: preserves the relative differences
    % between motors instead of clipping just the one motor over the limit.
    highest = max(motorCmds);
    if highest > 1
        motorCmds = motorCmds / highest;
    end
end