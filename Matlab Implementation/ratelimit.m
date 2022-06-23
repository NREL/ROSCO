function [rate] = ratelimit(inputSignal, inputSignalPrev, minRate, maxRate, DT)

    rate = (inputSignal - inputSignalPrev)/DT;      % Signal rate (unsaturated)
    rate = saturate(rate, minRate, maxRate);        % Saturate the signal rate
    rate = inputSignalPrev + rate*DT;               % Saturate the overall command using the rate limit
end