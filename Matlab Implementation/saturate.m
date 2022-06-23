function [outValue] = saturate(inputValue, minValue, maxValue)
% Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue

outValue = min(max(inputValue,minValue), maxValue);

end