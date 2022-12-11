function l = GetRoadRightBound(s)
% Note. This function is just an example to descirbe what an irregularly
% shaped road barrier could be. Do not try to investigate why the function
% is defined like this. It is not the main focus of this share of code.
l = -3.0 - 0.1 * cos(1.0009 * s + 0.001 * s.^1.5);
end