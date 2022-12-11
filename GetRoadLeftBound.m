function l = GetRoadLeftBound(s)
l = 3.0 + 0.5 * sin(0.9 * s - 0.01 * s.^2);
end