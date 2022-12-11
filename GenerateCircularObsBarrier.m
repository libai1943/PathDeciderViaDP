function [cir_x, cir_y] = GenerateCircularObsBarrier(x, y, r)
cir_x = x + cosd(0 : 30 : 360) * r;
cir_y = y + sind(0 : 30 : 360) * r;
end