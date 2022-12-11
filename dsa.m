function dsa()
global params_
figure(2)
hold on;
grid minor;
axis equal;
axis tight;
s_list = 0 : 0.01 : params_.dp.s_horizon;
plot(s_list, GetRoadRightBound(s_list), 'r', 'LineWidth', 2);
plot(s_list, GetRoadLeftBound(s_list), 'r', 'LineWidth', 2);
xlabel('x axis / m', 'Fontsize', 20, 'FontWeight', 'bold')
ylabel('y axis / m', 'Fontsize', 20, 'FontWeight', 'bold')
set(gca, 'FontSize', 12, 'FontWeight', 'bold')
set(gcf, 'outerposition', get(0,'screensize'));
plot([0 params_.dp.s_horizon], [0 0], 'k--');

for ii = 1 : size(params_.dp.demo.obstacle_radii, 2)
    [cir_x, cir_y] = GenerateCircularObsBarrier(params_.dp.demo.obstacle_centers(ii, 1), params_.dp.demo.obstacle_centers(ii, 2), params_.dp.demo.obstacle_radii(ii));
    fill(cir_x, cir_y, [0 0 0]);
end

s = params_.s;
l = params_.l;
nfe = round(s(end) / 1.0);
[s, l] = ResampleProfiles(s, l, nfe);

for ii = 1 : nfe
    v_ego_vehicle = CreateVehiclePolygon(s(ii), l(ii), 0, 2);
    plot(v_ego_vehicle.x, v_ego_vehicle.y, 'b');
end
plot(s, l, 'LineWidth', 3, 'Color', [34 177 76] ./ 255);
end