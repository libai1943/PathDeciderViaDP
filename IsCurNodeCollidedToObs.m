function is_collided = IsCurNodeCollidedToObs(cur_node)
global params_
is_collided = 1;
s0 = cur_node.cur_s - params_.dp.ds;
s1 = cur_node.cur_s;
l0 = cur_node.parent_l;
l1 = cur_node.cur_l;

distance = hypot(s1 - s0, l1 - l0);
nfe = 2 + ceil(distance / params_.dp.unit_s_for_resampling);
s_list = linspace(s0, s1, nfe);
l_list = linspace(l0, l1, nfe);

obs_x = [];
obs_y = [];

for ii = 1 : size(params_.dp.demo.obstacle_radii, 2)
    [cir_x, cir_y] = GenerateCircularObsBarrier(params_.dp.demo.obstacle_centers(ii, 1), params_.dp.demo.obstacle_centers(ii, 2), params_.dp.demo.obstacle_radii(ii));
    obs_x = [obs_x, cir_x];
    obs_y = [obs_y, cir_y];
end

subtle_s_list = linspace(s0 - params_.vehicle.lr, s1 + params_.vehicle.lw + params_.vehicle.lf, nfe + 10); % Magic number 10
for ii = 1 : length(subtle_s_list)
    cur_subtle_s = subtle_s_list(ii);
    obs_x = [obs_x, cur_subtle_s];
    obs_y = [obs_y, GetRoadRightBound(cur_subtle_s)];
    obs_x = [obs_x, cur_subtle_s];
    obs_y = [obs_y, GetRoadLeftBound(cur_subtle_s)];
end

for ii = 1 : nfe
    cur_subtle_s = s_list(ii);
    cur_subtle_l = l_list(ii);
    V_ego_vehicle = CreateVehiclePolygon(cur_subtle_s, cur_subtle_l, 0, 2);
    if (any(inpolygon(obs_x, obs_y, V_ego_vehicle.x, V_ego_vehicle.y)))
        return;
    end
end
is_collided = 0;
if (params_.dp.utility.enable_plot_valid_connections_between_adjacent_layers)
    plot([s0 s1], [l0 l1], 'g');
end
end