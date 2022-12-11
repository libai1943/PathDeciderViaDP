function InitializeParams()
global params_

params_.vehicle.lw = 2.8; % wheelbase
params_.vehicle.lf = 0.96; % front hang length
params_.vehicle.lr = 0.929; % rear hang length
params_.vehicle.lb = 1.942; % width
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;

params_.dp.time_horizon = 10.0; % T_max is set to 10 seconds
params_.dp.v_max = 10.0;
params_.dp.s_horizon = params_.dp.time_horizon * params_.dp.v_max * 1.2;
params_.dp.unit_s_for_resampling = 0.2;

params_.dp.max_dl = 3.0; % maximum dl/ds
params_.dp.max_ddl = 10.0; % maximum ddl/ds

params_.dp.ns = 30; % number of layers
params_.dp.nl = 10; % number of nodes in each layer
% Herein, ns means Number of Station layers while nl means Number of Lateral offset nodes in each layer.

params_.dp.ds = params_.dp.s_horizon / params_.dp.ns; % station gap between adjacent layers.
params_.dp.lateral_offset_rate_list = linspace(0.0, 1.0, params_.dp.nl);

params_.dp.weight.l = 1.0;
params_.dp.weight.dl = 1.0;
params_.dp.weight.ddl = 1.0;
params_.dp.weight.dddl = 1.0;
params_.dp.weight.norm_speed = 1.0;

params_.dp.utility.enable_plot_valid_connections_between_adjacent_layers = 1;
end