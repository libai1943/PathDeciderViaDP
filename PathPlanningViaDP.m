function [s, l] = PathPlanningViaDP()
global params_
if (params_.dp.utility.enable_plot_valid_connections_between_adjacent_layers)
    asd();
end

NS = params_.dp.ns;
NL = params_.dp.nl;

% % Zero-layer node definition
zero_layer_node.cost = 0.0;
zero_layer_node.parent_id = [-999, -999];
zero_layer_node.cur_s = 0.0;
zero_layer_node.cur_l = params_.task.l0;
zero_layer_node.cur_dl = params_.task.dl0;
zero_layer_node.cur_ddl = params_.task.ddl0;
zero_layer_node.cur_dddl = 0.0;
zero_layer_node.parent_l = params_.task.l0;
zero_layer_node.parent_dl = params_.task.dl0;
zero_layer_node.parent_ddl = params_.task.ddl0;

predefined_node = zero_layer_node;
predefined_node.cost = Inf;
state_space = repmat(predefined_node, NS, NL);

% % Enumeration on the first layer
drivable_left_lateral_offset = GetRoadLeftBound(params_.dp.ds * 1) - 0.5 * params_.vehicle.lb;
drivable_right_lateral_offset = GetRoadRightBound(params_.dp.ds * 1) + 0.5 * params_.vehicle.lb;
drivable_lateral_length = drivable_left_lateral_offset - drivable_right_lateral_offset;

for jj = 1 : NL
    cur_node.cur_l = drivable_right_lateral_offset + drivable_lateral_length * params_.dp.lateral_offset_rate_list(jj);
    cur_node.cur_s = params_.dp.ds * 1;
    cur_node.parent_l = zero_layer_node.cur_l;
    cur_node.parent_dl = zero_layer_node.cur_dl;
    cur_node.parent_ddl = zero_layer_node.cur_ddl;
    [cur_node.cur_dl, cur_node.cur_ddl, cur_node.cur_dddl] = GetStatesOfCurrentNode(cur_node);
    cur_node.cost = CalculateCost(cur_node) + zero_layer_node.cost;
    cur_node.parent_id = [0, 0];
    state_space(1, jj) = cur_node;
end

for ii = 1 : (NS - 1)
    cur_node.cur_s = params_.dp.ds * (ii+1);
    drivable_left_lateral_offset = GetRoadLeftBound(cur_node.cur_s) - 0.5 * params_.vehicle.lb;
    drivable_right_lateral_offset = GetRoadRightBound(cur_node.cur_s) + 0.5 * params_.vehicle.lb;
    drivable_lateral_length = drivable_left_lateral_offset - drivable_right_lateral_offset;
    for jj = 1 : NL
        parent_node_candidate = state_space(ii, jj);
        for kk = 1 : NL
            cur_node.cur_l = drivable_right_lateral_offset + drivable_lateral_length * params_.dp.lateral_offset_rate_list(kk);
            cur_node.parent_l = parent_node_candidate.cur_l;
            cur_node.parent_dl = parent_node_candidate.cur_dl;
            cur_node.parent_ddl = parent_node_candidate.cur_ddl;
            [cur_node.cur_dl, cur_node.cur_ddl, cur_node.cur_dddl] = GetStatesOfCurrentNode(cur_node);
            cur_node.cost = CalculateCost(cur_node) + parent_node_candidate.cost;
            obj = state_space(ii + 1, kk);
            if (cur_node.cost < obj.cost)
                cur_node.parent_id = [ii, jj]; % Change father of Node(ii+1,kk) to (ii,jj)
                state_space(ii + 1, kk) = cur_node;
            end
        end
    end
end

cur_best_cost = state_space(NS, 1).cost;
cur_best_l_ind = 1;
for ii = 2 : NL
    if (state_space(NS, ii).cost < cur_best_cost)
        cur_best_cost = state_space(NS, ii).cost;
        cur_best_l_ind = ii;
    end
end

disp('Cost derived by DP search = ');
disp(cur_best_cost);
if (cur_best_cost > 1e10)
    disp('Invalid DP search result.');
end

% % Node backtrack 1
s = [];
l = [];
ptr = NS;
while (ptr > 0)
    elem = state_space(ptr, cur_best_l_ind);
    s = [elem.cur_s, s];
    l = [elem.cur_l, l];
    cur_best_l_ind = elem.parent_id(2);
    ptr = ptr - 1;
end
s = [0, s];
l = [params_.task.l0, l];
end