function cost = CalculateCost(cur_node)
global params_
%% Discard excessive dl
if (abs(cur_node.cur_dl) > params_.dp.max_dl)
    cost = 1e20;
    return;
end

%% Discard excessive ddl
if (abs(cur_node.cur_ddl) > params_.dp.max_ddl)
    cost = 1e20;
    return;
end

%% Discard collisions
if (IsCurNodeCollidedToObs(cur_node))
    cost = 1e20;
    return;
end

cost = 0;
%% Penalize excessive dl, ddl, and dddl
cost = cost + params_.dp.weight.dl * abs(cur_node.cur_dl);
cost = cost + params_.dp.weight.ddl * abs(cur_node.cur_ddl);
cost = cost + params_.dp.weight.dddl * abs(cur_node.cur_dddl);

%% Penalize biased from current lane center
cost = cost + params_.dp.weight.l * abs(cur_node.cur_l - 0.0);
end