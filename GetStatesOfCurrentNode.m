function [cur_dl, cur_ddl, cur_dddl] = GetStatesOfCurrentNode(cur_node)
global params_
l1 = cur_node.cur_l;
l0 = cur_node.parent_l;
dl0 = cur_node.parent_dl;
ddl0 = cur_node.parent_ddl;

cur_dl = (l1 - l0) / params_.dp.ds;
cur_ddl = (cur_dl - dl0) / params_.dp.ds;
cur_dddl = (cur_ddl - ddl0) / params_.dp.ds;
end