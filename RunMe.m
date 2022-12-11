clear all; close all; clc;
global params_

InitializeParams();

% Initial state of the ego vehicle
params_.task.l0 = 0;
params_.task.dl0 = 0;
params_.task.ddl0 = 0;

% Setup of obstacles (DP path only considers static obstacles)
params_.dp.demo.obstacle_centers = [12.0, -2.0; 37, -2; 72, -1.1; 82, 1.1];
params_.dp.demo.obstacle_radii = [1.1, 0.4, 0.4, 0.37];

[s, l] = PathPlanningViaDP();
params_.s = s;
params_.l = l;

% % Plot path planning result with vehicle footprints
dsa();