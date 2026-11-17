function cfg = sim_config(varargin)
%SIM_CONFIG  Default robot, planner and controller settings.
%   cfg = sim_config('vmax', 0.8, 'replanEvery', 0.4)
cfg = struct( ...
    'dt',          0.1,  ... % simulation step (s)
    'Tmax',        60,   ... % episode time limit (s)
    'vmax',        1.0,  ... % robot top speed (m/s), a brisk walk is 1.3
    'wmax',        2.0,  ... % robot top turn rate (rad/s)
    'robotR',      0.30, ... % robot radius (m)
    'pedR',        0.30, ... % pedestrian body radius (m)
    'margin',      0.20, ... % extra safety gap added to the hard obstacle (m)
    'socialR',     0.80, ... % soft cost reaches this far beyond the hard obstacle (m)
    'costWeight',  3,    ... % how much A* pays for the soft cost
    'res',         0.25, ... % grid resolution (m)
    'replanEvery', 0.4,  ... % A* replanning period (s), = one data frame
    'lookahead',   0.8,  ... % pure pursuit look-ahead distance (m)
    'goalTol',     0.3,  ... % goal reached within this distance (m)
    'dSlow',       1.0,  ... % start slowing when clearance to a person < dSlow (m)
    'dStop',       0.15, ... % stop when clearance < dStop (m)
    'controller',  'pp', ... % 'pp' = pure pursuit + speed governor, 'dwa' = dynamic window
    'vmin',       -0.3,  ... % DWA: slowest (reverse) speed allowed (m/s)
    'accV',        1.5,  ... % DWA: linear acceleration limit (m/s^2)
    'accW',        4.0,  ... % DWA: angular acceleration limit (rad/s^2)
    'dwaReact',    0.4,  ... % DWA: window = commands reachable within this time (s)
    'dwaHorizon',  2.0,  ... % DWA: rollout length (s)
    'dwaTarget',   2.0,  ... % DWA: aim at the A* path point this far ahead (m)
    'dwaSafe',     0.05, ... % DWA: required clearance during the rollout (m)
    'dwaComfort',  0.6,  ... % DWA: clearance below this is penalised (m)
    'dwaWProg',    1.0,  ... % DWA cost weights
    'dwaWClear',   3.0,  ...
    'dwaWSpeed',   0.2,  ...
    'dwaWSmooth',  0.05, ...
    'obstacleFcn', []    ... % hook: @(S,t,ids,P,V,cfg,x) -> [centers radii viz], see make_obstacle_fcn
    );
cfg = parse_opts(cfg, varargin{:});
end
