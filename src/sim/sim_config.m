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
    'obstacleFcn', []    ... % hook: @(S,t,ids,P,V,cfg,x) -> [centers radii viz], see make_obstacle_fcn
    );
cfg = parse_opts(cfg, varargin{:});
end
