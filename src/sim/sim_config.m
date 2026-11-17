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
    'controller',  'pp', ... % 'pp' = pure pursuit + speed governor, 'dwa' = dynamic window, 'mpc' = sampling MPC
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
    'planner',     'astar', ... % 'astar' = 2-D A* (Phase 1), 'sta' = space-time A* (Phase 5)
    'stRes',       0.30, ... % space-time A*: grid resolution (m)
    'stDt',        0.40, ... % space-time A*: time step (s); diagonal move = 1.06 m/s
    'stLayers',    12,   ... % space-time A*: steps ahead (12 x 0.4 = 4.8 s)
    'stBlocked',   20,   ... % space-time A*: cost of entering a blocked cell (s)
    'mpcN',        8,    ... % MPC: commands in the horizon
    'mpcDt',       0.4,  ... % MPC: each command held this long (s) -> 3.2 s
    'mpcSamples',  120,  ... % MPC: rollouts per iteration (cross-entropy method)
    'mpcIters',    3,    ... % MPC: iterations
    'mpcElite',    0.1,  ... % MPC: share of best rollouts kept
    'mpcSigV',     0.3,  ... % MPC: initial sampling spread of v (m/s)
    'mpcSigW',     0.8,  ... % MPC: initial sampling spread of w (rad/s)
    'mpcWTrack',   1.0,  ... % MPC cost weights
    'mpcWClear',   6.0,  ...
    'mpcComfort',  0.5,  ...
    'mpcHit',      5.0,  ...
    'mpcWSmooth',  0.2,  ...
    'mpcWTurn',    0.02, ...
    'crowd',       'replay', ... % 'replay' = recorded people, 'reactive' = Social Force people who avoid the robot
    'crowdLook',   1.2,  ... % reactive crowd: head for the recorded position this far ahead (s)
    'crowdTau',    0.5,  ... % reactive crowd: velocity relaxation time (s)
    'crowdAr',     3.0,  ... % reactive crowd: robot repulsion strength (m/s^2)
    'crowdBr',     0.35, ... % reactive crowd: robot repulsion range (m)
    'crowdAp',     1.5,  ... % reactive crowd: person-person repulsion strength (m/s^2)
    'crowdBp',     0.25, ... % reactive crowd: person-person repulsion range (m)
    'crowdSide',   0.6,  ... % reactive crowd: sideways (pass-on-the-right) share of the robot push
    'crowdLambda', 0.35, ... % reactive crowd: weight of things behind a person (1 = all around)
    'crowdVmax',   2.0,  ... % reactive crowd: top walking speed (m/s)
    'seed',        1,    ... % random seed (MPC sampling)
    'obstacleFcn', []    ... % hook: @(S,t,ids,P,V,cfg,x) -> [centers radii viz], see make_obstacle_fcn
    );
cfg = parse_opts(cfg, varargin{:});
end
