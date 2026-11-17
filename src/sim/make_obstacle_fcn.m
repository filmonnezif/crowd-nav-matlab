function fn = make_obstacle_fcn(predName, popt)
%MAKE_OBSTACLE_FCN  Planner obstacle hook for a predictor.
%   cfg.obstacleFcn = make_obstacle_fcn('kalman')
%   'none' returns [] (Phase 1 behaviour: current positions only).
%   make_obstacle_fcn('kalman', struct('alpha', 0.1, 'cpMode', 'speed'))
%   adds 90% conformal uncertainty circles (Phase 4).
if nargin < 2, popt = struct(); end
d = struct('horizon', 3.2, 'band', 1.0, 'obsLen', 8, 'predLen', 12, ...
    'alpha', [], 'cpMode', 'plain', 'conformal', []);
f = fieldnames(d);
for k = 1:numel(f)
    if ~isfield(popt, f{k}), popt.(f{k}) = d.(f{k}); end
end
if strcmpi(predName, 'none')
    fn = [];
    return
end
pf = make_predictor(predName);
if ~isempty(popt.alpha) && isempty(popt.conformal)
    popt.conformal = get_conformal(predName, popt.alpha, popt.cpMode);
end
fn = @(S, t, ids, P, V, cfg, x) pred_obstacles(S, t, ids, P, V, cfg, x, pf, popt);
end
