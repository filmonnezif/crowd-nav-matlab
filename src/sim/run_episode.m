function [log, m] = run_episode(S, task, cfg)
%RUN_EPISODE  Drive the robot from task.start to task.goal through a
%   replayed crowd.
%   [log, metrics] = run_episode(S, task, cfg)
%
%   Loop (every cfg.dt):
%     1. read the crowd at time t from the recording (scene_peds_at)
%     2. every cfg.replanEvery s: turn people into obstacles (by default
%        their current positions; with cfg.obstacleFcn =
%        make_obstacle_fcn('kalman') etc. their predicted paths too),
%        build the cost map, run A*
%     3. pure pursuit on the A* path gives (v, w); a speed governor
%        scales v down as the nearest person gets close
%     4. integrate the differential-drive model
%   cfg.crowd = 'reactive' (Phase 5): people are simulated with Social
%   Force, follow their recorded routes and step around the robot; the
%   predictors then observe the simulated crowd.
%   cfg.planner = 'sta': space-time A* (st_plan) instead of 2-D A*.
%   cfg.controller = 'mpc': sampling MPC (mpc_control).
if nargin < 3, cfg = sim_config(); end
reactive = strcmpi(cfg.crowd, 'reactive');
if reactive
    crowd = crowd_init(S, cfg);
    ghost = crowd;            % same crowd with no robot: the reference for "disturbance"
end
useMpc = strcmpi(cfg.controller, 'mpc');
if useMpc, rng(cfg.seed + round(task.t0)); end
sol = [];
G = grid_make(task.bounds, cfg.res);
lim.vmax = cfg.vmax; lim.wmax = cfg.wmax; lim.vmin = 0;
if any(strcmpi(cfg.controller, {'dwa', 'mpc'})), lim.vmin = cfg.vmin; end
nSteps = round(cfg.Tmax / cfg.dt);
replanK = max(1, round(cfg.replanEvery / cfg.dt));

x = task.start(:)';
path = zeros(0, 2);
log.t = zeros(nSteps, 1);
log.x = zeros(nSteps, 3);
log.u = zeros(nSteps, 2);
log.clear = inf(nSteps, 1);
log.nPeds = zeros(nSteps, 1);
log.ids = cell(nSteps, 1);
log.P = cell(nSteps, 1);
log.path = cell(nSteps, 1);
log.planMs = [];
log.pred = cell(nSteps, 1);
log.dev = cell(nSteps, 1);
lastPred = [];
uPrev = [0 0];
reached = false;
for k = 1:nSteps
    t = task.t0 + (k - 1) * cfg.dt;
    if reactive
        [crowd, ids, P, V] = crowd_step(crowd, t, x, uPrev, cfg);
        [ghost, gids, GP] = crowd_step(ghost, t, [1e6 1e6 0], [0 0], cfg);
        [~, loc] = ismember(ids, gids);
        dev = hypot(P(:, 1) - GP(loc, 1), P(:, 2) - GP(loc, 2));
        S = crowd_write_frame(S, t, ids, P);
    else
        [ids, P, V] = scene_peds_at(S, t);
        dev = zeros(numel(ids), 1);
    end
    if isempty(P)
        clr = inf;
    else
        clr = min(hypot(P(:, 1) - x(1), P(:, 2) - x(2))) - cfg.robotR - cfg.pedR;
    end

    if mod(k - 1, replanK) == 0
        if isempty(cfg.obstacleFcn)
            C = P;
            r = repmat(cfg.pedR + cfg.robotR + cfg.margin, size(P, 1), 1);
        else
            [C, r, viz] = cfg.obstacleFcn(S, t, ids, P, V, cfg, x);
            lastPred = viz;
        end
        cells = [];
        if strcmpi(cfg.planner, 'sta')
            tic;
            path = st_plan(x, task.goal, lastPred, P, t, task.bounds, cfg);
            log.planMs(end + 1) = 1000 * toc;
        else
            [blocked, cost] = grid_costmap(G, C, r, cfg.socialR);
            tic;
            cells = astar_grid(blocked, cost, grid_world2cell(G, x(1:2)), ...
                grid_world2cell(G, task.goal), cfg.costWeight);
            log.planMs(end + 1) = 1000 * toc;
        end
        if ~isempty(cells)
            path = [G.X(sub2ind(size(G.X), cells(:, 1), cells(:, 2))), ...
                    G.Y(sub2ind(size(G.Y), cells(:, 1), cells(:, 2)))];
            path(end, :) = task.goal;
        end
        % no path found: keep following the last one; the governor
        % below stops the robot if someone is in the way.
    end

    dGoal = hypot(task.goal(1) - x(1), task.goal(2) - x(2));
    if strcmpi(cfg.controller, 'dwa')
        [v, w] = dwa_control(x, path, lastPred, P, t, uPrev, cfg);
        v = sign(v) * min(abs(v), max(dGoal, 0.2));
    elseif useMpc
        [v, w, sol] = mpc_control(x, path, lastPred, P, t, uPrev, sol, cfg);
        v = sign(v) * min(abs(v), max(dGoal, 0.2));
    elseif size(path, 2) >= 3
        [v, w] = time_pursuit(x, path, t, cfg);
        v = v * min(max((clr - cfg.dStop) / (cfg.dSlow - cfg.dStop), 0), 1);
    else
        [v, w] = pure_pursuit(x, path, cfg.lookahead, cfg.vmax, cfg.wmax);
        v = v * min(max((clr - cfg.dStop) / (cfg.dSlow - cfg.dStop), 0), 1);
        v = min(v, max(dGoal, 0.2));   % ease in to the goal
    end
    uPrev = [v w];

    log.t(k) = t; log.x(k, :) = x; log.u(k, :) = [v w];
    log.clear(k) = clr; log.nPeds(k) = numel(ids);
    log.ids{k} = ids; log.P{k} = P; log.path{k} = path; log.pred{k} = lastPred;
    log.dev{k} = dev;

    if dGoal < cfg.goalTol
        reached = true;
        break
    end
    x = diffdrive_step(x, [v w], cfg.dt, lim);
end
f = fieldnames(log);
for i = 1:numel(f)
    if strcmp(f{i}, 'planMs'), continue; end
    v = log.(f{i});
    log.(f{i}) = v(1:k, :);
end
log.task = task;
log.cfg = cfg;
log.scene = S.name;
m = episode_metrics(log, reached);
end
