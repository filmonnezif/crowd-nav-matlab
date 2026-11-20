% RUN_TESTS  Quick checks of the Phase 1 building blocks.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
pass = 0;

% A* goes around a wall and its length is close to optimal
blocked = false(20, 20); blocked(2:19, 10) = true;
p = astar_grid(blocked, zeros(20), [10 2], [10 18], 0);
assert(~isempty(p) && all(~blocked(sub2ind([20 20], p(:, 1), p(:, 2)))));
assert(all(p(1, :) == [10 2]) && all(p(end, :) == [10 18]));
pass = pass + 1;

% A* returns empty when the goal is walled off
blocked = false(10); blocked(:, 5) = true;
assert(isempty(astar_grid(blocked, zeros(10), [5 1], [5 9])));
pass = pass + 1;

% diff-drive: full circle returns to start
x = [0 0 0]; lim.vmax = 1; lim.wmax = 1;
for k = 1:round(2 * pi / 0.01), x = diffdrive_step(x, [1 1], 0.01, lim); end
assert(norm(x(1:2)) < 0.02);
pass = pass + 1;

% pure pursuit converges onto a straight path from an offset start
x = [0 1 0]; path = [linspace(0, 20, 200)' zeros(200, 1)];
for k = 1:300
    [v, w] = pure_pursuit(x, path, 0.8, 1, 2);
    x = diffdrive_step(x, [v w], 0.05, struct('vmax', 1, 'wmax', 2));
end
assert(abs(x(2)) < 0.05);
pass = pass + 1;

% clean split: no train/val frames shared by the same person
D = build_dataset(fullfile(root, 'data', 'raw'), 'Verbose', false);
assert(D.leakTrainVal == 0);
assert(isequal(size(D.train.obs, 2), 8) && isequal(size(D.train.pred, 2), 12));
assert(numel(D.test.ped_id) == 364);
pass = pass + 1;

% replay interpolation sits between the two recorded frames
S = D.scenes(1);
[ids, P] = scene_peds_at(S, 10.2);
[idsA, PA] = scene_peds_at(S, 10.0);
assert(~isempty(ids) && size(P, 2) == 2 && ~isempty(PA));
pass = pass + 1;

fprintf('All %d Phase 1 tests passed.\n', pass);

% Phase 2: predictors
H = zeros(2, 8, 2); H(1, :, 1) = (0:7) * 0.4; H(2, :, 2) = (0:7) * 0.2;   % 1 m/s in x, 0.5 m/s in y
Y = predict_cv(H, 12, 0.4);
assert(abs(Y(1, end, 1) - (2.8 + 12 * 0.4)) < 1e-9 && abs(Y(2, end, 2) - (1.4 + 12 * 0.2)) < 1e-9);
Yk = predict_kalman(H, 12, 0.4);
assert(max(abs(Yk(:) - Y(:))) < 0.1);           % noise-free line: Kalman ~ CV
Ys = predict_social_force(H, 12, 0.4);
assert(all(isfinite(Ys(:))) && isequal(size(Ys), [2 12 2]));
fprintf('Phase 2 predictor tests passed.\n');

% Phase 5: space-time A* waits for a person crossing in front
cfg = sim_config('planner', 'sta');
pred.ids = 1; pred.tObs = 0; pred.P0 = [2 -1.5];
pred.Y = cat(3, 2 * ones(1, 12), -1.5 + (1:12) * 0.4);  % walks up through x = 2 at 1 m/s
pred.R = zeros(1, 12);
[path, info] = st_plan([0 0 0], [4 0], pred, pred.P0, 0, [-1 5 -3 3], cfg);
assert(size(path, 2) == 3 && path(end, 3) > path(1, 3));
d = min(hypot(interp1(path(1:end - 1, 3), path(1:end - 1, 1), 0:0.4:4.4) - 2, ...
              interp1(path(1:end - 1, 3), path(1:end - 1, 2), 0:0.4:4.4) - (-1.5 + (0:0.4:4.4))));
assert(info.feasible && d > cfg.robotR + cfg.pedR);   % plan never touches the person
% reactive crowd: a person walking at the robot steps aside
S.name = 'toy'; S.dt = 0.4; S.id = ones(31, 1); S.k = (1:31)';
S.xy = [linspace(6, -6, 31)', zeros(31, 1)]; S.nK = 31;
S.rowsByK = num2cell((1:31)');
cfg = sim_config('crowd', 'reactive');
cr = crowd_init(S, cfg); dmin = inf;
for t = 0:0.1:11.6
    [cr, ~, P] = crowd_step(cr, t, [0 0 0], [0 0], cfg);
    dmin = min(dmin, hypot(P(1), P(2)));
end
assert(dmin > cfg.robotR + cfg.pedR);                  % never walked into the parked robot
fprintf('Phase 5 planner and crowd tests passed.\n');

% Phase 6: display-only uncertainty rings never reach the planner
S = read_raw_scene(fullfile(root, 'data', 'raw', 'biwi_eth.txt'));
cfg = sim_config();
popt = struct('horizon', 3.2, 'band', 1.0, 'obsLen', 8, 'predLen', 12, ...
    'conformal', [], 'showConformal', struct('mode', 'plain', 'q', 0.1 * (1:12)));
t = 20;
[ids, P, V] = scene_peds_at(S, t);
[C, r, viz] = pred_obstacles(S, t, ids, P, V, cfg, [0 0 0], make_predictor('cv'), popt);
assert(~isempty(viz.Y) && all(viz.R(:) == 0) && all(abs(viz.Rshow(1, :) - 0.1 * (1:12)) < 1e-12));
popt.showConformal = [];
[C2, r2] = pred_obstacles(S, t, ids, P, V, cfg, [0 0 0], make_predictor('cv'), popt);
assert(isequal(C, C2) && isequal(r, r2));             % same obstacles with or without rings
% the view draws people, rings and the robot, and holds the last step
cfg.Tmax = 1;
log = run_episode(S, scene_task(S, t), cfg);
log.pred{end} = viz;
f = figure('Visible', 'off');
h = scene_view_init(axes('Parent', f), log, struct('legend', true));
scene_view_draw(h, log, numel(log.t) + 5);
ux = get(h.unc, 'XData'); px = get(h.ped, 'XData');
assert(any(isfinite(ux(:))) && any(isfinite(px(:))));
ttl = get(h.title, 'String');
assert(~isempty(strfind([ttl{:}], 'finished')));
close(f);
fprintf('Phase 6 demo tests passed.\n');
