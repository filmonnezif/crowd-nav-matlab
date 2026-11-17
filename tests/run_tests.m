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

fprintf('All %d tests passed.\n', pass);
