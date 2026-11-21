% PAPER_SCENE_FIGURE  The scene snapshot used in the paper (Fig. 2):
% students003, reactive crowd, space-time A* + MPC with LSTM prediction,
% 90% conformal rings drawn at 1.6 s, at t = 12 s.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
S = read_raw_scene(fullfile(root, 'data', 'raw', 'students003.txt'));
t0s = linspace(0, (S.nK - 1) * S.dt - 60 - 1, 10);
task = scene_task(S, t0s(2), true);
cfg = sim_config('planner', 'sta', 'controller', 'mpc', 'crowd', 'reactive');
cfg.obstacleFcn = make_obstacle_fcn('lstm', struct('showAlpha', 0.1));
log = run_episode(S, task, cfg);
f = figure('Color', 'w', 'Position', [60 60 820 520], 'Visible', 'off');
h = scene_view_init(axes('Parent', f), log, struct('legend', true, 'rings', 4, 'label', 'LSTM + space-time A* + MPC'));
scene_view_draw(h, log, round(12 / cfg.dt) + 1);
set(findall(f, '-property', 'FontSize'), 'FontSize', 10);
if ~exist(fullfile(root, 'results', 'paper'), 'dir'), mkdir(fullfile(root, 'results', 'paper')); end
print(f, fullfile(root, 'results', 'paper', 'fig_scene.png'), '-dpng', '-r220');
close(f);
