% DEMO_PHASE1  Clean data + robot crossing a replayed crowd.
%
%   >> demo_phase1
%
% 1. Rebuilds train/val/test from the raw ETH/UCY files with no
%    train/val leak and prints the counts.
% 2. Runs the Phase 1 robot (A* around people's CURRENT positions,
%    pure pursuit, speed governor) through the busiest 30 s of a scene.
% 3. Animates it and saves results/phase1_<scene>.mp4.
%
% Change `sceneName` to any file in data/raw (without .txt).
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
sceneName = 'crowds_zara01';
saveVideo = true;

% ---- 1. data ----------------------------------------------------------
D = build_dataset(fullfile(root, 'data', 'raw'));
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
save(fullfile(root, 'results', 'dataset_clean.mat'), '-struct', 'D', ...
    'train', 'val', 'test', 'report');

% ---- 2. one episode ---------------------------------------------------
S = D.scenes(strcmp({D.scenes.name}, sceneName));
cfg = sim_config();
task = scene_task(S, scene_busy_time(S, 30));
[log, m] = run_episode(S, task, cfg);
fprintf('Reached goal: %d | collisions: %d | min clearance: %.2f m | time: %.1f s | path ratio: %.2f | stopped %.0f%% of the time\n', ...
    m.reached, m.collisions, m.minClear, m.timeToGoal, m.pathRatio, 100 * m.freezeFrac);

% ---- 3. animation -----------------------------------------------------
if saveVideo
    animate_episode(log, fullfile(root, 'results', ['phase1_' sceneName '.mp4']));
else
    animate_episode(log);
end
