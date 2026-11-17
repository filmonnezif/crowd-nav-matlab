% DEMO_PHASE2  Same crowd, with and without prediction.
%
%   >> demo_phase2
%
% Runs the robot through the busiest 30 s of a scene twice: first
% reacting only to where people are now (Phase 1), then planning around
% where the Kalman filter predicts they will be. Predicted paths are
% drawn as dashed blue lines. Saves results/phase2_<scene>_<predictor>.avi/mp4.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
sceneName = 'crowds_zara01';
predictors = {'none', 'kalman'};     % try 'cv' or 'sf' too
saveVideo = true;

S = read_raw_scene(fullfile(root, 'data', 'raw', [sceneName '.txt']));
task = scene_task(S, scene_busy_time(S, 30));
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
for i = 1:numel(predictors)
    cfg = sim_config();
    cfg.obstacleFcn = make_obstacle_fcn(predictors{i});
    [log, m] = run_episode(S, task, cfg);
    fprintf('%-7s reached %d | collisions %d | min clearance %5.2f m | time %4.1f s | stopped %2.0f%%\n', ...
        predictors{i}, m.reached, m.collisions, m.minClear, m.timeToGoal, 100 * m.freezeFrac);
    if saveVideo
        animate_episode(log, fullfile(root, 'results', ...
            sprintf('phase2_%s_%s.mp4', sceneName, predictors{i})));
    else
        animate_episode(log);
    end
end
