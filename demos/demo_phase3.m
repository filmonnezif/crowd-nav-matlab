% DEMO_PHASE3  A trained network + the DWA controller in a dense crowd.
%
%   >> demo_phase3                 % needs models/lstm.mat (train_predictors)
%
% Runs the robot through the busiest 30 s of the university scene twice:
% Phase 2 style (Kalman prediction + pure pursuit) and Phase 3 (LSTM
% prediction + DWA), then animates both and saves videos to results/.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
sceneName = 'students003';
runs = {'kalman', 'pp'; 'lstm', 'dwa'};    % {predictor, controller}
saveVideo = true;

S = read_raw_scene(fullfile(root, 'data', 'raw', [sceneName '.txt']));
task = scene_task(S, scene_busy_time(S, 30));
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
for i = 1:size(runs, 1)
    cfg = sim_config('controller', runs{i, 2});
    cfg.obstacleFcn = make_obstacle_fcn(runs{i, 1});
    [log, m] = run_episode(S, task, cfg);
    fprintf('%-7s+%-4s reached %d | contacts %d (at fault %d) | min clearance %5.2f m | time %4.1f s | stopped %2.0f%%\n', ...
        runs{i, 1}, runs{i, 2}, m.reached, m.collisions, m.faultCollisions, ...
        m.minClear, m.timeToGoal, 100 * m.freezeFrac);
    if saveVideo
        animate_episode(log, fullfile(root, 'results', ...
            sprintf('phase3_%s_%s_%s.mp4', sceneName, runs{i, 1}, runs{i, 2})));
    else
        animate_episode(log);
    end
end
