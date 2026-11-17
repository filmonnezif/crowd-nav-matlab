% DEMO_PHASE4  Prediction uncertainty: conformal rings around predictions.
%
%   >> demo_phase4
%
% Runs the robot (CV prediction + DWA) through the busiest 30 s of the
% ETH scene twice: without uncertainty, and with speed-normalised split
% conformal radii (90% target) inflating every predicted person. The
% animation draws the 1.6 / 3.2 / 4.8 s conformal rings in orange.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
sceneName = 'biwi_eth';
predName = 'cv';
runs = {'none', struct(); ...
        'cp90', struct('alpha', 0.1, 'cpMode', 'speed')};
saveVideo = true;

S = read_raw_scene(fullfile(root, 'data', 'raw', [sceneName '.txt']));
task = scene_task(S, scene_busy_time(S, 30));
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
for i = 1:size(runs, 1)
    cfg = sim_config('controller', 'dwa');
    cfg.obstacleFcn = make_obstacle_fcn(predName, runs{i, 2});
    [log, m] = run_episode(S, task, cfg);
    fprintf('%-5s reached %d | contacts %d (at fault %d) | min clearance %5.2f m | time %4.1f s | stopped %2.0f%%\n', ...
        runs{i, 1}, m.reached, m.collisions, m.faultCollisions, ...
        m.minClear, m.timeToGoal, 100 * m.freezeFrac);
    if saveVideo
        animate_episode(log, fullfile(root, 'results', ...
            sprintf('phase4_%s_%s_%s.mp4', sceneName, predName, runs{i, 1})));
    else
        animate_episode(log);
    end
end
