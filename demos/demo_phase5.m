% DEMO_PHASE5  Reactive crowd + space-time planning.
%
%   >> demo_phase5
%
% A busy stretch of the university scene (episode 2, reversed, from the
% Phase 5 experiment), with a REACTIVE crowd:
% people follow their recorded routes but step around the robot (Social
% Force). Two robots are compared:
%   Phase 3 style: 2-D A* + DWA
%   Phase 5:       space-time A* (can plan to wait) + sampling MPC
% Both use constant-velocity prediction. This episode was picked because
% it shows the difference well (in Octave: 4 contacts / 43 s vs 1 / 23 s);
% averaged over 120 episodes the gap is smaller, see README Phase 5.
% MPC samples randomly, so MATLAB and Octave numbers differ a little. The green line is the plan; for
% space-time A* it is where the robot intends to be over the next 4.8 s.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
sceneName = 'students003';
runs = {'astar', 'dwa'; 'sta', 'mpc'};     % {planner, controller}
crowdMode = 'reactive';                    % or 'replay'
saveVideo = true;

S = read_raw_scene(fullfile(root, 'data', 'raw', [sceneName '.txt']));
t0s = linspace(0, (S.nK - 1) * S.dt - 60 - 1, 10);   % the experiment's start times
task = scene_task(S, t0s(2), true);
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
for i = 1:size(runs, 1)
    cfg = sim_config('planner', runs{i, 1}, 'controller', runs{i, 2}, 'crowd', crowdMode);
    cfg.obstacleFcn = make_obstacle_fcn('cv');
    [log, m] = run_episode(S, task, cfg);
    fprintf('%-5s+%-4s reached %d | contacts %d (at fault %d) | min clearance %5.2f m | time %4.1f s | stopped %2.0f%% | people pushed %.2f m\n', ...
        runs{i, 1}, runs{i, 2}, m.reached, m.collisions, m.faultCollisions, ...
        m.minClear, m.timeToGoal, 100 * m.freezeFrac, m.crowdDisp);
    if saveVideo
        animate_episode(log, fullfile(root, 'results', ...
            sprintf('phase5_%s_%s_%s_%s.mp4', sceneName, crowdMode, runs{i, 1}, runs{i, 2})));
    else
        animate_episode(log);
    end
end
