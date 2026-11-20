% DEMO_PHASE6  The presentation demo: one scene, every predictor.
%
%   >> demo_phase6
%
% The same busy stretch of the university scene as demo_phase5, with the
% reactive crowd and the best Phase 5 robot (space-time A* + sampling
% MPC). It is run once per predictor, and everything is drawn:
%   blue dots / dashed lines  people and their predicted paths (4.8 s)
%   orange rings              90% conformal uncertainty at 1.6 s ahead
%                             (drawn only; the planner uses the bare
%                             predictions, as in the best Phase 5 run)
%   green line                the robot's space-time plan
%   red line / disc           the robot's path so far (black disc = contact)
%
% Output (in results/):
%   phase6_compare.mp4 (.avi on Linux)  all predictors side by side
%   phase6_<pred>.png                   a still of each run at tStill
%   phase6_compare_<t>s.png             the side-by-side frame at tStill
%   phase6_summary.csv                  the metrics table printed below
% and an interactive viewer (view_episodes) with a predictor drop-down,
% a time slider and Play, for live use in the presentation.
%
% Learned predictors need models/<name>.mat from train_predictors (MATLAB
% with the Deep Learning Toolbox). Missing ones are skipped with a note,
% so the demo also runs in Octave with the classical predictors.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');

% ---- settings -----------------------------------------------------------
sceneName  = 'students003';
predNames  = {'cv', 'kalman', 'lstm', 'sgcnn'};   % any of cv kalman sf mlp lstm tcn sgcnn tf gp
predLabels = {'Constant velocity', 'Kalman', 'LSTM', 'Social-grid CNN'};
planner    = 'sta';         % 'astar' or 'sta'
controller = 'mpc';         % 'pp', 'dwa' or 'mpc'
crowdMode  = 'reactive';    % 'replay' or 'reactive'
showAlpha  = 0.1;           % draw 90% conformal rings; [] for none
rings      = 4;             % ring at step 4 = 1.6 s ahead; [4 8] adds 3.2 s
tStill     = 12;            % time (s) of the saved stills
saveVideo  = true;
videoEvery = 2;             % video frame every 2nd step (5 fps, real time).
                            % MATLAB Online needs ~3-4 s per frame: ~10 min.
openViewer = true;
% -------------------------------------------------------------------------

S = read_raw_scene(fullfile(root, 'data', 'raw', [sceneName '.txt']));
t0s = linspace(0, (S.nK - 1) * S.dt - 60 - 1, 10);   % the experiment's start times
task = scene_task(S, t0s(2), true);
outDir = fullfile(root, 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

keep = true(size(predNames));
for i = 1:numel(predNames)
    if any(strcmpi(predNames{i}, {'mlp', 'lstm', 'tcn', 'sgcnn', 'tf'})) && ...
            ~exist(fullfile(root, 'models', [predNames{i} '.mat']), 'file')
        fprintf('Skipping %s: no models/%s.mat (run train_predictors in MATLAB).\n', ...
            predNames{i}, predNames{i});
        keep(i) = false;
    end
end
predNames = predNames(keep); predLabels = predLabels(keep);

n = numel(predNames);
logs = cell(1, n);
M = cell(1, n);
fprintf('\n%s, %s crowd, %s + %s\n', sceneName, crowdMode, planner, controller);
fprintf('%-18s %7s %9s %9s %12s %8s %8s %9s\n', 'predictor', 'reached', 'contacts', ...
    'at fault', 'min clear m', 'time s', 'stopped', 'pushed m');
for i = 1:n
    cfg = sim_config('planner', planner, 'controller', controller, 'crowd', crowdMode);
    cfg.obstacleFcn = make_obstacle_fcn(predNames{i}, struct('showAlpha', showAlpha));
    [logs{i}, M{i}] = run_episode(S, task, cfg);
    m = M{i};
    fprintf('%-18s %7d %9d %9d %12.2f %8.1f %7.0f%% %9.2f\n', predLabels{i}, m.reached, ...
        m.collisions, m.faultCollisions, m.minClear, m.timeToGoal, 100 * m.freezeFrac, m.crowdDisp);
end

% summary table
fid = fopen(fullfile(outDir, 'phase6_summary.csv'), 'w');
fprintf(fid, 'predictor,reached,contacts,at_fault,min_clearance_m,time_s,stopped_frac,people_pushed_m\n');
for i = 1:n
    m = M{i};
    fprintf(fid, '%s,%d,%d,%d,%.3f,%.2f,%.3f,%.3f\n', predNames{i}, m.reached, m.collisions, ...
        m.faultCollisions, m.minClear, m.timeToGoal, m.freezeFrac, m.crowdDisp);
end
fclose(fid);

% one still per run, with the legend
for i = 1:n
    f = figure('Color', 'w', 'Position', [60 60 900 560], 'Visible', 'off');
    h = scene_view_init(axes('Parent', f), logs{i}, struct('legend', true, 'label', predLabels{i}, 'rings', rings));
    scene_view_draw(h, logs{i}, round(tStill / logs{i}.cfg.dt) + 1);
    print(f, fullfile(outDir, sprintf('phase6_%s.png', predNames{i})), '-dpng', '-r110');
    close(f);
end

if saveVideo
    animate_compare(logs, predLabels, fullfile(outDir, 'phase6_compare.mp4'), ...
        struct('rings', rings, 'every', videoEvery, 'pngAt', tStill, 'pngFile', fullfile(outDir, 'phase6_compare_%02ds.png')));
end
if openViewer && (~exist('OCTAVE_VERSION', 'builtin') || ~isempty(getenv('DISPLAY')))
    view_episodes(logs, predLabels, struct('rings', rings));
end
