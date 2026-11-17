function T = exp_navigation(predName, nStarts, outFile, controller)
%EXP_NAVIGATION  Same episodes for any predictor -> one CSV row each.
%   exp_navigation('kalman')            % 10 start times x 2 directions x 6 scenes
%   exp_navigation('none', 5, 'results/nav_none.csv')
%   exp_navigation('kalman', 10, '', 'dwa')   % DWA local controller
%   Start times are spread evenly over each recording, so every
%   predictor faces exactly the same crowds.
if nargin < 2, nStarts = 10; end
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
if nargin < 4, controller = 'pp'; end
if nargin < 3 || isempty(outFile)
    outFile = fullfile(root, 'results', ['nav_' predName '_' controller '.csv']);
end
run(fullfile(root, 'startup.m'));
files = dir(fullfile(root, 'data', 'raw', '*.txt'));
cfg = sim_config('controller', controller);
cfg.obstacleFcn = make_obstacle_fcn(predName);
rows = {};
for f = 1:numel(files)
    S = read_raw_scene(fullfile(root, 'data', 'raw', files(f).name));
    Tlen = (S.nK - 1) * S.dt;
    for t0 = linspace(0, Tlen - cfg.Tmax - 1, nStarts)
        for flip = [false true]
            [~, m] = run_episode(S, scene_task(S, t0, flip), cfg);
            rows(end + 1, :) = {[predName '+' controller], m.scene, m.t0, flip, m.reached, ...
                m.success, m.collisions, m.minClear, m.timeToGoal, ...
                m.pathRatio, m.freezeFrac, m.intrusions, m.meanPlanMs, ...
                m.faultCollisions, m.successNoFault}; %#ok<AGROW>
        end
    end
    fprintf('%s: %s done\n', predName, S.name);
end
d = fileparts(outFile);
if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
fid = fopen(outFile, 'w');
fprintf(fid, 'predictor,scene,t0,flip,reached,success,collisions,minClear,timeToGoal,pathRatio,freezeFrac,intrusions,planMs,faultCollisions,successNoFault\n');
for r = 1:size(rows, 1)
    fprintf(fid, '%s,%s,%.1f,%d,%d,%d,%d,%.3f,%.2f,%.3f,%.3f,%.1f,%.1f,%d,%d\n', rows{r, :});
end
fclose(fid);
T = rows;
fprintf('%s+%s: success %.0f%%, collisions/ep %.2f, at-fault/ep %.2f, success (no at-fault) %.0f%%, reached %.0f%%\n', ...
    predName, controller, 100 * mean(cell2mat(rows(:, 6))), mean(cell2mat(rows(:, 7))), ...
    mean(cell2mat(rows(:, 14))), 100 * mean(cell2mat(rows(:, 15))), 100 * mean(cell2mat(rows(:, 5))));
end
