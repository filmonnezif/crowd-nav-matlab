% EXP_PHASE1_BASELINE  Reactive baseline over many start times and scenes.
% For each scene: 10 start times spread over the recording, both
% directions -> 20 episodes. Writes results/phase1_baseline.csv.
% This is the "no prediction" row of the paper's results table.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
files = dir(fullfile(root, 'data', 'raw', '*.txt'));
cfg = sim_config();
rows = {};
for f = 1:numel(files)
    S = read_raw_scene(fullfile(root, 'data', 'raw', files(f).name));
    Tlen = (S.nK - 1) * S.dt;
    starts = linspace(0, Tlen - cfg.Tmax - 1, 10);
    for t0 = starts
        for flip = [false true]
            [~, m] = run_episode(S, scene_task(S, t0, flip), cfg);
            rows(end + 1, :) = {m.scene, m.t0, flip, m.reached, m.success, ...
                m.collisions, m.minClear, m.timeToGoal, m.pathRatio, ...
                m.freezeFrac, m.intrusions}; %#ok<SAGROW>
        end
    end
    fprintf('%s done\n', S.name);
end
names = {'scene', 't0', 'flip', 'reached', 'success', 'collisions', ...
    'minClear', 'timeToGoal', 'pathRatio', 'freezeFrac', 'intrusions'};
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
fid = fopen(fullfile(root, 'results', 'phase1_baseline.csv'), 'w');
fprintf(fid, '%s,', names{1:end - 1}); fprintf(fid, '%s\n', names{end});
for r = 1:size(rows, 1)
    fprintf(fid, '%s,%.1f,%d,%d,%d,%d,%.3f,%.2f,%.3f,%.3f,%.1f\n', rows{r, :});
end
fclose(fid);
% summary per scene
sc = rows(:, 1); u = unique(sc);
fprintf('\n%-14s %8s %8s %10s %9s %8s\n', 'scene', 'success', 'reached', 'coll/ep', 'minClr', 'time');
for i = 1:numel(u)
    m = strcmp(sc, u{i});
    tg = cell2mat(rows(m, 8));
    mc = cell2mat(rows(m, 7)); mc = mc(isfinite(mc));
    fprintf('%-14s %7.0f%% %7.0f%% %10.2f %9.2f %8.1f\n', u{i}, ...
        100 * mean(cell2mat(rows(m, 5))), 100 * mean(cell2mat(rows(m, 4))), ...
        mean(cell2mat(rows(m, 6))), mean(mc), mean(tg(~isnan(tg))));
end
