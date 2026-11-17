% EXP_PHASE3_NAVIGATION  Navigation with the trained networks (MATLAB only).
% Same 120 episodes as Phases 1-2, with pure pursuit ('pp') and the
% DWA controller ('dwa'). Writes results/nav_<net>_<controller>.csv and
% prints a summary. Needs models/*.mat from train_predictors.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
if ~exist('nets', 'var'), nets = {'lstm', 'tcn', 'sgcnn', 'tf', 'mlp'}; end
if ~exist('controllers', 'var'), controllers = {'dwa', 'pp'}; end
summary = {};
for c = 1:numel(controllers)
    for i = 1:numel(nets)
        tic;
        T = exp_navigation(nets{i}, 10, '', controllers{c});
        coll = cell2mat(T(:, 7));
        summary(end + 1, :) = {nets{i}, controllers{c}, 100 * mean(cell2mat(T(:, 6))), ...
            mean(coll), 100 * mean(cell2mat(T(:, 5))), toc}; %#ok<SAGROW>
    end
end
fprintf('\n%-8s %-5s %8s %10s %8s %8s\n', 'net', 'ctrl', 'success', 'coll/ep', 'reached', 'sec');
for r = 1:size(summary, 1)
    fprintf('%-8s %-5s %7.0f%% %10.2f %7.0f%% %8.0f\n', summary{r, :});
end
