function run_phase7(worker, nWorkers, jobs)
%RUN_PHASE7  Run the Phase 7 grid (phase7_jobs), split over workers.
%   run_phase7(1, 4)   % this process runs jobs 1, 5, 9, ...
%   run_phase7(1, 1, [3 7])  % only jobs 3 and 7
%   Finished jobs (existing CSV) are skipped, so it can be restarted.
%   Learned predictors use models/<net>.mat with the Deep Learning
%   Toolbox, or the exported models/w_<net>.mat (net_forward) without it.
if nargin < 1, worker = 1; end
if nargin < 2, nWorkers = 1; end
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
run(fullfile(root, 'startup.m'));
J = phase7_jobs();
if nargin < 3, jobs = worker:nWorkers:size(J, 1); end
outDir = fullfile(root, 'results', 'phase7');
if ~exist(outDir, 'dir'), mkdir(outDir); end
for j = jobs
    f = fullfile(outDir, [J{j, 1} '.csv']);
    if exist(f, 'file'), continue; end
    t = tic;
    exp_navigation(J{j, 2}, 10, [f '.part'], J{j, 3}, J{j, 4}, J{j, 1}, J{j, 5});
    movefile([f '.part'], f);
    fprintf('[%s] job %d %s done in %.0f min\n', datestr(now, 'HH:MM'), j, J{j, 1}, toc(t) / 60);
end
end
