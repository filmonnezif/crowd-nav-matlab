% TRAIN_GP  Gaussian process regression predictor (Statistics and Machine
% Learning Toolbox). One GP per output (12 horizons x [x y] = 24), each
% with an ARD squared-exponential kernel on the 7 heading-frame step
% displacements. Exact GPs scale as N^3, so it uses a random subset of
% nTrain training windows (default 1000, ~5 min). Fitted GPs are
% saved after each one, so a disconnected MATLAB Online session resumes.
%
%   >> train_gp
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
run(fullfile(root, 'startup.m'));
if ~exist('nTrain', 'var'), nTrain = 1000; end
featOpts = struct('cell', 0.5, 'G', 16, 'predLen', 12);
D = build_dataset(fullfile(root, 'data', 'raw'), 'Verbose', false);
[F, T] = build_training_set(D.train, D.scenes, featOpts);
rng(1);
pick = randperm(size(T, 1), min(nTrain, size(T, 1)));
X = [F.motion(pick, :, 1), F.motion(pick, :, 2)];
T = T(pick, :);
models = cell(1, 24);
partFile = fullfile(root, 'models', sprintf('gp_partial_%d.mat', nTrain));
if exist(partFile, 'file'), load(partFile, 'models'); end
tic;
for j = 1:24
    if ~isempty(models{j}), continue; end
    models{j} = compact(fitrgp(X, T(:, j), 'KernelFunction', 'ardsquaredexponential', ...
        'Standardize', true, 'FitMethod', 'exact', 'PredictMethod', 'exact'));
    fprintf('GP %2d/24 fitted (%.0f s)\n', j, toc);
    save(partFile, 'models');
end
save(fullfile(root, 'models', 'gp.mat'), 'models', 'featOpts', 'nTrain');
clear predict_gp
R = eval_predictor(make_predictor('gp'), D.test, D.scenes);
Rv = eval_predictor(make_predictor('gp'), D.val, D.scenes);
fprintf('GP: val ADE/FDE %.3f / %.3f, ETH test %.3f / %.3f\n', Rv.ADE, Rv.FDE, R.ADE, R.FDE);
