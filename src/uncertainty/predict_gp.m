function [Y, info] = predict_gp(H, predLen, dt)
%PREDICT_GP  Gaussian process regression predictor (models/gp.mat, made
%   by train_gp). Same inputs/outputs as the networks: the 7 step
%   displacements in the heading frame -> correction to constant
%   velocity. Unlike the networks, a GP also says how unsure it is:
%   info.sigma (n x 12) is the predictive standard deviation (m) of the
%   position at each horizon.
persistent M
if isempty(M)
    f = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'models', 'gp.mat');
    if ~exist(f, 'file'), error('predict_gp:nomodel', 'Run train_gp first.'); end
    M = load(f);
end
n = size(H, 1);
info.sigma = zeros(n, predLen);
if n == 0
    Y = zeros(0, predLen, 2); return
end
F = traj_features(H, M.featOpts);
X = [F.motion(:, :, 1), F.motion(:, :, 2)];
res = zeros(n, 2 * predLen);
sd = zeros(n, 2 * predLen);
for j = 1:2 * predLen
    [res(:, j), sd(:, j)] = predict(M.models{j}, X);
end
Y = traj_decode(res, F);
info.sigma = sqrt((sd(:, 1:predLen).^2 + sd(:, predLen + 1:end).^2) / 2);
short = any(isnan(H(:, :, 1)), 2);
if any(short)
    Y(short, :, :) = predict_cv(H(short, :, :), predLen, dt);
end
end
