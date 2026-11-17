function [Y, info] = predict_kalman(H, predLen, dt, opts)
%PREDICT_KALMAN  Filter each track with KALMAN_CV, then roll the
%   constant-velocity model forward. Also returns the predicted position
%   covariance per step (info.Sigma n x predLen x 2 x 2), which Phase 4
%   compares with conformal radii.
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'q'), opts.q = 1.0; end   % tuned on val (tune_phase2)
if ~isfield(opts, 'r'), opts.r = 0.05; end
n = size(H, 1);
Y = zeros(n, predLen, 2);
info.Sigma = zeros(n, predLen, 2, 2);
info.vel = zeros(n, 2);
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
G = [dt^2 / 2 0; 0 dt^2 / 2; dt 0; 0 dt];
Q = opts.q^2 * (G * G');
for i = 1:n
    Z = reshape(H(i, :, :), [], 2);
    [x, P] = kalman_cv(Z, dt, opts.q, opts.r);
    info.vel(i, :) = x(3:4)';
    for h = 1:predLen
        x = F * x;
        P = F * P * F' + Q;
        Y(i, h, :) = reshape(x(1:2), 1, 1, 2);
        info.Sigma(i, h, :, :) = reshape(P(1:2, 1:2), 1, 1, 2, 2);
    end
end
end
