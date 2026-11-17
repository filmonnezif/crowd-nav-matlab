function [Y, info] = predict_cv(H, predLen, dt, opts)
%PREDICT_CV  Constant-velocity prediction.
%   Y = predict_cv(H, 12, 0.4) with H n x obsLen x 2 observed positions.
%   Velocity = mean displacement over the last opts.nAvg steps (default 1,
%   i.e. the last step). Returns Y n x predLen x 2.
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'nAvg'), opts.nAvg = 1; end
H = fill_history(H);
L = size(H, 2);
a = max(1, L - opts.nAvg);
v = (H(:, L, :) - H(:, a, :)) / ((L - a) * dt);   % n x 1 x 2
steps = reshape((1:predLen) * dt, 1, predLen);
Y = H(:, L, :) + v .* steps;
info.vel = reshape(v, [], 2);
end
