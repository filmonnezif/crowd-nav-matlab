function [Y, info] = predict_social_force(H, predLen, dt, opts)
%PREDICT_SOCIAL_FORCE  Helbing & Molnar (1995) social force rollout.
%   Everyone in H is simulated together. Each person keeps a desired
%   velocity (their Kalman-filtered velocity) and is pushed away from
%   others:
%       dv/dt = (v0 - v)/tau + sum_j A exp((2R - d_ij)/B) n_ij
%   where d_ij is the distance and n_ij the unit vector from j to i.
%   Only people within opts.range and roughly in front (field of view
%   weight lambda) push strongly.
if nargin < 4, opts = struct(); end
d = struct('tau', 0.5, 'A', 0.25, 'B', 0.2, 'R', 0.3, 'lambda', 0.35, ... % A, B tuned on val
    'range', 3, 'substeps', 4, 'q', 1.0, 'r', 0.05, 'vmax', 2.5);
f = fieldnames(d);
for k = 1:numel(f)
    if ~isfield(opts, f{k}), opts.(f{k}) = d.(f{k}); end
end
H = fill_history(H);
n = size(H, 1);
if n == 0
    Y = zeros(0, predLen, 2); info.vel = zeros(0, 2); return
end
[~, kinfo] = predict_kalman(H, 1, dt, opts);
v0 = kinfo.vel;
p = reshape(H(:, end, :), n, 2);
v = v0;
Y = zeros(n, predLen, 2);
h = dt / opts.substeps;
for s = 1:predLen
    for ss = 1:opts.substeps
        a = (v0 - v) / opts.tau;
        if n > 1
            dx = p(:, 1) - p(:, 1)';   % dx(i,j) = x_i - x_j
            dy = p(:, 2) - p(:, 2)';
            dist = hypot(dx, dy) + eye(n) * 1e6;
            mag = opts.A * exp((2 * opts.R - dist) / opts.B);
            mag(dist > opts.range) = 0;
            % anisotropy: people ahead of i matter more than people behind
            sp = max(hypot(v(:, 1), v(:, 2)), 1e-6);
            cosphi = -(v(:, 1) .* dx + v(:, 2) .* dy) ./ (sp .* dist);
            w = opts.lambda + (1 - opts.lambda) * (1 + cosphi) / 2;
            mag = mag .* w;
            a(:, 1) = a(:, 1) + sum(mag .* dx ./ dist, 2);
            a(:, 2) = a(:, 2) + sum(mag .* dy ./ dist, 2);
        end
        v = v + h * a;
        sp = hypot(v(:, 1), v(:, 2));
        too = sp > opts.vmax;
        if any(too)
            v(too, :) = v(too, :) .* (opts.vmax ./ sp(too));
        end
        p = p + h * v;
    end
    Y(:, s, 1) = p(:, 1);
    Y(:, s, 2) = p(:, 2);
end
info.vel = v0;
end
