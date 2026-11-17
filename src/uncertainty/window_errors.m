function [err, speed] = window_errors(R, W)
%WINDOW_ERRORS  Per-horizon errors (N x 12) of an eval_predictor result R
%   on windows W, and each window's last observed speed (m/s).
d = R.Y - W.pred;
err = sqrt(d(:, :, 1).^2 + d(:, :, 2).^2);
v = W.obs(:, end, :) - W.obs(:, end - 1, :);
speed = sqrt(v(:, 1, 1).^2 + v(:, 1, 2).^2) / 0.4;
end
