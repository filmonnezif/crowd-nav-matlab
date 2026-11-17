function C = conformal_calibrate(err, speed, alpha, mode, c)
%CONFORMAL_CALIBRATE  Split conformal prediction radii, one per horizon.
%   C = conformal_calibrate(err, speed, 0.1, 'plain')
%   err    N x 12 prediction errors (m) of ONE predictor on a calibration
%          set it was not trained on (here: the clean val split)
%   speed  N x 1 last observed walking speed (m/s)
%   alpha  miscoverage, 0.1 -> 90% of true future positions should fall
%          inside the circle of radius r_h around the prediction
%   mode   'plain':  score = error,               radius = q_h
%          'speed':  score = error / (c + speed), radius = q_h * (c + speed)
%                    (fast walkers get bigger circles than people standing)
%
%   q_h is the ceil((N+1)(1-alpha))-th smallest score at horizon h. If
%   the test data are exchangeable with the calibration data, this gives
%   P(error_h <= r_h) >= 1 - alpha for every horizon h, whatever the
%   predictor is: a guarantee that needs no model of the error.
if nargin < 4, mode = 'plain'; end
if nargin < 5, c = 0.5; end
ok = all(isfinite(err), 2);
err = err(ok, :); speed = speed(ok);
switch mode
    case 'plain', s = err;
    case 'speed', s = err ./ (c + speed);
    otherwise, error('conformal_calibrate:mode', 'Unknown mode %s', mode);
end
n = size(s, 1);
k = min(n, ceil((n + 1) * (1 - alpha)));
s = sort(s, 1);
C.q = s(k, :);
C.scores = s;       % sorted calibration scores, for adaptive conformal
C.alpha = alpha;
C.mode = mode;
C.c = c;
C.n = n;
end
