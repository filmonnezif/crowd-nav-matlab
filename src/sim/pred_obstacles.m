function [C, r, viz] = pred_obstacles(S, t, ids, P, V, cfg, x, fn, popt)
%PRED_OBSTACLES  Turn pedestrian predictions into planner obstacles.
%   Used through cfg.obstacleFcn (see make_obstacle_fcn).
%
%   Everyone's current position is a hard obstacle (as in Phase 1).
%   Each predicted position p_h, tau_h seconds ahead, is
%     - a HARD obstacle if the robot could be there at about that time,
%       i.e. | dist(robot, p_h) - vmax * tau_h | < popt.band.
%       This is a cheap stand-in for space-time planning (Phase 5): it
%       blocks the places where the robot would meet the person, but
%       not places the person will have left before the robot arrives.
%     - otherwise a SOFT cost only (keep some distance from where
%       people are heading).
%   Only predictions up to popt.horizon seconds ahead are used.
hardR = cfg.pedR + cfg.robotR + cfg.margin;
C = P;
r = repmat(hardR, size(P, 1), 1);
viz.ids = zeros(0, 1);
viz.Y = zeros(0, 12, 2);
viz.P0 = zeros(0, 2);
viz.tObs = t;
k = floor(t / S.dt + 1e-9) + 1;
[hid, H] = scene_history(S, k, popt.obsLen);
if isempty(hid), return; end
Y = fn(H, popt.predLen, S.dt);
viz.ids = hid;
viz.Y = Y;
viz.P0 = reshape(fill_history(H(:, end, :)), [], 2);   % positions at tObs
viz.tObs = (k - 1) * S.dt;                             % scene time of the last frame
tau = (k - 1) * S.dt + (1:popt.predLen) * S.dt - t;   % seconds ahead
use = tau <= popt.horizon;
Yx = Y(:, use, 1); Yy = Y(:, use, 2);
T = repmat(tau(use), size(Y, 1), 1);
d = hypot(Yx - x(1), Yy - x(2));
hard = abs(d - cfg.vmax * T) < popt.band;
C = [C; Yx(:), Yy(:)];
r = [r; hard(:) * hardR];
end
