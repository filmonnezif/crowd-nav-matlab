function [v, w] = time_pursuit(x, path, tNow, cfg)
%TIME_PURSUIT  Follow a space-time plan in time: aim at where the plan
%   wants the robot cfg.lookahead / vmax seconds from now. When the plan
%   waits, the target stops moving and so does the robot.
T = max(cfg.lookahead / cfg.vmax, cfg.stDt);
r = ref_at(path, x, tNow, T, cfg.vmax);
dx = r(1) - x(1); dy = r(2) - x(2);
d = hypot(dx, dy);
if d < 0.05
    v = 0; w = 0; return
end
err = atan2(sin(atan2(dy, dx) - x(3)), cos(atan2(dy, dx) - x(3)));
v = min(cfg.vmax, d / T) * max(cos(err), 0);
w = min(max(2.5 * err, -cfg.wmax), cfg.wmax);
end
