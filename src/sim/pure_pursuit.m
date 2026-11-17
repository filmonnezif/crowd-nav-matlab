function [v, w, target] = pure_pursuit(x, path, lookahead, vDes, wmax)
%PURE_PURSUIT  Follow a polyline path with a differential-drive robot.
%   x = [px py theta], path = K x 2 world points.
%   Chooses the first path point at least `lookahead` meters ahead of the
%   closest point, then steers along the circular arc through it:
%   curvature = 2*sin(alpha)/Ld, w = v*curvature. If the point is behind
%   the robot it turns in place first.
if isempty(path)
    v = 0; w = 0; target = x(1:2); return
end
d = hypot(path(:, 1) - x(1), path(:, 2) - x(2));
[~, iClose] = min(d);
ahead = iClose - 1 + find(d(iClose:end) >= lookahead, 1);
if isempty(ahead), ahead = size(path, 1); end
target = path(ahead, :);
dx = target(1) - x(1); dy = target(2) - x(2);
alpha = atan2(dy, dx) - x(3);
alpha = atan2(sin(alpha), cos(alpha));
Ld = max(hypot(dx, dy), 1e-6);
if abs(alpha) > pi / 2
    v = 0;
    w = sign(alpha) * wmax;
    return
end
v = vDes;
w = v * 2 * sin(alpha) / Ld;
if abs(w) > wmax          % keep the arc: slow down instead of cutting it
    v = v * wmax / abs(w);
    w = sign(w) * wmax;
end
end
