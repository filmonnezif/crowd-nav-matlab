function task = scene_task(S, t0, flip)
%SCENE_TASK  A start/goal pair across the long axis of a scene.
%   task = scene_task(S, t0) puts the start 1 m inside one end of the
%   scene and the goal 1 m inside the other, at mid-height, so the robot
%   has to cross the main pedestrian flow. flip = true swaps the ends.
if nargin < 3, flip = false; end
lo = prctile_(S.xy, 2); hi = prctile_(S.xy, 98);
mid = (lo + hi) / 2;
[~, ax] = max(hi - lo);
a = mid; b = mid;
a(ax) = lo(ax) + 1; b(ax) = hi(ax) - 1;
if flip, [a, b] = deal(b, a); end
task.start = [a, atan2(b(2) - a(2), b(1) - a(1))];
task.goal = b;
task.t0 = t0;
task.bounds = [lo(1) - 1, hi(1) + 1, lo(2) - 1, hi(2) + 1];
end

function p = prctile_(X, q)
% Percentile without the Statistics Toolbox.
X = sort(X, 1);
i = max(1, min(size(X, 1), round(q / 100 * size(X, 1))));
p = X(i, :);
end
