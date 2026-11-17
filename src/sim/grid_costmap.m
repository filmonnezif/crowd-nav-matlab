function [blocked, cost] = grid_costmap(G, centers, radii, socialR)
%GRID_COSTMAP  Hard and soft costs from circular obstacles.
%   blocked(i) is true inside any circle (radius = pedestrian + robot +
%   safety margin). cost(i) in [0,1] rises linearly from 0 at socialR
%   beyond the circle edge to 1 at the edge, so A* prefers to keep
%   distance from people instead of grazing past them.
blocked = false(G.ny, G.nx);
cost = zeros(G.ny, G.nx);
for i = 1:size(centers, 1)
    reach = radii(i) + socialR;
    c1 = max(1, floor((centers(i, 1) - reach - G.x0) / G.res) + 1);
    c2 = min(G.nx, floor((centers(i, 1) + reach - G.x0) / G.res) + 1);
    r1 = max(1, floor((centers(i, 2) - reach - G.y0) / G.res) + 1);
    r2 = min(G.ny, floor((centers(i, 2) + reach - G.y0) / G.res) + 1);
    if c1 > c2 || r1 > r2, continue; end
    d = hypot(G.X(r1:r2, c1:c2) - centers(i, 1), G.Y(r1:r2, c1:c2) - centers(i, 2));
    blocked(r1:r2, c1:c2) = blocked(r1:r2, c1:c2) | d <= radii(i);
    s = min(max((reach - d) / socialR, 0), 1);
    cost(r1:r2, c1:c2) = max(cost(r1:r2, c1:c2), s);
end
end
