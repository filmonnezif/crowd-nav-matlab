function [path, info] = astar_grid(blocked, cost, s, g, costWeight)
%ASTAR_GRID  A* search on an 8-connected grid.
%   path = astar_grid(blocked, cost, [r c], [r c]) returns the cells of a
%   shortest path as a K x 2 [row col] list (empty if none).
%   Step cost = length * (1 + costWeight * mean cost of the two cells).
%   The heuristic is the octile distance, which never overestimates,
%   so the path is optimal for this cost. Diagonal moves may not cut a
%   blocked corner. The start and goal cells are always treated as free.
if nargin < 5, costWeight = 3; end
[ny, nx] = size(blocked);
N = ny * nx;
si = sub2ind([ny nx], s(1), s(2));
gi = sub2ind([ny nx], g(1), g(2));
blocked(si) = false;
blocked(gi) = false;

dr = [-1 -1 -1 0 0 1 1 1];
dc = [-1 0 1 -1 1 -1 0 1];
dl = hypot(dr, dc);

[R, C] = ndgrid(1:ny, 1:nx);
ar = abs(R(:) - g(1)); ac = abs(C(:) - g(2));
H = max(ar, ac) + (sqrt(2) - 1) * min(ar, ac);   % octile distance

G = inf(N, 1);
F = inf(N, 1);          % f = g + h for open cells, inf otherwise
parent = zeros(N, 1);
closed = false(N, 1);
G(si) = 0; F(si) = H(si);
expanded = 0;
while true
    [fmin, i] = min(F);
    if isinf(fmin), break; end
    if i == gi, break; end
    F(i) = inf;
    closed(i) = true;
    expanded = expanded + 1;
    r = R(i); c = C(i);
    for k = 1:8
        nr = r + dr(k); nc = c + dc(k);
        if nr < 1 || nr > ny || nc < 1 || nc > nx, continue; end
        j = nr + (nc - 1) * ny;
        if blocked(j) || closed(j), continue; end
        if dr(k) ~= 0 && dc(k) ~= 0 && ...
                (blocked(r + (nc - 1) * ny) || blocked(nr + (c - 1) * ny))
            continue
        end
        ng = G(i) + dl(k) * (1 + costWeight * 0.5 * (cost(i) + cost(j)));
        if ng < G(j)
            G(j) = ng;
            parent(j) = i;
            F(j) = ng + H(j);
        end
    end
end
info.expanded = expanded;
info.cost = G(gi);
if isinf(G(gi))
    path = zeros(0, 2);
    return
end
idx = gi;
while idx(end) ~= si
    idx(end + 1) = parent(idx(end)); %#ok<AGROW>
end
idx = flipud(idx(:));
path = [R(idx), C(idx)];
end
