function [path, info] = st_plan(x, goal, pred, P, tNow, bounds, cfg)
%ST_PLAN  Space-time A* over predicted people (time is the third axis).
%   path = st_plan(x, goal, pred, P, tNow, bounds, cfg) returns an
%   (L+2) x 3 matrix [x y t]: where the robot should be at each time
%   step over the next cfg.stLayers * cfg.stDt seconds, then the goal.
%
%   The search space is a grid (cfg.stRes) copied once per time step
%   (cfg.stDt). From each cell the robot can stay (WAIT) or move to one
%   of the 8 neighbours, which always takes one time step. A cell is
%   blocked at a step if the robot would touch a person's predicted
%   position there (plus their uncertainty radius, Phase 4) at that time
%   or the step before. Because every move goes one step forward in
%   time, the graph has no cycles, so the best plan is found exactly by
%   dynamic programming backwards from the last layer (the same answer
%   as A* with a consistent heuristic, but vectorised over the grid).
%
%   Cost = time used + cfg.costWeight * social cost along the way
%        + (distance to goal / vmax) after the last layer.
%   Blocked cells cost cfg.stBlocked instead of infinity, so a plan
%   always exists; info.feasible says whether it is collision-free.
persistent GC
key = [bounds cfg.stRes];
if isempty(GC) || ~isequal(GC.key, key)
    GC.key = key;
    GC.G = grid_make(bounds, cfg.stRes);
end
G = GC.G;
L = cfg.stLayers;
tau = (0:L) * cfg.stDt;
[Q, Rad] = people_at(pred, P, tNow, tau);         % n x (L+1) x 2
ny = G.ny; nx = G.nx; nc = ny * nx;
hardR = cfg.robotR + cfg.pedR + cfg.margin;
blocked = false(ny, nx, L + 1);
soc = zeros(ny, nx, L + 1);
for l = 1:L + 1
    if isempty(Q), break; end
    dmin = inf(ny, nx);
    for i = 1:size(Q, 1)
        d = hypot(G.X - Q(i, l, 1), G.Y - Q(i, l, 2)) - Rad(i, l) - hardR;
        dmin = min(dmin, d);
    end
    blocked(:, :, l) = dmin < 0;
    soc(:, :, l) = max(0, 1 - max(dmin, 0) / cfg.socialR);
end
% entering a cell at step l needs it free at steps l-1 and l
blk = blocked;
blk(:, :, 2:end) = blocked(:, :, 2:end) | blocked(:, :, 1:end - 1);

ctg = hypot(G.X - goal(1), G.Y - goal(2)) / cfg.vmax;
atGoal = ctg * cfg.vmax < max(cfg.goalTol, cfg.stRes);
moves = [0 0; 1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];   % [dr dc]
mlen = hypot(moves(:, 1), moves(:, 2)) * cfg.stRes;
J = ctg;                                           % layer L+1 (index L+1)
pol = zeros(ny, nx, L, 'uint8');
for l = L:-1:1
    best = inf(ny, nx); arg = ones(ny, nx, 'uint8');
    nb = blk(:, :, l + 1); ns = soc(:, :, l + 1);
    for m = 1:size(moves, 1)
        cost = cfg.stDt + cfg.costWeight * cfg.stDt * ns + 0.05 * mlen(m) ...
            + cfg.stBlocked * nb + J;
        c = shift_in(cost, moves(m, 1), moves(m, 2));   % value at the neighbour
        better = c < best;
        best(better) = c(better);
        arg(better) = m;
    end
    best(atGoal) = 0;                              % arrived: no more cost
    J = best;
    pol(:, :, l) = arg;
end
% roll the policy forward from the robot's cell
[r, c] = cell_of(G, x(1:2));
pts = zeros(L + 1, 2);
pts(1, :) = x(1:2);
info.feasible = true;
for l = 1:L
    m = pol(r, c, l);
    r = r + moves(m, 1); c = c + moves(m, 2);
    pts(l + 1, :) = [G.X(r, c), G.Y(r, c)];
    if blk(r, c, l + 1), info.feasible = false; end
    if atGoal(r, c)
        pts(l + 2:end, :) = repmat(goal(:)', L - l, 1);
        break
    end
end
path = [pts, tNow + tau(:); goal(:)', tNow + tau(end) + ctg(r, c)];
info.cost = J(cell_of_idx(G, x(1:2)));
info.waits = nnz(all(abs(diff(pts)) < 1e-9, 2));
end

function B = shift_in(A, dr, dc)
% B(r, c) = A(r + dr, c + dc), inf outside the grid.
[ny, nx] = size(A);
B = inf(ny, nx);
rs = max(1, 1 - dr):min(ny, ny - dr);
cs = max(1, 1 - dc):min(nx, nx - dc);
B(rs, cs) = A(rs + dr, cs + dc);
end

function [r, c] = cell_of(G, p)
c = min(max(floor((p(1) - G.x0) / G.res) + 1, 1), G.nx);
r = min(max(floor((p(2) - G.y0) / G.res) + 1, 1), G.ny);
end

function i = cell_of_idx(G, p)
[r, c] = cell_of(G, p);
i = sub2ind([G.ny G.nx], r, c);
end
