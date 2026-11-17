function [v, w, info] = dwa_control(x, path, pred, P, tNow, uPrev, cfg)
%DWA_CONTROL  Dynamic Window Approach local controller.
%   Instead of pure pursuit + "slow down near people", try a grid of
%   (v, w) commands - including slow reversing - roll each one out for
%   cfg.dwaHorizon seconds, and check it against where people are
%   PREDICTED to be at each moment. Pick the command with the lowest
%       J = progress cost + clearance cost + speed cost + smoothness cost
%   among those that keep a positive clearance. If every command
%   collides, take the one with the largest clearance (escape).
%   The window: commands reachable from uPrev within the acceleration
%   limits cfg.accV, cfg.accW over cfg.dwaReact seconds.
Th = cfg.dwaHorizon; dtr = 0.2;
tau = dtr:dtr:Th;
nT = numel(tau);
vr = linspace(max(cfg.vmin, uPrev(1) - cfg.accV * cfg.dwaReact), ...
              min(cfg.vmax, uPrev(1) + cfg.accV * cfg.dwaReact), 7);
wr = linspace(max(-cfg.wmax, uPrev(2) - cfg.accW * cfg.dwaReact), ...
              min(cfg.wmax, uPrev(2) + cfg.accW * cfg.dwaReact), 11);
[VV, WW] = meshgrid(vr, wr);
VV = VV(:)'; WW = WW(:)';
nC = numel(VV);

% roll out all candidates at once (unicycle, exact arcs)
th = x(3) + WW .* tau(:);                 % nT x nC
small = abs(WW) < 1e-6;
Wsafe = WW; Wsafe(small) = 1;
X = x(1) + (VV ./ Wsafe) .* (sin(th) - sin(x(3)));
Y = x(2) - (VV ./ Wsafe) .* (cos(th) - cos(x(3)));
X(:, small) = x(1) + VV(small) .* tau(:) * cos(x(3));
Y(:, small) = x(2) + VV(small) .* tau(:) * sin(x(3));

% clearance to predicted people at each rollout time
[Q, Rad] = people_at(pred, P, tNow, tau);  % n x nT x 2, n x nT
minClr = inf(1, nC);
softC = zeros(1, nC);
if ~isempty(Q)
    for j = 1:nT
        dx = X(j, :) - Q(:, j, 1);          % n x nC
        dy = Y(j, :) - Q(:, j, 2);
        c = min(hypot(dx, dy) - Rad(:, j), [], 1) - cfg.robotR - cfg.pedR;
        minClr = min(minClr, c);
        softC = softC + max(0, cfg.dwaComfort - c) .^ 2 * (1 - 0.5 * tau(j) / Th);
    end
end

% progress: end of rollout vs a point further along the A* path
if isempty(path)
    goalPt = x(1:2);
elseif size(path, 2) >= 3                  % space-time plan: where it wants us at the end
    goalPt = ref_at(path, x, tNow, Th, cfg.vmax);
else
    d = hypot(path(:, 1) - x(1), path(:, 2) - x(2));
    [~, i0] = min(d);
    ahead = i0 - 1 + find(d(i0:end) >= cfg.dwaTarget, 1);
    if isempty(ahead), ahead = size(path, 1); end
    goalPt = path(ahead, :);
end
dEnd = hypot(X(end, :) - goalPt(1), Y(end, :) - goalPt(2));
d0 = hypot(x(1) - goalPt(1), x(2) - goalPt(2));
J = cfg.dwaWProg * (dEnd - d0) + cfg.dwaWClear * softC ...
    + cfg.dwaWSpeed * (cfg.vmax - VV) + cfg.dwaWSmooth * abs(WW - uPrev(2));
ok = minClr > cfg.dwaSafe;
if any(ok)
    J(~ok) = inf;
    [~, b] = min(J);
else
    [~, b] = max(minClr);                  % everything collides: escape
end
v = VV(b); w = WW(b);
info.minClr = minClr(b);
info.feasible = any(ok);
end
