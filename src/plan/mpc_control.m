function [v, w, sol, info] = mpc_control(x, path, pred, P, tNow, uPrev, solPrev, cfg)
%MPC_CONTROL  Sampling-based model predictive control (cross-entropy
%   method). Optimises a sequence of cfg.mpcN (v, w) commands, each held
%   for cfg.mpcDt seconds (3.2 s ahead by default), through the robot's
%   unicycle model, against predicted people. Only the first command is
%   applied; the rest warm-starts the next call (receding horizon).
%
%   Cost of a rollout:
%     track   distance to the plan's reference r(tau) at every step
%             (space-time plan: followed in time, so it can wait)
%     clear   (comfort - clearance)^2 when closer than cfg.mpcComfort,
%             plus cfg.mpcHit per step closer than cfg.dwaSafe
%     effort  turning and changes in (v, w)
%   Unlike DWA's constant (v, w) arcs, the command can change over the
%   horizon, so the robot can e.g. slow down now and speed up later.
N = cfg.mpcN; dtc = cfg.mpcDt; sub = 2;
tau = (1:N * sub) * (dtc / sub);
Rf = ref_at(path, x, tNow, tau, cfg.vmax);       % (N*sub) x 2
[Q, Rad] = people_at(pred, P, tNow, tau);
K = cfg.mpcSamples; nE = max(2, round(cfg.mpcElite * K));
if isempty(solPrev)
    mu = repmat(uPrev, N, 1);
else
    mu = [solPrev(2:end, :); solPrev(end, :)];
end
sig = repmat([cfg.mpcSigV cfg.mpcSigW], N, 1);
best = inf; bestU = mu;
for it = 1:cfg.mpcIters
    U = mu + sig .* randn(N, 2, K);               % N x 2 x K
    if it == 1                                    % always try a few anchors
        U(:, :, 1) = mu;
        U(:, :, 2) = 0;                           % stop
        U(:, :, 3) = repmat(uPrev, N, 1);         % keep going
    end
    U = clamp_u(U, uPrev, cfg, dtc);
    Jc = rollout_cost(U, x, Rf, Q, Rad, tau, sub, dtc, uPrev, cfg);
    [Js, o] = sort(Jc);
    if Js(1) < best, best = Js(1); bestU = U(:, :, o(1)); end
    E = U(:, :, o(1:nE));
    mu = mean(E, 3);
    sig = max(std(E, 0, 3), [0.02 0.05]);
end
sol = bestU;
v = sol(1, 1); w = sol(1, 2);
info.cost = best;
end

function U = clamp_u(U, uPrev, cfg, dtc)
% Speed/turn limits and acceleration limits between steps.
dv = cfg.accV * dtc; dw = cfg.accW * dtc;
prev = repmat(uPrev, [1 1 size(U, 3)]);
for j = 1:size(U, 1)
    a = U(j, :, :);
    a(1, 1, :) = min(max(a(1, 1, :), prev(1, 1, :) - dv), prev(1, 1, :) + dv);
    a(1, 2, :) = min(max(a(1, 2, :), prev(1, 2, :) - dw), prev(1, 2, :) + dw);
    a(1, 1, :) = min(max(a(1, 1, :), cfg.vmin), cfg.vmax);
    a(1, 2, :) = min(max(a(1, 2, :), -cfg.wmax), cfg.wmax);
    U(j, :, :) = a;
    prev = a;
end
end

function J = rollout_cost(U, x, Rf, Q, Rad, tau, sub, dtc, uPrev, cfg)
K = size(U, 3); N = size(U, 1);
V = reshape(U(:, 1, :), N, K); W = reshape(U(:, 2, :), N, K);
V = repelem(V, sub, 1); W = repelem(W, sub, 1);   % (N*sub) x K
h = dtc / sub;
th = x(3) + cumsum(W * h, 1);
th0 = [repmat(x(3), 1, K); th(1:end - 1, :)];
X = x(1) + cumsum(V .* cos(th0 + W * h / 2) * h, 1);
Y = x(2) + cumsum(V .* sin(th0 + W * h / 2) * h, 1);
J = sum(hypot(X - Rf(:, 1), Y - Rf(:, 2)), 1) * h * cfg.mpcWTrack;
J = J + 2 * cfg.mpcWTrack * hypot(X(end, :) - Rf(end, 1), Y(end, :) - Rf(end, 2));
if ~isempty(Q)
    for j = 1:numel(tau)
        c = min(hypot(X(j, :) - Q(:, j, 1), Y(j, :) - Q(:, j, 2)) - Rad(:, j), [], 1) ...
            - cfg.robotR - cfg.pedR;
        J = J + cfg.mpcWClear * max(0, cfg.mpcComfort - c) .^ 2 * h ...
              + cfg.mpcHit * (c < cfg.dwaSafe);
    end
end
dU = diff([repmat(uPrev, [1 1 K]); U], 1, 1);
J = J + cfg.mpcWSmooth * reshape(sum(sum(dU .^ 2, 1) .* [1 0.2], 2), 1, K) ...
      + cfg.mpcWTurn * sum(W .^ 2, 1) * h;
end
