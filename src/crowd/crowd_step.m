function [crowd, ids, P, V, dev] = crowd_step(crowd, t, xr, ur, cfg)
%CROWD_STEP  Advance the reactive crowd to time t.
%   [crowd, ids, P, V, dev] = crowd_step(crowd, t, xRobot, uRobot, cfg)
%   Who is in the scene follows the recording (people enter and leave
%   when they do in the data). Positions and velocities are simulated.
%   dev (n x 1) is each person's distance from where the recording has
%   them now: how far the robot has pushed them off their route.
S0 = crowd.S0;
[ids, Prec, Vrec] = scene_peds_at(S0, t);
n = numel(ids);
P = Prec; V = Vrec;
% carry over simulated state for people we already have
if ~isempty(crowd.ids) && n > 0
    [tf, loc] = ismember(ids, crowd.ids);
    P(tf, :) = crowd.P(loc(tf), :);
    V(tf, :) = crowd.V(loc(tf), :);
end
dtS = t - crowd.t;
if n > 0 && isfinite(dtS) && dtS > 0
    % where the recording has each person crowdLook seconds from now
    G = zeros(n, 2);
    [~, ti] = ismember(ids, crowd.uid);
    u = (t + cfg.crowdLook) / S0.dt + 1;
    for i = 1:n
        T = crowd.tracks{ti(i)};
        if u >= T(end, 1)
            G(i, :) = T(end, 2:3);
        else
            G(i, :) = lerp1(T(:, 1), T(:, 2:3), u);
        end
    end
    vdes = (G - P) / cfg.crowdLook;
    sp = hypot(vdes(:, 1), vdes(:, 2));
    vdes = vdes .* min(1, cfg.crowdVmax ./ max(sp, 1e-9));
    F = (vdes - V) / cfg.crowdTau;
    % heading of each person (for the field of view)
    sp = hypot(V(:, 1), V(:, 2));
    E = V ./ max(sp, 1e-9);
    E(sp < 0.1, :) = 0;
    % robot
    rob = xr(1:2);
    D = P - rob;
    d = hypot(D(:, 1), D(:, 2));
    N = D ./ max(d, 1e-9);
    w = fov_weight(E, -N, cfg.crowdLambda);
    mag = cfg.crowdAr * exp((cfg.robotR + cfg.pedR - d) / cfg.crowdBr) .* w;
    F = F + mag .* N;
    % Pure head-on repulsion can only slow a person down (a known Social
    % Force dead end). People actually side-step, here to their right.
    F = F + cfg.crowdSide * mag .* [E(:, 2), -E(:, 1)];
    % other people
    if n > 1
        DX = P(:, 1) - P(:, 1)';
        DY = P(:, 2) - P(:, 2)';
        Dd = hypot(DX, DY) + eye(n) * 1e9;
        NX = DX ./ Dd; NY = DY ./ Dd;
        cosf = -(E(:, 1) .* NX + E(:, 2) .* NY);       % other is in front -> 1
        W = cfg.crowdLambda + (1 - cfg.crowdLambda) * (1 + cosf) / 2;
        W(sp < 0.1, :) = 1;
        M = cfg.crowdAp * exp((2 * cfg.pedR - Dd) / cfg.crowdBp) .* W;
        F(:, 1) = F(:, 1) + sum(M .* NX, 2);
        F(:, 2) = F(:, 2) + sum(M .* NY, 2);
    end
    V = V + dtS * F;
    sp = hypot(V(:, 1), V(:, 2));
    V = V .* min(1, cfg.crowdVmax ./ max(sp, 1e-9));
    P = P + dtS * V;
end
dev = hypot(P(:, 1) - Prec(:, 1), P(:, 2) - Prec(:, 2));
crowd.ids = ids; crowd.P = P; crowd.V = V; crowd.t = t;
end

function w = fov_weight(E, toOther, lambda)
% Anisotropy: 1 for things straight ahead, lambda for things behind.
% Standing people (E = 0) look everywhere (weight 1).
c = sum(E .* toOther, 2);
w = lambda + (1 - lambda) * (1 + c) / 2;
w(all(E == 0, 2)) = 1;
end
