function [ids, P, V] = scene_peds_at(S, t)
%SCENE_PEDS_AT  Replay the recorded crowd at any time t (seconds).
%   [ids, P, V] = scene_peds_at(S, t) returns the pedestrians annotated
%   at both neighbouring 0.4 s frames, their linearly interpolated
%   positions P (n x 2) and velocities V (n x 2, m/s).
ids = zeros(0, 1); P = zeros(0, 2); V = zeros(0, 2);
u = t / S.dt;
k = floor(u) + 1;
a = u - floor(u);
if k < 1 || k + 1 > S.nK, return; end
rA = S.rowsByK{k};
rB = S.rowsByK{k + 1};
if isempty(rA) || isempty(rB), return; end
[ids, ia, ib] = intersect(S.id(rA), S.id(rB));
PA = S.xy(rA(ia), :);
PB = S.xy(rB(ib), :);
P = (1 - a) * PA + a * PB;
V = (PB - PA) / S.dt;
ids = ids(:);
end
