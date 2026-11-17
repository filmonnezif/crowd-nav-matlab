function [Q, Rad] = people_at(pred, P, tNow, tau)
%PEOPLE_AT  Where people will be tau seconds from now (tau vector).
%   Uses the latest prediction (pred.P0 at pred.tObs, then pred.Y every
%   0.4 s) with linear interpolation. Without a prediction, people stay
%   where they are now (P). Q is n x numel(tau) x 2. Rad (n x numel(tau))
%   is each person's uncertainty radius at those times (0 without one).
if isempty(pred) || isempty(pred.Y)
    Q = repmat(reshape(P, [], 1, 2), 1, numel(tau), 1);
    Rad = zeros(size(P, 1), numel(tau));
    return
end
dtp = 0.4;
tk = pred.tObs + (0:size(pred.Y, 2)) * dtp - tNow;      % times of knots, relative
X = [pred.P0(:, 1), pred.Y(:, :, 1)];
Yv = [pred.P0(:, 2), pred.Y(:, :, 2)];
tau = min(max(tau(:)', tk(1)), tk(end));
Q = zeros(size(X, 1), numel(tau), 2);
Q(:, :, 1) = interp1(tk, X', tau)';
Q(:, :, 2) = interp1(tk, Yv', tau)';
if numel(tau) == 1                                       % interp1 orientation
    Q = reshape(Q, [], 1, 2);
end
% uncertainty radius (0 at the last observation, conformal after that)
Rk = [zeros(size(X, 1), 1), pred.R];
if size(X, 1) == 1
    Rad = interp1(tk, Rk', tau);
else
    Rad = interp1(tk, Rk', tau)';
end
Rad = reshape(Rad, size(X, 1), numel(tau));
end
