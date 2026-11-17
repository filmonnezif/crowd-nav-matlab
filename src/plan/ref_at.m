function R = ref_at(path, x, tNow, tau, vmax)
%REF_AT  Where the plan wants the robot tau seconds from now (k x 2).
%   A space-time plan ([x y t] rows) is followed in time, so a WAIT in
%   the plan means the reference stands still. A geometric A* path
%   ([x y] rows) is followed at full speed from the closest point.
tau = tau(:);
if isempty(path)
    R = repmat(x(1:2), numel(tau), 1);
    return
end
if size(path, 2) >= 3
    tk = path(:, 3);
    [tk, iu] = unique(tk);
    p = path(iu, 1:2);
    if numel(tk) == 1
        R = repmat(p, numel(tau), 1);
        return
    end
    tq = min(max(tNow + tau, tk(1)), tk(end));
    R = [interp1(tk, p(:, 1), tq), interp1(tk, p(:, 2), tq)];
    return
end
d = hypot(path(:, 1) - x(1), path(:, 2) - x(2));
[~, i0] = min(d);
p = [x(1:2); path(i0:end, :)];
s = [0; cumsum(hypot(diff(p(:, 1)), diff(p(:, 2))))];
[s, iu] = unique(s);
p = p(iu, :);
if numel(s) == 1
    R = repmat(p, numel(tau), 1);
    return
end
sq = min(vmax * tau, s(end));
R = [interp1(s, p(:, 1), sq), interp1(s, p(:, 2), sq)];
end
