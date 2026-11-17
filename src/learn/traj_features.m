function F = traj_features(H, o)
%TRAJ_FEATURES  Network inputs for everyone in H (n x 8 x 2 positions).
%   Everything is expressed in each person's own HEADING FRAME: origin at
%   their last observed position, x axis along their last velocity. The
%   networks then only have to learn how people deviate from walking
%   straight, not where the scene is or which way it faces.
%
%   F.motion  n x 7 x 2  step displacements (m per 0.4 s), heading frame
%   F.grid    G x G x 3 x n  social grid around each person, heading frame:
%             ch1 = number of neighbours in the cell, ch2/ch3 = their mean
%             velocity (x, y). Cells are o.cell m; the grid spans
%             +-G*cell/2 m (ahead/behind, left/right).
%   F.base    n x 12 x 2  constant-velocity rollout in the heading frame.
%             The networks predict a CORRECTION to this (residual
%             learning), so an untrained net already behaves like CV.
%   F.th, F.origin  heading angle and origin, to map back to the world.
if nargin < 2, o = struct(); end
if ~isfield(o, 'cell'), o.cell = 0.5; end
if ~isfield(o, 'G'), o.G = 16; end
if ~isfield(o, 'predLen'), o.predLen = 12; end
H = fill_history(H);
n = size(H, 1);
L = size(H, 2);
V = H(:, 2:L, :) - H(:, 1:L - 1, :);                 % n x 7 x 2
vh = reshape(mean(V(:, end - 1:end, :), 2), n, 2);   % heading from last 2 steps
th = atan2(vh(:, 2), vh(:, 1));
th(hypot(vh(:, 1), vh(:, 2)) < 0.02) = 0;            % standing still: no heading
c = cos(th); s = sin(th);
F.th = th;
F.origin = reshape(H(:, L, :), n, 2);
F.motion = zeros(n, L - 1, 2);
F.motion(:, :, 1) = c .* V(:, :, 1) + s .* V(:, :, 2);
F.motion(:, :, 2) = -s .* V(:, :, 1) + c .* V(:, :, 2);
vl = reshape(F.motion(:, end, :), n, 2);             % last step, heading frame
h = reshape(1:o.predLen, 1, []);
F.base = zeros(n, o.predLen, 2);
F.base(:, :, 1) = vl(:, 1) .* h;
F.base(:, :, 2) = vl(:, 2) .* h;

G = o.G;
F.grid = zeros(G, G, 3, n, 'single');
vw = reshape(V(:, end, :), n, 2);                    % world-frame last step
half = G * o.cell / 2;
for i = 1:n
    j = [1:i - 1, i + 1:n];
    if isempty(j), continue; end
    dx = F.origin(j, 1) - F.origin(i, 1);
    dy = F.origin(j, 2) - F.origin(i, 2);
    rx = c(i) * dx + s(i) * dy;
    ry = -s(i) * dx + c(i) * dy;
    col = floor((rx + half) / o.cell) + 1;           % ahead = larger col
    row = floor((ry + half) / o.cell) + 1;           % left  = larger row
    in = col >= 1 & col <= G & row >= 1 & row <= G;
    if ~any(in), continue; end
    j = j(in); col = col(in); row = row(in);
    nvx = c(i) * vw(j, 1) + s(i) * vw(j, 2);
    nvy = -s(i) * vw(j, 1) + c(i) * vw(j, 2);
    idx = sub2ind([G G], row, col);
    cnt = accumarray(idx, 1, [G * G 1]);
    svx = accumarray(idx, nvx, [G * G 1]);
    svy = accumarray(idx, nvy, [G * G 1]);
    m = cnt > 0;
    svx(m) = svx(m) ./ cnt(m);
    svy(m) = svy(m) ./ cnt(m);
    F.grid(:, :, 1, i) = reshape(cnt, G, G);
    F.grid(:, :, 2, i) = reshape(svx, G, G);
    F.grid(:, :, 3, i) = reshape(svy, G, G);
end
end
