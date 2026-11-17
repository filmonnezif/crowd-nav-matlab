function Y = traj_decode(res, F)
%TRAJ_DECODE  Network output (n x 24 correction [x1..x12 y1..y12] in the
%   heading frame) -> world positions Y (n x 12 x 2).
n = size(res, 1);
P = size(F.base, 2);
res = double(res);
rx = F.base(:, :, 1) + res(:, 1:P);
ry = F.base(:, :, 2) + res(:, P + 1:2 * P);
c = cos(F.th); s = sin(F.th);
Y = zeros(n, P, 2);
Y(:, :, 1) = F.origin(:, 1) + c .* rx - s .* ry;
Y(:, :, 2) = F.origin(:, 2) + s .* rx + c .* ry;
end
