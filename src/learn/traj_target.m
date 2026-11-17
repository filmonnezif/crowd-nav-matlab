function T = traj_target(Yw, F)
%TRAJ_TARGET  True future (n x 12 x 2, world) -> training target: the
%   correction to constant velocity in the heading frame, n x 24.
P = size(F.base, 2);
dx = Yw(:, :, 1) - F.origin(:, 1);
dy = Yw(:, :, 2) - F.origin(:, 2);
c = cos(F.th); s = sin(F.th);
rx = c .* dx + s .* dy;
ry = -s .* dx + c .* dy;
T = [rx - F.base(:, :, 1), ry - F.base(:, :, 2)];
end
