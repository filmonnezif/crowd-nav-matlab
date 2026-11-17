function x = diffdrive_step(x, u, dt, lim)
%DIFFDRIVE_STEP  Differential-drive (unicycle) robot kinematics.
%   x = [px py theta], u = [v w] (m/s, rad/s). lim = struct with vmax,
%   wmax; commands are saturated before integration. Uses the exact
%   arc update so large w does not distort the path.
v = min(max(u(1), 0), lim.vmax);
w = min(max(u(2), -lim.wmax), lim.wmax);
th = x(3);
if abs(w) < 1e-6
    x(1) = x(1) + v * dt * cos(th);
    x(2) = x(2) + v * dt * sin(th);
else
    x(1) = x(1) + v / w * (sin(th + w * dt) - sin(th));
    x(2) = x(2) - v / w * (cos(th + w * dt) - cos(th));
end
x(3) = atan2(sin(th + w * dt), cos(th + w * dt));
end
