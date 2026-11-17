function [x, P, Xf] = kalman_cv(Z, dt, q, r)
%KALMAN_CV  Kalman filter with a constant-velocity motion model.
%   [x, P] = kalman_cv(Z, dt, q, r)
%   Z  T x 2 measured positions (NaN = missing), dt sample time.
%   q  acceleration noise (m/s^2): how quickly people change velocity.
%   r  position measurement noise (m).
%   State x = [px; py; vx; vy]. Returns the final state, its 4x4
%   covariance P, and the filtered states Xf (T x 4).
%
%   Predict:  x = F x,          P = F P F' + Q
%   Update:   K = P H' (H P H' + R)^-1
%             x = x + K (z - H x),   P = (I - K H) P
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
G = [dt^2 / 2 0; 0 dt^2 / 2; dt 0; 0 dt];
Q = q^2 * (G * G');
Hm = [1 0 0 0; 0 1 0 0];
R = r^2 * eye(2);
first = find(~isnan(Z(:, 1)), 1);
x = [Z(first, :)'; 0; 0];
P = diag([r^2 r^2 1 1]);        % unknown initial velocity: ~1 m/s spread
T = size(Z, 1);
Xf = nan(T, 4);
Xf(first, :) = x';
for t = first + 1:T
    x = F * x;
    P = F * P * F' + Q;
    if ~isnan(Z(t, 1))
        S = Hm * P * Hm' + R;
        K = P * Hm' / S;
        x = x + K * (Z(t, :)' - Hm * x);
        P = (eye(4) - K * Hm) * P;
    end
    Xf(t, :) = x';
end
end
