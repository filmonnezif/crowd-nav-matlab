function S = crowd_write_frame(S, t, ids, P)
%CROWD_WRITE_FRAME  Store simulated positions in the scene used by the
%   predictors, so they observe the crowd that actually happened (not
%   the recording). Only acts when t falls on a 0.4 s data frame.
u = t / S.dt;
if abs(u - round(u)) > 1e-6, return; end
k = round(u) + 1;
if k < 1 || k > S.nK || isempty(S.rowsByK{k}) || isempty(ids), return; end
rows = S.rowsByK{k};
[tf, loc] = ismember(S.id(rows), ids);
S.xy(rows(tf), :) = P(loc(tf), :);
end
