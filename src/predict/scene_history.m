function [ids, H] = scene_history(S, k, obsLen)
%SCENE_HISTORY  Observed tracks of everyone present at time index k.
%   [ids, H] = scene_history(S, k, 8) returns the pedestrians annotated
%   at time index k and their positions over the last obsLen indices,
%   H is n x obsLen x 2 (H(:, end, :) = position at k). Steps before a
%   person appeared are NaN.
ids = zeros(0, 1);
H = zeros(0, obsLen, 2);
if k < 1 || k > S.nK || isempty(S.rowsByK{k}), return; end
rows = S.rowsByK{k};
ids = S.id(rows);
n = numel(ids);
H = nan(n, obsLen, 2);
for j = 1:obsLen
    kk = k - obsLen + j;
    if kk < 1 || isempty(S.rowsByK{kk}), continue; end
    r = S.rowsByK{kk};
    [tf, loc] = ismember(ids, S.id(r));
    H(tf, j, 1) = S.xy(r(loc(tf)), 1);
    H(tf, j, 2) = S.xy(r(loc(tf)), 2);
end
end
