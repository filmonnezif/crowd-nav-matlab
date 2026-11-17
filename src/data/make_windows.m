function W = make_windows(S, obsLen, predLen, stride)
%MAKE_WINDOWS  Cut every pedestrian track into (observed, future) windows.
%   W = make_windows(S, 8, 12, 1)
%
%   Only uses runs of consecutive annotations (no gaps in time), so every
%   window is exactly obsLen+predLen steps of 0.4 s.
%
%   Fields of W (N windows):
%     traj      N x L x 2 positions, L = obsLen + predLen
%     ped_id    N x 1
%     start_k   N x 1 time index of the first step
%     end_k     N x 1 time index of the last step
%     scene     N x 1 cellstr
if nargin < 4, stride = 1; end
L = obsLen + predLen;
ids = unique(S.id);
trajC = {}; pidC = {}; skC = {};
for n = 1:numel(ids)
    m = S.id == ids(n);
    k = S.k(m);
    xy = S.xy(m, :);
    [k, o] = sort(k);
    xy = xy(o, :);
    if numel(k) < L, continue; end
    breaks = [0; find(diff(k) ~= 1); numel(k)];
    for r = 1:numel(breaks) - 1
        a = breaks(r) + 1;
        b = breaks(r + 1);
        starts = a:stride:(b - L + 1);
        if isempty(starts), continue; end
        T = zeros(numel(starts), L, 2);
        for s = 1:numel(starts)
            seg = xy(starts(s):starts(s) + L - 1, :);
            T(s, :, 1) = seg(:, 1);
            T(s, :, 2) = seg(:, 2);
        end
        trajC{end + 1, 1} = T; %#ok<AGROW>
        pidC{end + 1, 1} = repmat(ids(n), numel(starts), 1); %#ok<AGROW>
        skC{end + 1, 1} = k(starts); %#ok<AGROW>
    end
end
if isempty(trajC)
    W.traj = zeros(0, L, 2);
    W.ped_id = zeros(0, 1);
    W.start_k = zeros(0, 1);
else
    W.traj = cat(1, trajC{:});
    W.ped_id = cat(1, pidC{:});
    W.start_k = cat(1, skC{:});
end
W.end_k = W.start_k + L - 1;
W.scene = repmat({S.name}, size(W.ped_id));
end
