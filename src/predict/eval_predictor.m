function R = eval_predictor(fn, W, scenes, obsLen, predLen)
%EVAL_PREDICTOR  ADE/FDE of a predictor on a set of windows.
%   R = eval_predictor(make_predictor('kalman'), D.val, D.scenes, 8, 12)
%   For each window the predictor sees everyone present in the scene at
%   the last observed step (so Social Force can use neighbours), and the
%   window's own pedestrian is scored.
%   ADE = mean distance over the 12 future steps, FDE = distance at the
%   last step (meters). R.ade, R.fde are per window; R.ADE, R.FDE means.
if nargin < 4, obsLen = 8; end
if nargin < 5, predLen = 12; end
N = numel(W.ped_id);
R.ade = nan(N, 1);
R.fde = nan(N, 1);
R.Y = nan(N, predLen, 2);
names = {scenes.name};
sc = cellstr(W.scene);
[us, ~, gs] = unique(sc);
for s = 1:numel(us)
    S = scenes(strcmp(names, us{s}));
    idxS = find(gs == s);
    kObs = W.start_k(idxS) + obsLen - 1;
    [uk, ~, gk] = unique(kObs);
    for u = 1:numel(uk)
        [ids, H] = scene_history(S, uk(u), obsLen);
        Y = fn(H, predLen, S.dt);
        for w = idxS(gk == u)'
            i = find(ids == W.ped_id(w), 1);
            if isempty(i), continue; end
            yp = reshape(Y(i, :, :), predLen, 2);
            yt = reshape(W.pred(w, :, :), predLen, 2);
            e = hypot(yp(:, 1) - yt(:, 1), yp(:, 2) - yt(:, 2));
            R.ade(w) = mean(e);
            R.fde(w) = e(end);
            R.Y(w, :, :) = reshape(yp, 1, predLen, 2);
        end
    end
end
R.ADE = mean(R.ade, 'omitnan');
R.FDE = mean(R.fde, 'omitnan');
R.missing = nnz(isnan(R.ade));
end
