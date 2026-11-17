function [F, T] = build_training_set(W, scenes, o)
%BUILD_TRAINING_SET  Features and targets for every window in W.
%   Neighbours come from the scene at the last observed step, exactly as
%   the predictor sees them at run time (see eval_predictor).
if nargin < 3, o = struct(); end
obsLen = size(W.obs, 2);
N = numel(W.ped_id);
names = {scenes.name};
sc = cellstr(W.scene);
parts = {};
order = [];
[us, ~, gs] = unique(sc);
for s = 1:numel(us)
    S = scenes(strcmp(names, us{s}));
    idxS = find(gs == s);
    kObs = W.start_k(idxS) + obsLen - 1;
    [uk, ~, gk] = unique(kObs);
    for u = 1:numel(uk)
        [ids, H] = scene_history(S, uk(u), obsLen);
        ws = idxS(gk == u);
        [tf, loc] = ismember(W.ped_id(ws), ids);
        ws = ws(tf); loc = loc(tf);
        if isempty(ws), continue; end
        Fall = traj_features(H, o);
        parts{end + 1} = subset_features(Fall, loc); %#ok<AGROW>
        order = [order; ws(:)]; %#ok<AGROW>
    end
end
F = parts{1};
for p = 2:numel(parts)
    F.motion = cat(1, F.motion, parts{p}.motion);
    F.grid = cat(4, F.grid, parts{p}.grid);
    F.base = cat(1, F.base, parts{p}.base);
    F.th = cat(1, F.th, parts{p}.th);
    F.origin = cat(1, F.origin, parts{p}.origin);
end
F.index = order;                  % F row r belongs to window order(r)
T = traj_target(W.pred(order, :, :), F);
if numel(order) < N
    fprintf('build_training_set: %d of %d windows had no scene match\n', N - numel(order), N);
end
end

function F = subset_features(F, i)
F.motion = F.motion(i, :, :);
F.grid = F.grid(:, :, :, i);
F.base = F.base(i, :, :);
F.th = F.th(i);
F.origin = F.origin(i, :);
end
