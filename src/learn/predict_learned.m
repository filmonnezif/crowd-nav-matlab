function [Y, info] = predict_learned(H, predLen, dt, name)
%PREDICT_LEARNED  Run a trained network (models/<name>.mat) as a predictor.
%   Same interface as predict_cv: H n x 8 x 2 -> Y n x 12 x 2.
persistent cache
if isempty(cache), cache = struct(); end
if ~isfield(cache, name)
    f = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'models', [name '.mat']);
    if ~exist(f, 'file')
        error('predict_learned:nomodel', ...
            'No trained model %s. Run train_predictors first.', f);
    end
    cache.(name) = load(f);
end
M = cache.(name);
info = struct();
n = size(H, 1);
if n == 0
    Y = zeros(0, predLen, 2); return
end
F = traj_features(H, M.featOpts);
X = net_inputs(name, F);
res = minibatchpredict(M.net, X{:});
Y = traj_decode(res, F);
% The networks were trained on full 8-step histories only. For people
% who appeared less than 8 frames ago (padded history), fall back to
% constant velocity rather than trust an out-of-distribution input.
short = any(isnan(H(:, :, 1)), 2);
if any(short)
    Ycv = predict_cv(H(short, :, :), predLen, dt);
    Y(short, :, :) = Ycv;
end
end
