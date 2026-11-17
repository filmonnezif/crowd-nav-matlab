function [Y, info] = predict_learned(H, predLen, dt, name) %#ok<INUSD>
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
end
