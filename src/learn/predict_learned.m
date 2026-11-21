function [Y, info] = predict_learned(H, predLen, dt, name)
%PREDICT_LEARNED  Run a trained network (models/<name>.mat) as a predictor.
%   Same interface as predict_cv: H n x 8 x 2 -> Y n x 12 x 2.
%   Uses the dlnetwork with minibatchpredict when the Deep Learning
%   Toolbox is there, otherwise the exported weights (models/w_<name>.mat)
%   with net_forward, e.g. in Octave.
persistent cache
if isempty(cache), cache = struct(); end
if ~isfield(cache, name)
    md = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'models');
    f = fullfile(md, [name '.mat']);
    fw = fullfile(md, ['w_' name '.mat']);
    useNet = exist(f, 'file') && exist('minibatchpredict', 'file');
    if useNet
        cache.(name) = load(f);
    elseif exist(fw, 'file')
        cache.(name) = load(fw);          % plain weights (export_weights): no toolbox needed
    else
        error('predict_learned:nomodel', ...
            'No trained model %s or %s. Run train_predictors first.', f, fw);
    end
end
M = cache.(name);
info = struct();
n = size(H, 1);
if n == 0
    Y = zeros(0, predLen, 2); return
end
F = traj_features(H, M.featOpts);
if isfield(M, 'net')
    X = net_inputs(name, F);
    res = minibatchpredict(M.net, X{:});
else
    res = net_forward(M, name, F);
end
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
