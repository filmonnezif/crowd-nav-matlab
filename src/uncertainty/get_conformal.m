function C = get_conformal(name, alpha, mode)
%GET_CONFORMAL  Conformal radii for a predictor, calibrated on val.
%   Cached in models/conformal_<name>_<mode>_<alpha>.mat.
if nargin < 3, mode = 'plain'; end
root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
f = fullfile(root, 'models', sprintf('conformal_%s_%s_%02d.mat', name, mode, round(100 * alpha)));
if exist(f, 'file')
    S = load(f); C = S.C; return
end
D = build_dataset(fullfile(root, 'data', 'raw'), 'Verbose', false);
R = eval_predictor(make_predictor(name), D.val, D.scenes);
[err, speed] = window_errors(R, D.val);
C = conformal_calibrate(err, speed, alpha, mode);
if ~exist(fullfile(root, 'models'), 'dir'), mkdir(fullfile(root, 'models')); end
save(f, 'C');
end
