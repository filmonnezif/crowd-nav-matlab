% TRAIN_PREDICTORS  Train the learned predictors and compare them with the
% classical ones. Needs the Deep Learning Toolbox (MATLAB / MATLAB Online).
%
%   >> train_predictors                       % all networks
%   >> nets = {'lstm'}; train_predictors      % just one
%
% Data: the clean Phase 1 split. Train = Hotel/Univ/Zara (first 80% of
% each scene) + left/right mirrored copies; val = the last 20%, used for
% early stopping (best validation loss is kept); test = ETH, never
% seen in training. Models are saved to models/<name>.mat.
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
run(fullfile(root, 'startup.m'));
if ~exist('nets', 'var'), nets = {'mlp', 'lstm', 'tcn', 'sgcnn', 'tf'}; end
if ~exist('maxEpochs', 'var'), maxEpochs = 40; end
featOpts = struct('cell', 0.5, 'G', 16, 'predLen', 12);

D = build_dataset(fullfile(root, 'data', 'raw'), 'Verbose', false);
fprintf('Building features...\n');
[Ftr, Ttr] = build_training_set(D.train, D.scenes, featOpts);
[Fm, Tm] = mirror_features(Ftr, Ttr);
Ftr.motion = cat(1, Ftr.motion, Fm.motion);
Ftr.grid = cat(4, Ftr.grid, Fm.grid);
Ttr = [Ttr; Tm];
[Fva, Tva] = build_training_set(D.val, D.scenes, featOpts);
fprintf('Train %d samples (with mirrors), val %d\n', size(Ttr, 1), size(Tva, 1));

for i = 1:numel(nets)
    name = nets{i};
    fprintf('\n=== %s ===\n', name);
    net = predictor_net(name, featOpts);
    Xtr = net_inputs(name, Ftr);
    Xva = net_inputs(name, Fva);
    opts = trainingOptions('adam', ...
        'MaxEpochs', maxEpochs, 'MiniBatchSize', 256, ...
        'InitialLearnRate', 1e-3, 'LearnRateSchedule', 'piecewise', ...
        'LearnRateDropPeriod', 15, 'LearnRateDropFactor', 0.3, ...
        'L2Regularization', 1e-4, 'Shuffle', 'every-epoch', ...
        'ValidationData', {Xva{:}, single(Tva)}, 'ValidationFrequency', 100, ...
        'OutputNetwork', 'best-validation', 'Verbose', true, ...
        'VerboseFrequency', 200, 'Plots', 'none', 'ExecutionEnvironment', 'cpu');
    tic;
    if numel(Xtr) == 1
        net = trainnet(Xtr{1}, single(Ttr), net, 'mse', opts);
    else
        % multi-input: one datastore per input + targets, in InputNames order
        dsM = arrayDatastore(Xtr{1}, 'IterationDimension', 1);
        dsG = arrayDatastore(Xtr{2}, 'IterationDimension', 4);
        dsT = arrayDatastore(single(Ttr), 'IterationDimension', 1);
        ds = combine(dsM, dsG, dsT);
        vM = arrayDatastore(Xva{1}, 'IterationDimension', 1);
        vG = arrayDatastore(Xva{2}, 'IterationDimension', 4);
        vT = arrayDatastore(single(Tva), 'IterationDimension', 1);
        opts.ValidationData = combine(vM, vG, vT);
        net = trainnet(ds, net, 'mse', opts);
    end
    trainSec = toc;
    if ~exist(fullfile(root, 'models'), 'dir'), mkdir(fullfile(root, 'models')); end
    save(fullfile(root, 'models', [name '.mat']), 'net', 'featOpts', 'trainSec');
    clear predict_learned            % drop the cached old model
    fprintf('%s trained in %.0f s\n', name, trainSec);
end

% ---- compare everything ------------------------------------------------
allNames = [{'cv', 'kalman', 'sf'}, nets];
fid = fopen(fullfile(root, 'results', 'phase3_prediction.csv'), 'w');
fprintf(fid, 'predictor,valADE,valFDE,valMovingADE,valMovingFDE,testADE,testFDE\n');
fprintf('\n%-8s %16s %16s %16s\n', '', 'val ADE/FDE', 'val moving', 'ETH test');
mv = ~D.val.isStatic;
for i = 1:numel(allNames)
    fn = make_predictor(allNames{i});
    Rv = eval_predictor(fn, D.val, D.scenes);
    Rt = eval_predictor(fn, D.test, D.scenes);
    r = [Rv.ADE, Rv.FDE, mean(Rv.ade(mv), 'omitnan'), mean(Rv.fde(mv), 'omitnan'), Rt.ADE, Rt.FDE];
    fprintf('%-8s %7.3f / %6.3f %7.3f / %6.3f %7.3f / %6.3f\n', allNames{i}, r);
    fprintf(fid, '%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n', allNames{i}, r);
end
fclose(fid);
