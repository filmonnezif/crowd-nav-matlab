% EXPORT_WEIGHTS  Save each trained network's weights as plain arrays
% (models/w_<name>.mat, -v7) so net_forward can run it without the Deep
% Learning Toolbox (Octave, or for the big Phase 7 grid), then check that
% net_forward gives the same predictions as minibatchpredict.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
root = fullfile(here, '..');
names = {'mlp', 'lstm', 'tcn', 'sgcnn'};
D = build_dataset(fullfile(root, 'data', 'raw'), 'Verbose', false);
idx = round(linspace(1, numel(D.val.ped_id), 3000));
H = D.val.obs(idx, :, :);
for i = 1:numel(names)
    M = load(fullfile(root, 'models', [names{i} '.mat']));
    T = M.net.Learnables;
    W.L = cell(height(T), 1);
    W.names = cell(height(T), 1);
    for j = 1:height(T)
        W.L{j} = double(extractdata(T.Value{j}));
        W.names{j} = [char(T.Layer(j)) '/' char(T.Parameter(j))];
    end
    W.featOpts = M.featOpts;
    save(fullfile(root, 'models', ['w_' names{i} '.mat']), '-struct', 'W', '-v7');
    F = traj_features(H, M.featOpts);
    X = net_inputs(names{i}, F);
    r1 = double(minibatchpredict(M.net, X{:}));
    r2 = net_forward(W, names{i}, F);
    fprintf('%-6s %2d learnables  max |toolbox - net_forward| = %.2e\n', ...
        names{i}, height(T), max(abs(r1(:) - r2(:))));
    fprintf('       %s\n', strjoin(W.names', ', '));
    clear W
end
