% SHOW_ARCHITECTURES  Diagrams of every predictor network.
%
%   >> show_architectures                 % layer graphs -> results/arch_*.png
%   >> deepNetworkDesigner(predictor_net('sgcnn'))   % open one in the app
%
% Uses the trained model if models/<name>.mat exists, otherwise a fresh
% (untrained) copy of the same architecture. Also prints the number of
% learnable parameters for the report table.
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
run(fullfile(root, 'startup.m'));
if ~exist('nets', 'var'), nets = {'mlp', 'lstm', 'tcn', 'sgcnn', 'tf'}; end
titles = struct('mlp', 'MLP (fully connected)', 'lstm', 'LSTM', ...
    'tcn', 'Temporal CNN', 'sgcnn', 'Social-grid CNN', 'tf', 'Transformer');
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
fprintf('\n%-8s %10s %8s\n', 'network', 'params', 'layers');
for i = 1:numel(nets)
    f = fullfile(root, 'models', [nets{i} '.mat']);
    if exist(f, 'file')
        M = load(f); net = M.net;
    else
        net = predictor_net(nets{i});
    end
    nParams = sum(cellfun(@numel, net.Learnables.Value));
    fprintf('%-8s %10d %8d\n', nets{i}, nParams, numel(net.Layers));
    fig = figure('Color', 'w', 'Name', nets{i}, 'Position', [100 100 520 640]);
    plot(net);
    title(sprintf('%s  (%s learnable parameters)', titles.(nets{i}), ...
        num2str(nParams)));
    exportgraphics(fig, fullfile(root, 'results', ['arch_' nets{i} '.png']), 'Resolution', 150);
end
