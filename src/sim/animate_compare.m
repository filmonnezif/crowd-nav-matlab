function animate_compare(logs, names, videoFile, opts)
%ANIMATE_COMPARE  Play several episodes of the same task side by side.
%   animate_compare({logCV, logLSTM}, {'CV', 'LSTM'})
%   animate_compare(logs, names, 'results/compare.mp4')
%   opts.speed, opts.every, opts.trail as in animate_episode, opts.rings
%   as in scene_view_init;
%   opts.pngAt times (s) to also save a still, opts.pngFile a sprintf
%   pattern with one %d (seconds).
%   Writing a frame takes ~3-4 s in MATLAB Online (vs ~0.05 s on a
%   desktop), so use opts.every there to keep the video short to make.
%   All episodes must start at the same time. A panel whose robot has
%   finished keeps showing its last step until the longest one ends.
if nargin < 3, videoFile = ''; end
if nargin < 4, opts = struct(); end
o = parse_opts(struct('speed', 1, 'trail', 3.2, 'every', 1, ...
    'pngAt', [], 'pngFile', '', 'rings', [4 8]), opts);
n = numel(logs);
cfg = logs{1}.cfg;
nCols = ceil(sqrt(n));
nRows = ceil(n / nCols);
fig = figure('Color', 'w', 'Name', 'Crowd navigation: side by side', ...
    'Position', [50 50 560 * nCols 430 * nRows + 40]);
clf(fig);
H = cell(1, n);
for i = 1:n
    ax = subplot(nRows, nCols, i, 'Parent', fig);
    H{i} = scene_view_init(ax, logs{i}, struct('trail', o.trail, 'label', names{i}, 'rings', o.rings));
end
nk = max(cellfun(@(L) numel(L.t), logs));
vw = [];
if ~isempty(videoFile)
    [vw, videoFile] = open_video(videoFile, round(1 / cfg.dt * o.speed / o.every));
end
frames = 1:o.every:nk;
tStart = tic;
for j = 1:numel(frames)
    k = frames(j);
    for i = 1:n
        scene_view_draw(H{i}, logs{i}, k);
    end
    drawnow;
    if ~isempty(vw)
        writeVideo(vw, getframe(fig));
    elseif isempty(o.pngAt)
        pause(cfg.dt * o.every / o.speed);
    end
    if any(abs((k - 1) * cfg.dt - o.pngAt) < cfg.dt / 2)
        print(fig, sprintf(o.pngFile, round((k - 1) * cfg.dt)), '-dpng', '-r90');
    end
    if ~isempty(vw) && mod(j, 25) == 0
        el = toc(tStart);
        fprintf('  video frame %d / %d, about %.0f s left\n', j, numel(frames), ...
            el / j * (numel(frames) - j));
    end
end
if ~isempty(vw)
    close(vw);
    fprintf('Saved %s\n', videoFile);
end
end
