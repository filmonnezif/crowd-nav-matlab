function animate_episode(log, videoFile, opts)
%ANIMATE_EPISODE  Play back an episode; optionally save it as a video.
%   animate_episode(log)                      % play in a figure
%   animate_episode(log, 'results/demo.mp4')  % also write MP4 (VideoWriter)
%   opts.speed  playback speed-up (default 1), opts.trail seconds of trail
%   opts.every  draw every n-th step (default 1)
if nargin < 2, videoFile = ''; end
if nargin < 3, opts = struct(); end
o = parse_opts(struct('speed', 1, 'trail', 3.2, 'every', 1, ...
    'pngAt', [], 'pngFile', ''), opts);
cfg = log.cfg;
b = log.task.bounds;

fig = figure('Color', 'w', 'Name', 'Crowd navigation replay');
clf(fig);
ax = axes('Parent', fig); hold(ax, 'on'); axis(ax, 'equal');
axis(ax, b); box(ax, 'on');
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');

th = linspace(0, 2 * pi, 24);
circ = [cos(th); sin(th)]';
plot(ax, log.task.goal(1), log.task.goal(2), 'p', 'MarkerSize', 16, ...
    'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k');
hTrail = plot(ax, nan, nan, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 4);
hPed = patch(ax, nan, nan, [0.25 0.45 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.85);
hPred = plot(ax, nan, nan, '--', 'Color', [0.45 0.6 0.9], 'LineWidth', 1);
hPath = plot(ax, nan, nan, '-', 'Color', [0.1 0.65 0.3], 'LineWidth', 1.5);
hDone = plot(ax, nan, nan, '-', 'Color', [0.85 0.2 0.2], 'LineWidth', 1);
hRob = patch(ax, nan, nan, [0.85 0.2 0.2], 'EdgeColor', 'k');
hHead = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
hTitle = title(ax, '');

vw = [];
if ~isempty(videoFile)
    % MPEG-4 needs Windows or macOS; MATLAB Online (Linux) falls back to AVI.
    try
        vw = VideoWriter(videoFile, 'MPEG-4');
    catch
        [d, n] = fileparts(videoFile);
        videoFile = fullfile(d, [n '.avi']);
        try
            vw = VideoWriter(videoFile, 'Motion JPEG AVI');
        catch err
            warning('animate_episode:video', 'No video: %s', err.message);
            vw = [];
        end
    end
    if ~isempty(vw)
        vw.FrameRate = round(1 / cfg.dt * o.speed / o.every);
        open(vw);
    end
end

nTrail = round(o.trail / cfg.dt);
for k = 1:o.every:numel(log.t)
    P = log.P{k};
    if isempty(P)
        set(hPed, 'XData', nan, 'YData', nan);
    else
        X = P(:, 1)' + cfg.pedR * circ(:, 1);
        Y = P(:, 2)' + cfg.pedR * circ(:, 2);
        set(hPed, 'XData', X, 'YData', Y);
    end
    k0 = max(1, k - nTrail);
    T = cat(1, log.P{k0:k});
    if isempty(T), T = [nan nan]; end
    set(hTrail, 'XData', T(:, 1), 'YData', T(:, 2));
    if isfield(log, 'pred') && ~isempty(log.pred{k}) && ~isempty(log.pred{k}.Y)
        Y = log.pred{k}.Y;
        n = size(Y, 1);
        px = [reshape(Y(:, :, 1)', [], n); nan(1, n)];
        py = [reshape(Y(:, :, 2)', [], n); nan(1, n)];
        set(hPred, 'XData', px(:), 'YData', py(:));
    else
        set(hPred, 'XData', nan, 'YData', nan);
    end
    p = log.path{k};
    if isempty(p), p = [nan nan]; end
    set(hPath, 'XData', p(:, 1), 'YData', p(:, 2));
    set(hDone, 'XData', log.x(1:k, 1), 'YData', log.x(1:k, 2));
    x = log.x(k, :);
    set(hRob, 'XData', x(1) + cfg.robotR * circ(:, 1), ...
              'YData', x(2) + cfg.robotR * circ(:, 2));
    set(hHead, 'XData', x(1) + [0 1.6 * cfg.robotR * cos(x(3))], ...
               'YData', x(2) + [0 1.6 * cfg.robotR * sin(x(3))]);
    if log.clear(k) < 0
        set(hRob, 'FaceColor', [0 0 0]);
    else
        set(hRob, 'FaceColor', [0.85 0.2 0.2]);
    end
    set(hTitle, 'String', sprintf('%s   t = %4.1f s   v = %.2f m/s   clearance = %.2f m', ...
        strrep(log.scene, '_', '\_'), log.t(k) - log.t(1), log.u(k, 1), log.clear(k)));
    drawnow;
    if ~isempty(vw)
        writeVideo(vw, getframe(fig));
    elseif isempty(o.pngAt)
        pause(cfg.dt * o.every / o.speed);
    end
    if any(abs(log.t(k) - log.t(1) - o.pngAt) < cfg.dt / 2)
        print(fig, sprintf(o.pngFile, round(log.t(k) - log.t(1))), '-dpng', '-r100');
    end
end
if ~isempty(vw)
    close(vw);
    fprintf('Saved %s\n', videoFile);
end
end
