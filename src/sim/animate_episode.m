function animate_episode(log, videoFile, opts)
%ANIMATE_EPISODE  Play back an episode; optionally save it as a video.
%   animate_episode(log)                      % play in a figure
%   animate_episode(log, 'results/demo.mp4')  % also write MP4 (VideoWriter)
%   opts.speed  playback speed-up (default 1), opts.trail seconds of trail
%   opts.every  draw every n-th step (default 1)
%   opts.pngAt  times (s) to also save a still, opts.pngFile sprintf pattern
%   See animate_compare for several episodes side by side.
if nargin < 2, videoFile = ''; end
if nargin < 3, opts = struct(); end
o = parse_opts(struct('speed', 1, 'trail', 3.2, 'every', 1, ...
    'pngAt', [], 'pngFile', ''), opts);
cfg = log.cfg;

fig = figure('Color', 'w', 'Name', 'Crowd navigation replay');
clf(fig);
ax = axes('Parent', fig);
h = scene_view_init(ax, log, struct('trail', o.trail));

vw = [];
if ~isempty(videoFile)
    [vw, videoFile] = open_video(videoFile, round(1 / cfg.dt * o.speed / o.every));
end
for k = 1:o.every:numel(log.t)
    scene_view_draw(h, log, k);
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
