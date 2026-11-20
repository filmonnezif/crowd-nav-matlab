function h = scene_view_init(ax, log, opts)
%SCENE_VIEW_INIT  Create the graphics for one episode view in axes ax.
%   h = scene_view_init(ax, log)
%   h = scene_view_init(ax, log, struct('legend', true, 'trail', 3.2))
%   opts.rings: prediction steps (0.4 s each) that get an uncertainty
%   ring, default [4 8] = 1.6 s and 3.2 s ahead.
%   Draw a time step with scene_view_draw(h, log, k). Used by
%   animate_episode, animate_compare and view_episodes.
if nargin < 3, opts = struct(); end
o = parse_opts(struct('legend', false, 'trail', 3.2, 'label', '', 'rings', [4 8]), opts);
cfg = log.cfg;
try, legend(ax, 'off'); catch, end   % re-init: drop the old legend first
cla(ax); hold(ax, 'on'); axis(ax, 'equal');
axis(ax, log.task.bounds); box(ax, 'on');
xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');

th = linspace(0, 2 * pi, 24);
h.circ = [cos(th); sin(th)]';
h.ax = ax;
h.label = o.label;
h.nTrail = round(o.trail / cfg.dt);
h.uncSteps = o.rings;
h.goal = plot(ax, log.task.goal(1), log.task.goal(2), 'p', 'MarkerSize', 14, ...
    'MarkerFaceColor', [0.95 0.75 0.1], 'MarkerEdgeColor', 'k');
h.trail = plot(ax, nan, nan, '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 4);
h.ped = patch(ax, nan, nan, [0.25 0.45 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.85);
h.pred = plot(ax, nan, nan, '--', 'Color', [0.45 0.6 0.9], 'LineWidth', 1);
h.unc = plot(ax, nan, nan, '-', 'Color', [0.95 0.55 0.15], 'LineWidth', 0.75);
h.path = plot(ax, nan, nan, '-', 'Color', [0.1 0.65 0.3], 'LineWidth', 1.5);
h.done = plot(ax, nan, nan, '-', 'Color', [0.85 0.2 0.2], 'LineWidth', 1);
h.rob = patch(ax, nan, nan, [0.85 0.2 0.2], 'EdgeColor', 'k');
h.head = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
h.title = title(ax, '');
if o.legend
    legend(ax, [h.ped h.pred h.unc h.path h.done h.rob h.goal], ...
        {'person', 'predicted path (4.8 s)', ringLabel(o.rings), 'robot plan', ...
         'robot so far', 'robot', 'goal'}, 'Location', 'northeastoutside');
end
end

function s = ringLabel(rings)
t = sprintf('%.1f / ', 0.4 * rings);
s = sprintf('90%% uncertainty at %s s', t(1:end - 3));
end
