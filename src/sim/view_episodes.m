function fig = view_episodes(logs, names, opts)
%VIEW_EPISODES  Interactive viewer with a predictor switch.
%   view_episodes({logCV, logKalman, logLSTM}, {'CV', 'Kalman', 'LSTM'})
%   A drop-down picks which run is shown, the slider scrubs through time
%   and Play / Pause animates. Switching predictor keeps the same time, so
%   you can flip between runs at the moment something interesting happens.
%   opts.rings as in scene_view_init.
%   The table under the plot gives each run's summary metrics.
if nargin < 3, opts = struct(); end
o = parse_opts(struct('rings', [4 8]), opts);
n = numel(logs);
fig = figure('Color', 'w', 'Name', 'Crowd navigation: predictor switch', ...
    'Position', [80 80 980 720]);
clf(fig);
ax = axes('Parent', fig, 'Position', [0.07 0.24 0.72 0.66]);
nk = max(cellfun(@(L) numel(L.t), logs));
st.logs = logs; st.names = names; st.ax = ax; st.k = 1; st.nk = nk;
st.cur = 1; st.playing = false; st.rings = o.rings;
st.h = scene_view_init(ax, logs{1}, struct('legend', true, 'label', names{1}, 'rings', o.rings));
uicontrol(fig, 'Style', 'text', 'String', 'Predictor:', 'Units', 'normalized', ...
    'Position', [0.02 0.13 0.09 0.04], 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
st.pop = uicontrol(fig, 'Style', 'popupmenu', 'String', names, 'Units', 'normalized', ...
    'Position', [0.11 0.135 0.18 0.04], 'Callback', @(s, e) on_pick(fig));
st.play = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Play', 'Units', 'normalized', ...
    'Position', [0.31 0.135 0.09 0.04], 'Callback', @(s, e) on_play(fig));
st.slider = uicontrol(fig, 'Style', 'slider', 'Min', 1, 'Max', max(nk, 2), 'Value', 1, ...
    'SliderStep', [1 10] / max(nk - 1, 1), 'Units', 'normalized', ...
    'Position', [0.42 0.135 0.40 0.04], 'Callback', @(s, e) on_slide(fig));
rows = cell(n, 1);
for i = 1:n
    m = episode_metrics(logs{i}, logs{i}.reached);
    rows{i} = sprintf('%-14s reached %-3s  contacts %d (at fault %d)  min clearance %5.2f m  time %4.1f s  stopped %2.0f%%', ...
        names{i}, yesno(m.reached), m.collisions, m.faultCollisions, m.minClear, ...
        m.timeToGoal, 100 * m.freezeFrac);
end
uicontrol(fig, 'Style', 'listbox', 'String', rows, 'Units', 'normalized', ...
    'Position', [0.02 0.01 0.96 0.11], 'FontName', 'Courier', 'Enable', 'inactive');
setappdata(fig, 'viewer', st);
redraw(fig);
end

function on_pick(fig)
st = getappdata(fig, 'viewer');
st.cur = get(st.pop, 'Value');
st.h = scene_view_init(st.ax, st.logs{st.cur}, ...
    struct('legend', true, 'label', st.names{st.cur}, 'rings', st.rings));
setappdata(fig, 'viewer', st);
redraw(fig);
end

function on_slide(fig)
st = getappdata(fig, 'viewer');
st.k = round(get(st.slider, 'Value'));
setappdata(fig, 'viewer', st);
redraw(fig);
end

function on_play(fig)
st = getappdata(fig, 'viewer');
st.playing = ~st.playing;
set(st.play, 'String', ternary(st.playing, 'Pause', 'Play'));
if st.k >= st.nk, st.k = 1; end
setappdata(fig, 'viewer', st);
dt = st.logs{1}.cfg.dt;
while ishandle(fig)
    st = getappdata(fig, 'viewer');
    if ~st.playing || st.k >= st.nk, break; end
    st.k = st.k + 1;
    setappdata(fig, 'viewer', st);
    redraw(fig);
    pause(dt);
end
if ishandle(fig)
    st = getappdata(fig, 'viewer');
    st.playing = false;
    set(st.play, 'String', 'Play');
    setappdata(fig, 'viewer', st);
end
end

function redraw(fig)
st = getappdata(fig, 'viewer');
scene_view_draw(st.h, st.logs{st.cur}, st.k);
set(st.slider, 'Value', st.k);
drawnow;
end

function s = yesno(b)
s = ternary(b, 'yes', 'no');
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
