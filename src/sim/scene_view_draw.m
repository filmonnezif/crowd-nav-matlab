function scene_view_draw(h, log, k)
%SCENE_VIEW_DRAW  Update a view made by scene_view_init to step k of log.
%   k beyond the end of the episode shows the last step (the robot has
%   arrived or timed out), so several episodes can play side by side.
cfg = log.cfg;
nk = numel(log.t);
done = k > nk;
k = min(k, nk);
circ = h.circ;
P = log.P{k};
if isempty(P)
    set(h.ped, 'XData', nan, 'YData', nan);
else
    set(h.ped, 'XData', P(:, 1)' + cfg.pedR * circ(:, 1), ...
               'YData', P(:, 2)' + cfg.pedR * circ(:, 2));
end
T = cat(1, log.P{max(1, k - h.nTrail):k});
if isempty(T), T = [nan nan]; end
set(h.trail, 'XData', T(:, 1), 'YData', T(:, 2));

px = nan; py = nan; ux = nan; uy = nan;
pr = [];
if isfield(log, 'pred') && ~isempty(log.pred{k}), pr = log.pred{k}; end
if ~isempty(pr) && ~isempty(pr.Y)
    Y = pr.Y;
    n = size(Y, 1);
    px = [reshape(Y(:, :, 1)', [], n); nan(1, n)]; px = px(:);
    py = [reshape(Y(:, :, 2)', [], n); nan(1, n)]; py = py(:);
    % rings: display-only radii (Rshow) if present, else the planner's R
    R = [];
    if isfield(pr, 'Rshow') && ~isempty(pr.Rshow), R = pr.Rshow;
    elseif isfield(pr, 'R'), R = pr.R; end
    if ~isempty(R) && any(R(:) > 0)
        ux = []; uy = [];
        for hz = h.uncSteps(h.uncSteps <= size(Y, 2))
            cx = Y(:, hz, 1)' + R(:, hz)' .* circ(:, 1);
            cy = Y(:, hz, 2)' + R(:, hz)' .* circ(:, 2);
            ux = [ux; [cx; nan(1, n)]]; %#ok<AGROW>
            uy = [uy; [cy; nan(1, n)]]; %#ok<AGROW>
        end
        ux = ux(:); uy = uy(:);
    end
end
set(h.pred, 'XData', px, 'YData', py);
set(h.unc, 'XData', ux, 'YData', uy);

p = log.path{k};
if isempty(p) || done, p = [nan nan]; end
set(h.path, 'XData', p(:, 1), 'YData', p(:, 2));
set(h.done, 'XData', log.x(1:k, 1), 'YData', log.x(1:k, 2));
x = log.x(k, :);
set(h.rob, 'XData', x(1) + cfg.robotR * circ(:, 1), ...
           'YData', x(2) + cfg.robotR * circ(:, 2));
set(h.head, 'XData', x(1) + [0 1.6 * cfg.robotR * cos(x(3))], ...
            'YData', x(2) + [0 1.6 * cfg.robotR * sin(x(3))]);
if log.clear(k) < 0
    set(h.rob, 'FaceColor', [0 0 0]);
else
    set(h.rob, 'FaceColor', [0.85 0.2 0.2]);
end
nContact = sum(diff([false; log.clear(1:k) < 0]) > 0);
if isempty(h.label)
    head = strrep(log.scene, '_', '\_');
else
    head = h.label;
end
if done
    status = sprintf('finished at %4.1f s', log.t(end) - log.t(1));
else
    status = sprintf('t = %4.1f s   v = %.2f m/s', log.t(k) - log.t(1), log.u(k, 1));
end
set(h.title, 'String', {head, sprintf('%s   clearance %.2f m   contacts %d', ...
    status, log.clear(k), nContact)});
end
