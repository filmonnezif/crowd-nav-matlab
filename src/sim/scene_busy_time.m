function [t0, counts] = scene_busy_time(S, horizon)
%SCENE_BUSY_TIME  Start time (s) whose next `horizon` seconds hold the
%   most pedestrians on average. Good for a demo that shows interaction.
if nargin < 2, horizon = 30; end
counts = cellfun(@numel, S.rowsByK);
n = max(1, round(horizon / S.dt));
c = cumsum([0; counts(:)]);
avg = (c(n + 1:end) - c(1:end - n)) / n;
[~, k] = max(avg);
t0 = (k - 1) * S.dt;
end
