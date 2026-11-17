function m = episode_metrics(log, reached)
%EPISODE_METRICS  Safety and efficiency numbers for one episode.
%   collisions   number of separate contacts (clearance < 0)
%   minClear     smallest robot-to-person gap, edge to edge (m)
%   timeToGoal   seconds (NaN if the goal was not reached)
%   pathLen      meters driven
%   pathRatio    pathLen / straight-line start-goal distance
%   freezeFrac   share of time spent nearly stopped (v < 0.05 m/s)
%   intrusions   seconds spent with clearance < 0.5 m (personal space)
dt = log.cfg.dt;
inContact = log.clear < 0;
m.scene = log.scene;
m.t0 = log.task.t0;
m.reached = reached;
m.collisions = nnz(diff([false; inContact]) == 1);
m.success = reached && m.collisions == 0;
m.minClear = min(log.clear);
m.timeToGoal = NaN;
if reached, m.timeToGoal = log.t(end) - log.t(1); end
m.pathLen = sum(hypot(diff(log.x(:, 1)), diff(log.x(:, 2))));
m.pathRatio = m.pathLen / hypot(log.task.goal(1) - log.task.start(1), ...
    log.task.goal(2) - log.task.start(2));
m.freezeFrac = mean(log.u(:, 1) < 0.05);
m.intrusions = nnz(log.clear < 0.5) * dt;
m.meanPlanMs = mean(log.planMs);
end
