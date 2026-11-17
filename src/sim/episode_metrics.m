function m = episode_metrics(log, reached)
%EPISODE_METRICS  Safety and efficiency numbers for one episode.
%   collisions   number of separate contacts (clearance < 0)
%   faultCollisions  contacts the robot drove into (see below)
%   minClear     smallest robot-to-person gap, edge to edge (m)
%   timeToGoal   seconds (NaN if the goal was not reached)
%   pathLen      meters driven
%   pathRatio    pathLen / straight-line start-goal distance
%   freezeFrac   share of time spent nearly stopped (|v| < 0.05 m/s)
%   intrusions   seconds spent with clearance < 0.5 m (personal space)
dt = log.cfg.dt;
inContact = log.clear < 0;
m.scene = log.scene;
m.t0 = log.task.t0;
m.reached = reached;
m.collisions = nnz(diff([false; inContact]) == 1);
m.success = reached && m.collisions == 0;
% At-fault contacts: at the moment a contact starts, was the ROBOT moving
% toward that person (closing speed > 0.1 m/s)? In a replayed crowd
% people never yield, so many contacts are people walking into a robot
% that is stopped or backing away; those are not counted here.
onsets = find(diff([false; inContact]) == 1);
m.faultCollisions = 0;
for k = onsets(:)'
    P = log.P{k};
    if isempty(P), continue; end
    x = log.x(k, :);
    d = hypot(P(:, 1) - x(1), P(:, 2) - x(2));
    [dmin, j] = min(d);
    dirv = (P(j, :) - x(1:2)) / max(dmin, 1e-6);
    closing = log.u(k, 1) * (cos(x(3)) * dirv(1) + sin(x(3)) * dirv(2));
    m.faultCollisions = m.faultCollisions + (closing > 0.1);
end
m.successNoFault = reached && m.faultCollisions == 0;
m.minClear = min(log.clear);
m.timeToGoal = NaN;
if reached, m.timeToGoal = log.t(end) - log.t(1); end
m.pathLen = sum(hypot(diff(log.x(:, 1)), diff(log.x(:, 2))));
m.pathRatio = m.pathLen / hypot(log.task.goal(1) - log.task.start(1), ...
    log.task.goal(2) - log.task.start(2));
m.freezeFrac = mean(abs(log.u(:, 1)) < 0.05);
m.intrusions = nnz(log.clear < 0.5) * dt;
m.meanPlanMs = mean(log.planMs);
end
