function J = phase7_jobs()
%PHASE7_JOBS  The Phase 7 experiment grid, one row per configuration.
%   Each job is run by exp_navigation on the same 120 episodes (6 scenes
%   x 10 start times x 2 directions) and writes results/phase7/<tag>.csv.
%   Columns: tag, predictor, controller, popt (uncertainty), sim_config args.
react = {'crowd', 'reactive'};
staR  = {'planner', 'sta', 'crowd', 'reactive'};
J = {};
% R1: classical baseline, A* + pure pursuit (reactive crowd)
for p = {'none', 'cv'}
    J(end + 1, :) = {['r1_' p{1}], p{1}, 'pp', struct(), react}; %#ok<AGROW>
end
% R2: A* + DWA (main predictors), R3: space-time A* + MPC (every predictor)
preds = {'none', 'cv', 'kalman', 'sf', 'mlp', 'lstm', 'tcn', 'sgcnn'};
for p = {'none', 'cv', 'kalman', 'lstm', 'tcn', 'sgcnn'}
    J(end + 1, :) = {['r2_' p{1}], p{1}, 'dwa', struct(), react}; %#ok<AGROW>
end
for p = preds
    J(end + 1, :) = {['r3_' p{1}], p{1}, 'mpc', struct(), staR}; %#ok<AGROW>
end
% Uncertainty margins on R3: speed-scaled split conformal at 90/70/50%
for p = {'cv', 'lstm'}
    for a = [0.1 0.3 0.5]
        J(end + 1, :) = {sprintf('r3_%s_cp%02d', p{1}, round(100 * (1 - a))), p{1}, 'mpc', ...
            struct('alpha', a, 'cpMode', 'speed'), staR}; %#ok<AGROW>
    end
end
% Replayed (non-reactive) crowd for reference
J(end + 1, :) = {'r3_cv_replay', 'cv', 'mpc', struct(), {'planner', 'sta'}};
end
