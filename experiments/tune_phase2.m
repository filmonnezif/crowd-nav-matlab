% TUNE_PHASE2  Fit the few hand-set parameters on the VALIDATION split
% (never on test): Kalman acceleration noise q, and Social Force
% strength A and range B. Prints val ADE for each setting.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
D = build_dataset(fullfile(here, '..', 'data', 'raw'), 'Verbose', false);

fprintf('Kalman q (m/s^2):\n');
for q = [0.1 0.25 0.5 1 2]
    R = eval_predictor(make_predictor('kalman', struct('q', q)), D.val, D.scenes);
    fprintf('  q = %.2f   ADE %.3f  FDE %.3f\n', q, R.ADE, R.FDE);
end
fprintf('Social Force (q = 0.5):\n');
best = inf;
for A = [0.25 0.5 1 2]
    for B = [0.1 0.2 0.35]
        R = eval_predictor(make_predictor('sf', struct('A', A, 'B', B)), D.val, D.scenes);
        fprintf('  A = %.2f B = %.2f   ADE %.3f  FDE %.3f\n', A, B, R.ADE, R.FDE);
        if R.ADE < best, best = R.ADE; bestAB = [A B]; end
    end
end
fprintf('Best SF on val: A = %.2f, B = %.2f (ADE %.3f)\n', bestAB, best);
