% EXP_PHASE4_UNCERTAINTY  Do the uncertainty circles contain the truth?
%
% For each predictor: calibrate conformal radii on val (alpha = 0.1, so
% 90% target coverage), then measure on the ETH test scene, which
% none of the models saw:
%   coverage  share of true future positions inside the circle
%   radius    mean circle radius (m) at 1.6 s, 3.2 s and 4.8 s ahead
% Also compares "model-based" uncertainty: the Kalman filter's own
% covariance (and the GP's predictive std when run in MATLAB with
% models/gp.mat), turned into 90% circles.
%
%   >> exp_phase4_uncertainty
%   >> names = {'cv','kalman','lstm','gp'}; exp_phase4_uncertainty   % MATLAB
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
run(fullfile(root, 'startup.m'));
if ~exist('names', 'var'), names = {'cv', 'kalman', 'sf'}; end
alpha = 0.1;
D = build_dataset(fullfile(root, 'data', 'raw'), 'Verbose', false);
H = [4 8 12];
k2 = sqrt(-2 * log(alpha));   % 90% circle for an isotropic 2-D Gaussian: r = 2.146 sigma

fprintf('\n%-8s %-7s %22s %22s %24s\n', 'pred', 'method', 'val coverage', 'ETH coverage', 'ETH radius @1.6/3.2/4.8s');
rows = {};
for i = 1:numel(names)
    fn = make_predictor(names{i});
    Rv = eval_predictor(fn, D.val, D.scenes);
    Rt = eval_predictor(fn, D.test, D.scenes);
    [ev, sv] = window_errors(Rv, D.val);
    [et, st] = window_errors(Rt, D.test);
    for mode = {'plain', 'speed'}
        C = conformal_calibrate(ev, sv, alpha, mode{1});
        covV = mean(ev <= conformal_radius(C, sv), 1);
        radT = conformal_radius(C, st);
        covT = mean(et <= radT, 1);
        r = mean(radT, 1);
        fprintf('%-8s %-7s   %5.1f%% (h4 %4.1f h12 %4.1f)   %5.1f%% (h4 %4.1f h12 %4.1f)   %5.2f / %5.2f / %5.2f\n', ...
            names{i}, mode{1}, 100 * mean(covV), 100 * covV(4), 100 * covV(12), ...
            100 * mean(covT), 100 * covT(4), 100 * covT(12), r(H));
        rows(end + 1, :) = {names{i}, ['cp-' mode{1}], mean(covV), mean(covT), r(4), r(8), r(12)}; %#ok<SAGROW>
        % adaptive conformal on the ETH sequence (online, outcome-delayed)
        [covA, rA] = aci_offline(C, et, st, D.test.start_k, 0.03);
        fprintf('%-8s %-7s   %22s   %5.1f%% (h4 %4.1f h12 %4.1f)   %5.2f / %5.2f / %5.2f\n', ...
            names{i}, ['aci-' mode{1}(1:2)], '(online on test)', 100 * mean(covA), 100 * covA(4), 100 * covA(12), rA(H));
        rows(end + 1, :) = {names{i}, ['aci-' mode{1}], NaN, mean(covA), rA(4), rA(8), rA(12)}; %#ok<SAGROW>
    end
    % model-based uncertainty, where the model has one
    if any(strcmp(names{i}, {'kalman', 'gp'}))
        if strcmp(names{i}, 'kalman')
            [~, info] = predict_kalman(D.test.obs, 12, 0.4);
            [~, infoV] = predict_kalman(D.val.obs, 12, 0.4);
            sig = @(I) sqrt(max(I.Sigma(:, :, 1, 1), I.Sigma(:, :, 2, 2)));
        else
            [~, info] = predict_gp(D.test.obs, 12, 0.4);
            [~, infoV] = predict_gp(D.val.obs, 12, 0.4);
            sig = @(I) I.sigma;
        end
        radT = k2 * sig(info); radV = k2 * sig(infoV);
        covT = mean(et <= radT, 1); covV = mean(ev <= radV, 1);
        r = mean(radT, 1);
        fprintf('%-8s %-7s   %5.1f%% (h4 %4.1f h12 %4.1f)   %5.1f%% (h4 %4.1f h12 %4.1f)   %5.2f / %5.2f / %5.2f\n', ...
            names{i}, 'model', 100 * mean(covV), 100 * covV(4), 100 * covV(12), ...
            100 * mean(covT), 100 * covT(4), 100 * covT(12), r(H));
        rows(end + 1, :) = {names{i}, 'model', mean(covV), mean(covT), r(4), r(8), r(12)}; %#ok<SAGROW>
    end
end
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
fid = fopen(fullfile(root, 'results', 'phase4_coverage.csv'), 'a');
for r = 1:size(rows, 1)
    fprintf(fid, '%s,%s,%.4f,%.4f,%.3f,%.3f,%.3f\n', rows{r, :});
end
fclose(fid);
