% EXP_PHASE2_PREDICTION  ADE/FDE of the classical predictors.
% Clean val split (Hotel/Univ/Zara, last 20% of each scene) and the ETH
% test scene. "moving" excludes windows flagged isStatic.
here = fileparts(mfilename('fullpath'));
run(fullfile(here, '..', 'startup.m'));
D = build_dataset(fullfile(here, '..', 'data', 'raw'), 'Verbose', false);
names = {'cv', 'kalman', 'sf'};
fprintf('\n%-8s %16s %16s %16s\n', '', 'val ADE/FDE', 'val moving', 'ETH test');
for i = 1:numel(names)
    fn = make_predictor(names{i});
    Rv = eval_predictor(fn, D.val, D.scenes);
    Rt = eval_predictor(fn, D.test, D.scenes);
    mv = ~D.val.isStatic;
    fprintf('%-8s %7.3f / %6.3f %7.3f / %6.3f %7.3f / %6.3f\n', names{i}, ...
        Rv.ADE, Rv.FDE, mean(Rv.ade(mv), 'omitnan'), mean(Rv.fde(mv), 'omitnan'), ...
        Rt.ADE, Rt.FDE);
end
