% CHECK_OLD_SPLIT  Show the train/val leak of a "last 20% of windows" split
% (the way the course's processed/ folder was built) next to the clean one.
run(fullfile(fileparts(mfilename('fullpath')), '..', 'startup.m'));
rawDir = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'raw');
files = dir(fullfile(rawDir, '*.txt'));
tr = {}; va = {};
for f = 1:numel(files)
    S = read_raw_scene(fullfile(rawDir, files(f).name));
    if strcmp(S.name, 'biwi_eth'), continue; end
    W = make_windows(S, 8, 12, 1);
    [~, order] = sort(W.start_k);
    nTr = floor(0.8 * numel(order));
    tr{end + 1} = subset_windows(W, order(1:nTr)); %#ok<SAGROW>
    va{end + 1} = subset_windows(W, order(nTr + 1:end)); %#ok<SAGROW>
end
A = concat_windows(tr); B = concat_windows(va);
fprintf('Old split: %d train, %d val, %d val windows leak into train\n', ...
    numel(A.ped_id), numel(B.ped_id), count_overlap(A, B));
D = build_dataset(rawDir, 'Verbose', false);
fprintf('Clean split: %d train, %d val, %d leaking\n', ...
    numel(D.train.ped_id), numel(D.val.ped_id), D.leakTrainVal);
