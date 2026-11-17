function D = build_dataset(rawDir, varargin)
%BUILD_DATASET  Clean train/val/test windows from the ETH/UCY raw files.
%   D = build_dataset('data/raw')
%   D = build_dataset('data/raw', 'TestScene', 'biwi_eth', 'ValFraction', 0.2)
%
%   What it fixes compared with the course's processed/ split:
%   1. Leak between train and val. The old split took the last 20% of
%      windows in time, but windows slide with stride 1, so windows near
%      the boundary shared frames with train windows of the same person.
%      Here each scene gets one split time; a train window must END
%      before it and a val window must START after it. Windows that
%      straddle the split are dropped, so no frame is in both sets.
%   2. Near-static windows. Windows whose pedestrian moves less than
%      StaticThresh meters over all 20 steps are flagged in isStatic.
%      With 'DropStaticTrain', true they are removed from train only
%      (val/test keep them so evaluation stays honest).
%
%   Output D has D.train, D.val, D.test (window structs with fields
%   obs N x 8 x 2, pred N x 12 x 2, scene, ped_id, start_k, end_k,
%   isStatic), D.scenes (raw scene structs for the simulator) and
%   D.report (counts per scene and split).
defaults = struct('TestScene', 'biwi_eth', 'ValFraction', 0.2, ...
    'ObsLen', 8, 'PredLen', 12, 'Stride', 1, 'StaticThresh', 0.5, ...
    'DropStaticTrain', false, 'Verbose', true);
o = parse_opts(defaults, varargin{:});

files = dir(fullfile(rawDir, '*.txt'));
if isempty(files)
    error('build_dataset:nofiles', 'No .txt files in %s', rawDir);
end
tr = {}; va = {}; te = {};
D.scenes = struct([]);
rep = {};
for f = 1:numel(files)
    S = read_raw_scene(fullfile(rawDir, files(f).name));
    if isempty(D.scenes), D.scenes = S; else, D.scenes(end + 1) = S; end
    W = make_windows(S, o.ObsLen, o.PredLen, o.Stride);
    disp20 = sqrt(sum((W.traj(:, end, :) - W.traj(:, 1, :)).^2, 3));
    W.isStatic = disp20(:) < o.StaticThresh;
    if strcmp(S.name, o.TestScene)
        te{end + 1} = W; %#ok<AGROW>
        rep(end + 1, :) = {S.name, 0, 0, numel(W.ped_id), 0, mean(W.isStatic)}; %#ok<AGROW>
    else
        kSplit = floor((1 - o.ValFraction) * S.nK);
        inTrain = W.end_k <= kSplit;
        inVal = W.start_k > kSplit;
        tr{end + 1} = subset_windows(W, inTrain); %#ok<AGROW>
        va{end + 1} = subset_windows(W, inVal); %#ok<AGROW>
        rep(end + 1, :) = {S.name, nnz(inTrain), nnz(inVal), 0, ...
            nnz(~inTrain & ~inVal), mean(W.isStatic)}; %#ok<AGROW>
    end
end
D.train = concat_windows(tr);
D.val = concat_windows(va);
D.test = concat_windows(te);
if o.DropStaticTrain
    D.train = subset_windows(D.train, ~D.train.isStatic);
end
names = {'train', 'val', 'test'};
for i = 1:3
    X = D.(names{i});
    X.obs = X.traj(:, 1:o.ObsLen, :);
    X.pred = X.traj(:, o.ObsLen + 1:end, :);
    X = rmfield(X, 'traj');
    D.(names{i}) = X;
end
D.report = rep;
D.opts = o;
D.leakTrainVal = count_overlap(D.train, D.val);

if o.Verbose
    fprintf('\n%-14s %8s %8s %8s %9s %8s\n', 'scene', 'train', 'val', ...
        'test', 'dropped', 'static');
    for r = 1:size(rep, 1)
        fprintf('%-14s %8d %8d %8d %9d %7.1f%%\n', rep{r, 1:5}, 100 * rep{r, 6});
    end
    fprintf('%-14s %8d %8d %8d\n', 'TOTAL', numel(D.train.ped_id), ...
        numel(D.val.ped_id), numel(D.test.ped_id));
    fprintf('Train/val windows sharing frames (same person): %d\n\n', ...
        D.leakTrainVal);
end
end
