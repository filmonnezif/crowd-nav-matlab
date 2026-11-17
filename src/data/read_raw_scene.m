function S = read_raw_scene(file, name)
%READ_RAW_SCENE  Load one ETH/UCY raw file into a scene struct.
%   S = read_raw_scene('data/raw/crowds_zara01.txt')
%
%   Raw format: one row per (frame, pedestrian): frame, ped_id, x, y,
%   positions in meters, annotated at 2.5 Hz (dt = 0.4 s).
%
%   Fields of S:
%     name       scene name (file name without extension)
%     frame, id  per-row frame number and pedestrian id
%     xy         per-row position [x y] in meters
%     k          per-row time index (1 = first frame of the scene)
%     t          per-row time in seconds from the first frame
%     frameStep  frame-number increment between annotations (10 here)
%     dt         seconds between annotations (0.4)
%     rowsByK    cell: rowsByK{k} = rows annotated at time index k
%     bounds     [xmin xmax ymin ymax] of all positions
if nargin < 2
    [~, name] = fileparts(file);
end
M = load(file, '-ascii');
M = sortrows(M(:, 1:4), [1 2]);

fr = unique(M(:, 1));
S.name = name;
S.frameStep = mode(diff(fr));
S.dt = 0.4;
S.frame = M(:, 1);
S.id = M(:, 2);
S.xy = M(:, 3:4);
S.k = round((S.frame - fr(1)) / S.frameStep) + 1;
S.t = (S.k - 1) * S.dt;
S.nK = max(S.k);

% Rows grouped by time index, for fast replay lookups.
S.rowsByK = cell(S.nK, 1);
[ks, order] = sort(S.k);
edges = [0; find(diff(ks)); numel(ks)];
for e = 1:numel(edges) - 1
    rows = order(edges(e) + 1:edges(e + 1));
    S.rowsByK{ks(edges(e) + 1)} = rows;
end
S.bounds = [min(S.xy(:, 1)) max(S.xy(:, 1)) min(S.xy(:, 2)) max(S.xy(:, 2))];
end
