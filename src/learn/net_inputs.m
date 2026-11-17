function X = net_inputs(name, F)
%NET_INPUTS  Arrange features the way each network expects them.
%   Returns a cell array with one entry per network input.
n = size(F.motion, 1);
flat = [F.motion(:, :, 1), F.motion(:, :, 2)];        % n x 14
switch lower(name)
    case 'mlp'
        X = {single(flat)};
    case {'lstm', 'tf'}
        seq = cell(n, 1);
        for i = 1:n
            seq{i} = single(reshape(F.motion(i, :, :), [], 2)');  % 2 x 7 (C x T)
        end
        X = {seq};
    case 'tcn'
        X = {single(permute(F.motion, [4 2 3 1]))};          % 1 x 7 x 2 x n
    case 'sgcnn'
        X = {single(flat), single(F.grid)};
end
end
