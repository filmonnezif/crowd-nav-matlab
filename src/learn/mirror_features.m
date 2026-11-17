function [F, T] = mirror_features(F, T)
%MIRROR_FEATURES  Left-right mirror image of every sample (data
%   augmentation): negate the sideways (y) components and flip the grid
%   rows. People pass on both sides, so this doubles the data for free.
P = size(F.base, 2);
F.motion(:, :, 2) = -F.motion(:, :, 2);
F.base(:, :, 2) = -F.base(:, :, 2);
F.grid = flip(F.grid, 1);
F.grid(:, :, 3, :) = -F.grid(:, :, 3, :);
F.th = -F.th;                    % only used for decoding; not for training
T(:, P + 1:end) = -T(:, P + 1:end);
end
