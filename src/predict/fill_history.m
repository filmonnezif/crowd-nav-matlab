function H = fill_history(H)
%FILL_HISTORY  Replace leading NaNs (person not yet seen) with their
%   first observed position, so every predictor gets a full track. A
%   person seen only once therefore looks stationary.
for i = 1:size(H, 1)
    first = find(~isnan(H(i, :, 1)), 1);
    if isempty(first) || first == 1, continue; end
    H(i, 1:first - 1, 1) = H(i, first, 1);
    H(i, 1:first - 1, 2) = H(i, first, 2);
end
% gaps in the middle: carry the last seen position forward
for j = 2:size(H, 2)
    m = isnan(H(:, j, 1));
    H(m, j, :) = H(m, j - 1, :);
end
end
