function W = subset_windows(W, m)
%SUBSET_WINDOWS  Keep the windows selected by logical or index vector m.
n = numel(W.ped_id);
f = fieldnames(W);
for k = 1:numel(f)
    v = W.(f{k});
    if size(v, 1) ~= n, continue; end
    if ndims(v) == 3
        W.(f{k}) = v(m, :, :);
    else
        W.(f{k}) = v(m, :);
    end
end
end
