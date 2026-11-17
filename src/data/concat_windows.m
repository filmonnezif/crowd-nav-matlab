function W = concat_windows(Ws)
%CONCAT_WINDOWS  Stack a cell array of window structs.
W = Ws{1};
f = fieldnames(W);
for i = 2:numel(Ws)
    for k = 1:numel(f)
        W.(f{k}) = cat(1, W.(f{k}), Ws{i}.(f{k}));
    end
end
end
