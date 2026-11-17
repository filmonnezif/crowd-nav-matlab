function [n, mask] = count_overlap(A, B)
%COUNT_OVERLAP  How many windows of B share time with a window of A
%   for the same pedestrian in the same scene (a train/val leak).
%   A, B need fields scene, ped_id, start_k, end_k.
%   Works on the course's processed .mat files too: pass start_frame as
%   start_k and start_frame + 19*frameStep as end_k.
sa = cellstr(A.scene); sb = cellstr(B.scene);
[~, ~, si] = unique([sa(:); sb(:)]);
ka = si(1:numel(sa)) * 1e6 + double(A.ped_id(:));
kb = si(numel(sa) + 1:end) * 1e6 + double(B.ped_id(:));
mask = false(numel(kb), 1);
[ukb, ~, gb] = unique(kb);
for u = 1:numel(ukb)
    ia = find(ka == ukb(u));
    if isempty(ia), continue; end
    ib = find(gb == u);
    for j = ib(:)'
        hit = A.start_k(ia) <= B.end_k(j) & B.start_k(j) <= A.end_k(ia);
        mask(j) = any(hit);
    end
end
n = nnz(mask);
end
