function q = aci_quantile(C, a)
%ACI_QUANTILE  Conformal score quantile at miscoverage level a (1 x 12
%   vector, one per horizon), from the stored calibration scores.
%   a <= 0 means "never miss": the largest calibration score.
n = size(C.scores, 1);
q = zeros(1, numel(a));
for h = 1:numel(a)
    if a(h) <= 0
        q(h) = C.scores(end, h);
    else
        k = min(n, max(1, ceil((n + 1) * (1 - a(h)))));
        q(h) = C.scores(k, h);
    end
end
end
