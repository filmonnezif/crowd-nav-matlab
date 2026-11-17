function [cov, radMean, aTrace] = aci_offline(C, err, speed, startK, gamma)
%ACI_OFFLINE  Adaptive conformal inference (Gibbs & Candes, 2021) run
%   through a test sequence in time order.
%   Each horizon h keeps its own level a_h, starting at C.alpha. When the
%   outcome of an earlier prediction becomes known (h steps after it was
%   made), a_h is nudged:  a_h <- a_h + gamma * (alpha - miss)
%   so misses shrink a_h (bigger circles) and hits let it grow back.
%   Coverage then tracks 1 - alpha over time even when the scene is
%   different from the calibration data (here: ETH vs Hotel/Univ/Zara).
%   startK: the time index of each window's first observed step.
if nargin < 5, gamma = 0.03; end
[startK, o] = sort(startK);
err = err(o, :); speed = speed(o);
N = size(err, 1); P = size(err, 2);
obsEnd = startK + 7;
a = repmat(C.alpha, 1, P);
miss = false(N, P);
rad = zeros(N, P);
aTrace = zeros(N, P);
nextUpd = ones(1, P);           % next window whose outcome is not yet used
for t = 1:N
    for h = 1:P                 % outcomes known by now: obsEnd + h <= current obsEnd
        while nextUpd(h) < t && obsEnd(nextUpd(h)) + h <= obsEnd(t)
            a(h) = a(h) + gamma * (C.alpha - miss(nextUpd(h), h));
            nextUpd(h) = nextUpd(h) + 1;
        end
    end
    q = aci_quantile(C, a);
    switch C.mode
        case 'plain', rad(t, :) = q;
        case 'speed', rad(t, :) = (C.c + speed(t)) * q;
    end
    miss(t, :) = err(t, :) > rad(t, :);
    aTrace(t, :) = a;
end
cov = 1 - mean(miss, 1);
radMean = mean(rad, 1);
end
