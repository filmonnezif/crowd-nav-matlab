function yq = lerp1(x, y, xq)
%LERP1  Fast linear interpolation, same result as interp1(x, y, xq) for
%   increasing x: y is a vector or a matrix with one row per x (columns
%   interpolated together), xq a vector; NaN outside [x(1), x(end)].
%   interp1 is slow in Octave when called thousands of times per episode.
x = x(:);
if isvector(y), y = y(:); end
xq = xq(:);
n = numel(x);
if n == 1
    yq = nan(numel(xq), size(y, 2));
    yq(xq == x, :) = repmat(y, nnz(xq == x), 1);
    return
end
i = sum(x' <= xq, 2);                 % last knot at or before xq
i = min(max(i, 1), n - 1);
w = (xq - x(i)) ./ (x(i + 1) - x(i));
yq = y(i, :) + w .* (y(i + 1, :) - y(i, :));
yq(xq < x(1) | xq > x(n), :) = NaN;
end
