function res = net_forward(W, name, F)
%NET_FORWARD  Run a trained predictor network without the Deep Learning
%   Toolbox (plain matrix code; also works in Octave).
%   res = net_forward(W, 'lstm', F)   % F from traj_features, res n x 24
%   W is the weight file written by export_weights (models/w_<name>.mat):
%   W.L{i} = learnable i in the network's own order, W.names{i} its
%   'Layer/Parameter' name. check_forward compares the result with
%   minibatchpredict in MATLAB.
n = size(F.motion, 1);
flat = [F.motion(:, :, 1), F.motion(:, :, 2)]';          % 14 x n
L = W.L;
switch lower(name)
    case 'mlp'
        h = relu(fc(L{1}, L{2}, flat));
        h = relu(fc(L{3}, L{4}, h));
        res = head(L(5:8), h);
    case 'lstm'
        Wi = L{1}; R = L{2}; b = L{3};
        H = size(R, 2);
        hS = zeros(H, n); cS = zeros(H, n);
        for t = 1:size(F.motion, 2)
            x = reshape(F.motion(:, t, :), n, 2)';          % 2 x n
            z = Wi * x + R * hS + b;
            ig = sigm(z(1:H, :)); fg = sigm(z(H + 1:2 * H, :));
            gg = tanh(z(2 * H + 1:3 * H, :)); og = sigm(z(3 * H + 1:4 * H, :));
            cS = fg .* cS + ig .* gg;
            hS = og .* tanh(cS);
        end
        res = head(L(4:7), hS);
    case 'tcn'
        X = permute(F.motion, [2 3 1]);                     % 7 x 2 x n (time x ch)
        X = relu(conv1d(X, L{1}, L{2}, 1, 'same'));
        X = relu(conv1d(X, L{3}, L{4}, 2, 'same'));
        X = relu(conv1d(X, L{5}, L{6}, 1, 'valid'));        % 5 x 64 x n
        res = head(L(7:10), reshape(X, [], n));             % (time, ch) column-major = (1, w, c)
    case 'sgcnn'
        % two branches: look the layers up by name
        p = @(nm) L{strcmp(W.names, nm)};
        m = relu(fc(p('m_fc/Weights'), p('m_fc/Bias'), flat));          % 64 x n
        G = double(F.grid);                                             % G x G x 3 x n
        G = maxpool2(relu(conv2same(G, p('g_conv1/Weights'), p('g_conv1/Bias'))));
        G = maxpool2(relu(conv2same(G, p('g_conv2/Weights'), p('g_conv2/Bias'))));
        g = relu(fc(p('g_fc/Weights'), p('g_fc/Bias'), reshape(G, [], n)));
        res = head({p('head_fc1/Weights'), p('head_fc1/Bias'), p('out/Weights'), p('out/Bias')}, [m; g]);
    otherwise
        error('net_forward:name', 'No plain-code forward pass for %s', name);
end
res = res';                                                 % n x 24
end

function y = fc(Wt, b, x)
y = Wt * x + b(:);
end

function y = head(L, x)
y = fc(L{3}, L{4}, relu(fc(L{1}, L{2}, x)));
end

function y = relu(x)
y = max(x, 0);
end

function y = sigm(x)
y = 1 ./ (1 + exp(-x));
end

function Y = conv1d(X, K, b, dil, pad)
% X T x Cin x n; K is the [1 kw Cin Cout] kernel of a [1 k] conv2d layer
% (cross-correlation, as in convolution2dLayer).
K = reshape(K, size(K, 2), size(K, 3), size(K, 4));    % kw x Cin x Cout
[kw, cin, cout] = size(K);
[T, ~, n] = size(X);
span = (kw - 1) * dil;
if strcmp(pad, 'same')
    p = floor(span / 2);
    X = [zeros(p, cin, n); X; zeros(span - p, cin, n)];
end
To = size(X, 1) - span;
Y = repmat(reshape(b(:), 1, cout, 1), [To 1 n]);
for j = 1:kw
    Xs = X((j - 1) * dil + (1:To), :, :);                % To x Cin x n
    Kj = reshape(K(j, :, :), cin, cout);
    Xm = reshape(permute(Xs, [1 3 2]), To * n, cin);
    Y = Y + permute(reshape(Xm * Kj, To, n, cout), [1 3 2]);
end
end

function Y = conv2same(X, K, b)
% X H x W x Cin x n, K kh x kw x Cin x Cout, 'same' zero padding.
[kh, kw, cin, cout] = size(K);
[Hh, Ww, ~, n] = size(X);
ph = floor((kh - 1) / 2); pw = floor((kw - 1) / 2);
Xp = zeros(Hh + kh - 1, Ww + kw - 1, cin, n);
Xp(ph + (1:Hh), pw + (1:Ww), :, :) = X;
Y = repmat(reshape(b(:), 1, 1, cout), [Hh Ww 1 n]);
for i = 1:kh
    for j = 1:kw
        Xs = Xp(i - 1 + (1:Hh), j - 1 + (1:Ww), :, :);     % H x W x Cin x n
        Xm = reshape(permute(Xs, [1 2 4 3]), Hh * Ww * n, cin);
        Kij = reshape(K(i, j, :, :), cin, cout);
        Y = Y + permute(reshape(Xm * Kij, Hh, Ww, n, cout), [1 2 4 3]);
    end
end
end

function Y = maxpool2(X)
% 2 x 2 max pooling, stride 2.
[Hh, Ww, c, n] = size(X);
X = X(1:2 * floor(Hh / 2), 1:2 * floor(Ww / 2), :, :);
Y = max(X(1:2:end, :, :, :), X(2:2:end, :, :, :));
Y = max(Y(:, 1:2:end, :, :), Y(:, 2:2:end, :, :));
end
