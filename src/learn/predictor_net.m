function net = predictor_net(name, o)
%PREDICTOR_NET  The learned predictors (Deep Learning Toolbox).
%   All output a 24-number correction to constant velocity (12 steps x
%   [x y] in the heading frame).
%
%   'mlp'    fully connected net on the 7 flattened step displacements
%            (no notion of time or neighbours: the simplest learner)
%   'lstm'   LSTM reads the 7 displacements as a sequence
%   'tcn'    temporal CNN: 1-D convolutions over time (as a 1x7 "image"
%            with 2 channels), the course CNN applied to motion
%   'sgcnn'  social-grid CNN: a 2-D CNN looks at the 16x16 occupancy /
%            velocity grid of neighbours, its features are joined with
%            the person's own motion, so it can learn to step around people
%   'tf'     small transformer encoder: self-attention over the 7 steps
if nargin < 2, o = struct(); end
if ~isfield(o, 'G'), o.G = 16; end
out = [fullyConnectedLayer(128, 'Name', 'head_fc1'), reluLayer('Name', 'head_relu'), ...
       fullyConnectedLayer(24, 'Name', 'out')];
switch lower(name)
    case 'mlp'
        layers = [featureInputLayer(14, 'Name', 'motion'), ...
            fullyConnectedLayer(128), reluLayer, ...
            fullyConnectedLayer(128), reluLayer, out];
        net = dlnetwork(layers);
    case 'lstm'
        layers = [sequenceInputLayer(2, 'Name', 'motion'), ...
            lstmLayer(64, 'OutputMode', 'last'), out];
        net = dlnetwork(layers);
    case 'tcn'
        layers = [imageInputLayer([1 7 2], 'Name', 'motion', 'Normalization', 'none'), ...
            convolution2dLayer([1 3], 32, 'Padding', 'same'), reluLayer, ...
            convolution2dLayer([1 3], 64, 'Padding', 'same', 'DilationFactor', [1 2]), reluLayer, ...
            convolution2dLayer([1 3], 64), reluLayer, ...    % 'valid': 7 -> 5 steps
            out];
        net = dlnetwork(layers);
    case 'tf'
        % embed each step, add a learned position embedding, one
        % self-attention block with residual connections, average over time
        net = dlnetwork;
        net = addLayers(net, [sequenceInputLayer(2, 'Name', 'motion'), ...
            fullyConnectedLayer(64, 'Name', 'embed')]);
        net = addLayers(net, positionEmbeddingLayer(64, 7, 'Name', 'pos'));
        net = addLayers(net, [additionLayer(2, 'Name', 'add_pos'), ...
            selfAttentionLayer(4, 64, 'Name', 'attn')]);
        net = addLayers(net, [additionLayer(2, 'Name', 'res1'), ...
            layerNormalizationLayer('Name', 'ln1'), ...
            fullyConnectedLayer(128, 'Name', 'ff1'), reluLayer('Name', 'ffr'), ...
            fullyConnectedLayer(64, 'Name', 'ff2')]);
        net = addLayers(net, [additionLayer(2, 'Name', 'res2'), ...
            layerNormalizationLayer('Name', 'ln2'), ...
            globalAveragePooling1dLayer('Name', 'pool'), out]);
        net = connectLayers(net, 'embed', 'pos');
        net = connectLayers(net, 'embed', 'add_pos/in1');
        net = connectLayers(net, 'pos', 'add_pos/in2');
        net = connectLayers(net, 'attn', 'res1/in1');
        net = connectLayers(net, 'add_pos', 'res1/in2');
        net = connectLayers(net, 'ff2', 'res2/in1');
        net = connectLayers(net, 'ln1', 'res2/in2');
        net = initialize(net);
    case 'sgcnn'
        motion = [featureInputLayer(14, 'Name', 'motion'), ...
            fullyConnectedLayer(64, 'Name', 'm_fc'), reluLayer('Name', 'm_relu')];
        grid = [imageInputLayer([o.G o.G 3], 'Name', 'grid', 'Normalization', 'none'), ...
            convolution2dLayer(3, 16, 'Padding', 'same', 'Name', 'g_conv1'), reluLayer('Name', 'g_relu1'), ...
            maxPooling2dLayer(2, 'Stride', 2, 'Name', 'g_pool1'), ...
            convolution2dLayer(3, 32, 'Padding', 'same', 'Name', 'g_conv2'), reluLayer('Name', 'g_relu2'), ...
            maxPooling2dLayer(2, 'Stride', 2, 'Name', 'g_pool2'), ...
            fullyConnectedLayer(64, 'Name', 'g_fc'), reluLayer('Name', 'g_relu3')];
        net = dlnetwork;
        net = addLayers(net, motion);
        net = addLayers(net, grid);
        net = addLayers(net, [concatenationLayer(1, 2, 'Name', 'cat'), out]);
        net = connectLayers(net, 'm_relu', 'cat/in1');
        net = connectLayers(net, 'g_relu3', 'cat/in2');
        net = initialize(net);
    otherwise
        error('predictor_net:name', 'Unknown network %s', name);
end
end
