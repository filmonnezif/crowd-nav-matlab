function fn = make_predictor(name, opts)
%MAKE_PREDICTOR  Common handle for every predictor:
%   Y = fn(H, predLen, dt)  with H n x obsLen x 2 (everyone in the scene
%   at that moment), Y n x predLen x 2.
%   Names: 'cv', 'cv2' (2-step velocity average), 'kalman', 'sf'.
if nargin < 2, opts = struct(); end
switch lower(name)
    case 'cv'
        fn = @(H, L, dt) predict_cv(H, L, dt, opts);
    case 'cv2'
        o = opts; o.nAvg = 2;
        fn = @(H, L, dt) predict_cv(H, L, dt, o);
    case 'kalman'
        fn = @(H, L, dt) predict_kalman(H, L, dt, opts);
    case 'sf'
        fn = @(H, L, dt) predict_social_force(H, L, dt, opts);
    otherwise
        error('make_predictor:name', 'Unknown predictor %s', name);
end
end
