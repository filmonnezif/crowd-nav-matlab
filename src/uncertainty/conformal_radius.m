function R = conformal_radius(C, speed)
%CONFORMAL_RADIUS  Radii (n x 12, m) for people walking at speed (n x 1).
if isempty(C)
    R = zeros(numel(speed), 12);
    return
end
switch C.mode
    case 'plain', R = repmat(C.q, numel(speed), 1);
    case 'speed', R = (C.c + speed(:)) .* C.q;
end
end
