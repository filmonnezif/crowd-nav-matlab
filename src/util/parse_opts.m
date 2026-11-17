function opts = parse_opts(defaults, varargin)
%PARSE_OPTS  Merge name/value pairs (or a struct) into a struct of defaults.
%   opts = parse_opts(defaults, 'Name', value, ...)
%   opts = parse_opts(defaults, s)   % s is a struct of overrides
opts = defaults;
if numel(varargin) == 1 && isstruct(varargin{1})
    s = varargin{1};
    f = fieldnames(s);
    for k = 1:numel(f)
        opts.(f{k}) = s.(f{k});
    end
    return
end
if mod(numel(varargin), 2) ~= 0
    error('parse_opts:pairs', 'Options must be name/value pairs.');
end
for k = 1:2:numel(varargin)
    opts.(varargin{k}) = varargin{k+1};
end
end
