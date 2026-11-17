function crowd = crowd_init(S, cfg)
%CROWD_INIT  Reactive Social Force crowd built on a recorded scene.
%   crowd = crowd_init(S, cfg)
%
%   Every person keeps the route they walked in the recording: at each
%   moment they head for where the recording has them crowdLook seconds
%   later. On top of that, Social Force (Helbing & Molnar 1995) pushes
%   them away from the robot and from each other, stronger for things in
%   front of them than behind. So people step around the robot and then
%   rejoin their route, instead of walking through it.
%   Parameters come from cfg (sim_config): crowdLook, crowdTau,
%   crowdAr/crowdBr (robot), crowdAp/crowdBp (people), crowdLambda,
%   crowdVmax.
crowd.S0 = S;
crowd.ids = zeros(0, 1);
crowd.P = zeros(0, 2);
crowd.V = zeros(0, 2);
crowd.t = -inf;
% per-person track: tracks{i} = [k x y], sorted by k, for ids uid(i)
[uid, ~, g] = unique(S.id);
crowd.uid = uid;
crowd.tracks = cell(numel(uid), 1);
for i = 1:numel(uid)
    r = find(g == i);
    [kk, o] = sort(S.k(r));
    crowd.tracks{i} = [kk, S.xy(r(o), :)];
end
crowd.cfg = cfg;
end
