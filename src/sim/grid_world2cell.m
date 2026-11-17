function rc = grid_world2cell(G, p)
%GRID_WORLD2CELL  World [x y] (n x 2) -> grid [row col], clamped to the map.
c = floor((p(:, 1) - G.x0) / G.res) + 1;
r = floor((p(:, 2) - G.y0) / G.res) + 1;
rc = [min(max(r, 1), G.ny), min(max(c, 1), G.nx)];
end
