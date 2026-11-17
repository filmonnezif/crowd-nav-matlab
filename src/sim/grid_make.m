function G = grid_make(bounds, res)
%GRID_MAKE  Occupancy-grid geometry covering bounds [xmin xmax ymin ymax].
G.res = res;
G.x0 = bounds(1);
G.y0 = bounds(3);
G.nx = ceil((bounds(2) - bounds(1)) / res);
G.ny = ceil((bounds(4) - bounds(3)) / res);
[G.X, G.Y] = meshgrid(G.x0 + ((1:G.nx) - 0.5) * res, ...
                      G.y0 + ((1:G.ny) - 0.5) * res);
end
