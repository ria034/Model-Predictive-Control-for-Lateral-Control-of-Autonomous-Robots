function [xl, yl, xr, yr] = road_edges(path)
%ROAD_EDGES Left and right road edge polylines.
n = path.half_width;
xl = path.x - sin(path.psi)*n;  yl = path.y + cos(path.psi)*n;
xr = path.x + sin(path.psi)*n;  yr = path.y - cos(path.psi)*n;
end
