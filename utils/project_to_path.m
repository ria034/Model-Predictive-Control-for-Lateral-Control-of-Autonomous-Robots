function [i, s, ey] = project_to_path(path, X, Y, i_hint)
%PROJECT_TO_PATH Nearest path sample (searched near i_hint), arc length and signed lateral error.
%   ey > 0 means the point is to the left of the path.
n = numel(path.s);
lo = max(1, i_hint - 300);
hi = min(n, i_hint + 300);
d2 = (path.x(lo:hi) - X).^2 + (path.y(lo:hi) - Y).^2;
[~, k] = min(d2);
i = lo + k - 1;
ps = path.psi(i);
ex = X - path.x(i); eyw = Y - path.y(i);
ey = -sin(ps)*ex + cos(ps)*eyw;
s = path.s(i) + cos(ps)*ex + sin(ps)*eyw;
end
