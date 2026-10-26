function path = finish_path(x, y, ds)
%FINISH_PATH Resample a polyline by arc length and add heading and curvature.
if nargin < 3, ds = 0.1; end
x = x(:); y = y(:);
d = hypot(diff(x), diff(y));
s = [0; cumsum(d)];
S = (0:ds:s(end) - ds)';
path.s = S;
path.x = interp1(s, x, S);
path.y = interp1(s, y, S);
path.psi = unwrap(atan2(gradient(path.y), gradient(path.x)));
kap = gradient(path.psi, ds);
k = 15;                                   % moving-average smoothing of curvature
kp = [repmat(kap(1), k, 1); kap; repmat(kap(end), k, 1)];
path.kappa = conv(kp, ones(2*k+1, 1)/(2*k+1), 'valid');
path.ds = ds;
end
