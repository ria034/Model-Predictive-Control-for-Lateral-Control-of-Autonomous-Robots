function path = make_scenario(name)
%MAKE_SCENARIO Reference path, speed profile and obstacles for a test scenario.
%   'lane_change' : ISO 3888-1 style double lane change at a constant 20 m/s
%   's_curves'    : curvature changes; speed set by a 0.6 g lateral limit (12-19 m/s)
%   'hairpin_wet' : two tight bends on a wet road (mu = 0.5), 7-17 m/s
%   'obstacles'   : two-lane road at 15 m/s with three parked cars partly in the lane
obstacles = struct('s', {}, 'd', {}, 'r', {});
half = 3.5;    % road half-width (m)
mu = 1.0;
switch name
    case 'lane_change'
        xs = (-30:0.05:220)';
        sig = @(q) 1./(1 + exp(-q));
        ys = 3.5*(sig((xs - 50)/6.0) - sig((xs - 120)/6.0));
        path = finish_path(xs, ys);
        v = 20*ones(size(path.s)); v0 = 20;
    case 's_curves'
        xs = (0:0.05:400)';
        ys = 12*sin(2*pi*xs/120).*min(1, xs/60);
        path = finish_path(xs, ys);
        v = speed_profile(path, 25, 0.6*9.81, 2.5, 4.0, 12.0); v0 = v(1);
    case 'hairpin_wet'
        mu = 0.5;
        th1 = linspace(-pi/2, pi/2, 120)';
        th2 = linspace(3*pi/2, pi/2, 120)';
        P = [0 0; 60 0;
             60 + 18*cos(th1), 18 + 18*sin(th1);
             20 36;
             20 + 14*cos(th2), 50 + 14*sin(th2);
             80 64];
        t = [0; cumsum(hypot(diff(P(:,1)), diff(P(:,2))))];
        keep = [true; diff(t) > 0];          % drop repeated points
        P = P(keep,:); t = t(keep);
        tt = (0:0.05:t(end))';
        path = finish_path(interp1(t, P(:,1), tt), interp1(t, P(:,2), tt));
        v = speed_profile(path, 20, 0.8*mu*9.81, 2.0, 0.6*mu*9.81, 10.0); v0 = v(1);
    case 'obstacles'
        xs = (0:0.05:300)';
        ys = 4*sin(2*pi*xs/250);
        path = finish_path(xs, ys);
        v = 15*ones(size(path.s)); v0 = 15;
        obstacles = struct('s', {60, 130, 200}, 'd', {0.3, -0.4, 0.5}, 'r', {1.0, 1.2, 1.0});
    otherwise
        error('make_scenario:name', 'Unknown scenario "%s".', name);
end
path.v = v;
path.a = v .* gradient(v, path.ds);      % feedforward acceleration along the profile
path.half_width = half;
path.obstacles = obstacles;
path.mu = mu;
path.v0 = v0;
path.name = name;
end
