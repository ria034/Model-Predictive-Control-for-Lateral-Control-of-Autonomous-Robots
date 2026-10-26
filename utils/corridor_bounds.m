function [lo, hi, ref] = corridor_bounds(path, s, p, use_obstacles)
%CORRIDOR_BOUNDS Allowed lateral offset of the vehicle centre at arc length s.
%   lo, hi : bounds from the road edges and, next to an obstacle, the
%            obstacle (hard, softened by the MPC slack variable)
%   ref    : smooth target offset that ramps in over 20 m before an
%            obstacle and back out over 15 m after it
hw = p.half_width;
road = path.half_width - hw;
lo = -road; hi = road; ref = 0;
if nargin < 4 || ~use_obstacles, return; end
for k = 1:numel(path.obstacles)
    o = path.obstacles(k);
    s0 = o.s - o.r - 3.0;
    s1 = o.s + o.r + 3.0;
    if o.d >= 0          % obstacle on the left half: pass on the right
        bound = o.d - o.r - hw - 0.3;  target = max(-road, bound - 0.4);
    else                 % obstacle on the right half: pass on the left
        bound = o.d + o.r + hw + 0.3;  target = min(road, bound + 0.4);
    end
    if s >= s0 && s <= s1
        if o.d >= 0, hi = min(hi, bound); else, lo = max(lo, bound); end
    end
    if s > s0 - 20 && s < s0
        w = 0.5 - 0.5*cos(pi*(s - (s0 - 20))/20);
    elseif s >= s0 && s <= s1
        w = 1;
    elseif s > s1 && s < s1 + 15
        w = 0.5 + 0.5*cos(pi*(s - s1)/15);
    else
        w = 0;
    end
    if abs(w*target) > abs(ref), ref = w*target; end
end
end
