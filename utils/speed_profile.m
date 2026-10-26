function v = speed_profile(path, v_max, ay_max, a_acc, a_dec, v0)
%SPEED_PROFILE Speed limited by lateral acceleration, then by accel/decel limits.
v = min(v_max, sqrt(ay_max ./ max(abs(path.kappa), 1e-6)));
v(1) = min(v(1), v0);
ds = path.ds;
for i = 2:numel(v)
    v(i) = min(v(i), sqrt(v(i-1)^2 + 2*a_acc*ds));
end
for i = numel(v)-1:-1:1
    v(i) = min(v(i), sqrt(v(i+1)^2 + 2*a_dec*ds));
end
end
