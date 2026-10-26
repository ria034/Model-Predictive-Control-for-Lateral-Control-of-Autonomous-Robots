function a = wrap_angle(a)
%WRAP_ANGLE Wrap to (-pi, pi].
a = atan2(sin(a), cos(a));
end
