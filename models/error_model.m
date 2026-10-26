function [A, B, E] = error_model(vx, p)
%ERROR_MODEL Linear lateral error dynamics (Rajamani, Vehicle Dynamics and Control, ch. 2).
%   state x = [e_y; e_y_dot; e_psi; e_psi_dot], input delta,
%   disturbance = desired yaw rate vx*kappa.
m = p.m; a = p.a; b = p.b; Iz = p.Iz; Cf = p.Cf; Cr = p.Cr;
vx = max(vx, 1.0);
A = [0  1                       0              0;
     0 -(Cf+Cr)/(m*vx)          (Cf+Cr)/m      (-Cf*a+Cr*b)/(m*vx);
     0  0                       0              1;
     0 -(Cf*a-Cr*b)/(Iz*vx)     (Cf*a-Cr*b)/Iz -(Cf*a^2+Cr*b^2)/(Iz*vx)];
B = [0; Cf/m; 0; Cf*a/Iz];
E = [0; -(Cf*a-Cr*b)/(m*vx) - vx; 0; -(Cf*a^2+Cr*b^2)/(Iz*vx)];
end
