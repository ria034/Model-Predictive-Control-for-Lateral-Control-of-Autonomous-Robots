function p = vehicle_params(preset)
%VEHICLE_PARAMS Single-track vehicle parameters.
%   p = vehicle_params('sedan') returns a mid-size sedan (default).
%   p = vehicle_params('repo') returns the values from the original
%   Simulink model's Vehicle Body 3DOF block (very soft tyres: the
%   effective axle stiffnesses below are NF*Cy*Fz/Fznom from that block).

if nargin < 1, preset = 'sedan'; end
switch lower(preset)
    case 'repo'
        p.m = 2000;  p.Iz = 4000;  p.a = 1.4;  p.b = 1.6;
        p.Cf = 25114; p.Cr = 20143;          % N/rad, whole axle
    case 'sedan'
        p.m = 1600;  p.Iz = 2500;  p.a = 1.2;  p.b = 1.6;
        p.Cf = 110e3; p.Cr = 130e3;          % N/rad, whole axle (understeering)
    otherwise
        error('vehicle_params:preset', 'Unknown preset "%s". Use ''sedan'' or ''repo''.', preset);
end
p.mu = 1.0;               % road friction (overwritten per scenario)
p.g = 9.81;
p.delta_max = 0.5;        % rad, road-wheel steering limit (same as the original MPC)
p.delta_rate_max = 0.8;   % rad/s, steering actuator rate limit
p.tau_steer = 0.05;       % s, steering actuator time constant
p.ax_max = 3.0;           % m/s^2
p.ax_min = -8.0;          % m/s^2
p.half_width = 0.9;       % m, half the vehicle width (for corridor and clearance)
p.L = p.a + p.b;
end
