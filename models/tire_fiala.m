function Fy = tire_fiala(alpha, C, Fz, mu)
%TIRE_FIALA Fiala brush tyre: linear for small slip, saturates at mu*Fz.
%   alpha : slip angle (rad), C : cornering stiffness (N/rad),
%   Fz : normal load (N), mu : friction coefficient.
ta = tan(alpha);
th = C / (3*mu*Fz);
if abs(ta) < 1/th
    Fy = 3*mu*Fz*th*ta*(1 - abs(th*ta) + (th*ta)^2/3);
else
    Fy = mu*Fz*sign(alpha);
end
end
