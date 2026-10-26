function x = plant_step_rk4(x, delta_cmd, ax_cmd, p, h)
%PLANT_STEP_RK4 One RK4 step of the vehicle model.
k1 = plant_deriv(x,          delta_cmd, ax_cmd, p);
k2 = plant_deriv(x + h/2*k1, delta_cmd, ax_cmd, p);
k3 = plant_deriv(x + h/2*k2, delta_cmd, ax_cmd, p);
k4 = plant_deriv(x + h*k3,   delta_cmd, ax_cmd, p);
x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
