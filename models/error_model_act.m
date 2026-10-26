function [A, B, E] = error_model_act(vx, p)
%ERROR_MODEL_ACT Error dynamics augmented with the first-order steering actuator.
%   state x = [e_y; e_y_dot; e_psi; e_psi_dot; delta], input = steering command.
[A4, B4, E4] = error_model(vx, p);
A = zeros(5);  A(1:4,1:4) = A4;  A(1:4,5) = B4;  A(5,5) = -1/p.tau_steer;
B = zeros(5,1);  B(5) = 1/p.tau_steer;
E = [E4; 0];
end
