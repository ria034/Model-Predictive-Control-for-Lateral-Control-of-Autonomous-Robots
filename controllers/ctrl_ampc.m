function [u, mem] = ctrl_ampc(meas, path, cfg, mem)
%CTRL_AMPC Adaptive (linear time-varying) MPC scheduled on the previewed speed.
[u, mem] = ctrl_linear_mpc(meas, path, cfg, mem, true);
end
