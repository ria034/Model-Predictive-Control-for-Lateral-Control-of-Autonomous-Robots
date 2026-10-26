function [u, mem] = ctrl_lmpc(meas, path, cfg, mem)
%CTRL_LMPC Linear MPC with a single model at the design speed.
[u, mem] = ctrl_linear_mpc(meas, path, cfg, mem, false);
end
