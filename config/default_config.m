function cfg = default_config()
%DEFAULT_CONFIG All settings for scenarios, controllers and robustness tests.
%   Edit a copy of the returned struct rather than this file, e.g.
%       cfg = default_config();
%       cfg.robust.delay_steps = 2;
%       res = run_scenario(cfg, 'hairpin_wet', 'nmpc');

cfg.vehicle = vehicle_params('sedan');

% Simulation
cfg.sim.dt = 0.002;        % s, plant integration step (RK4)
cfg.sim.Ts = 0.05;         % s, controller sample time
cfg.sim.t_max = 60;        % s, abort after this
cfg.sim.fail_ey = 6.0;     % m, abort when this far from the path ("left road")
cfg.sim.speed_kp = 2.0;    % 1/s, speed controller gain (plus profile feedforward)

% Robustness tests (all off by default)
cfg.robust.noise = false;
cfg.robust.sigma_pos = 0.10;          % m, GNSS position noise (1 sigma)
cfg.robust.sigma_psi = 0.3*pi/180;    % rad, heading noise
cfg.robust.sigma_r = 0.01;            % rad/s, yaw-rate noise
cfg.robust.sigma_vy = 0.05;           % m/s, lateral-velocity estimate noise
cfg.robust.sigma_vx = 0.05;           % m/s, speed noise
cfg.robust.mismatch = false;          % plant differs from the controllers' model
cfg.robust.m_scale = 1.2;             % +20 % mass (passengers, luggage)
cfg.robust.Iz_scale = 1.2;
cfg.robust.C_scale = 0.8;             % -20 % cornering stiffness (worn tyres)
cfg.robust.mu_scale = 1.0;            % friction relative to what the controllers assume
cfg.robust.delay_steps = 0;           % steering command delay, in controller samples
cfg.robust.compensate_delay = true;   % model-based controllers predict over the delay
cfg.robust.seed = 1;

% Pure pursuit
cfg.pp.k_ld = 0.6;  cfg.pp.L0 = 3.0;  cfg.pp.Lmin = 3.0;  cfg.pp.Lmax = 25.0;

% Stanley
cfg.stanley.k = 2.0;  cfg.stanley.k_soft = 1.0;  cfg.stanley.k_yaw = 0.2;

% LQR (gain-scheduled on speed, with steady-state feedforward)
cfg.lqr.Q = [1 0 1 0];  cfg.lqr.R = 10;

% MPC (shared by lmpc, ampc and nmpc)
cfg.mpc.Np = 40;             % prediction horizon (2 s at Ts = 0.05)
cfg.mpc.Nc = 10;             % control horizon (input held afterwards)
cfg.mpc.q_ey = 1.0;          % lateral error weight
cfg.mpc.q_epsi = 1.0;        % heading error weight
cfg.mpc.r_ddelta = 20.0;     % steering-change weight
cfg.mpc.r_delta = 0.0;       % steering magnitude weight
cfg.mpc.w_slack = 1e4;       % quadratic penalty on corridor violation
cfg.mpc.rho_slack = 1e3;     % linear penalty on corridor violation
cfg.mpc.v_design = 15.0;     % m/s, fixed model speed for the linear (non-adaptive) MPC
cfg.mpc.use_obstacles = true;

% Nonlinear MPC (real-time iteration)
cfg.nmpc.iters = 1;          % SQP iterations per sample (1 = RTI)
cfg.nmpc.fd_eps = 1e-5;      % finite-difference step for the Jacobians
cfg.nmpc.substeps = 2;       % RK4 substeps per sample in the prediction model
end
