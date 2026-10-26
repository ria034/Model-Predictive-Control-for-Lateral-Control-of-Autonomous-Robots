function res = run_scenario(cfg, scenario, controller)
%RUN_SCENARIO Closed-loop simulation of one controller on one scenario.
%   res = run_scenario(cfg, 'hairpin_wet', 'nmpc')
%   scenario   : 'lane_change' | 's_curves' | 'hairpin_wet' | 'obstacles'
%   controller : 'pure_pursuit' | 'stanley' | 'lqr' | 'lmpc' | 'ampc' | 'nmpc'
%   cfg.robust selects noise, model mismatch and steering delay.

path = make_scenario(scenario);
nom = cfg.vehicle; nom.mu = path.mu;        % what the controllers assume
plant = nom; rb = cfg.robust;                % what the "real" car is
if rb.mismatch
    plant.m = plant.m*rb.m_scale;
    plant.Iz = plant.Iz*rb.Iz_scale;
    plant.Cf = plant.Cf*rb.C_scale;
    plant.Cr = plant.Cr*rb.C_scale;
    plant.mu = plant.mu*rb.mu_scale;
end
cfg.nominal = nom;
rng(rb.seed);
ctrl = str2func(['ctrl_' controller]);
mem = struct();

dt = cfg.sim.Ts/round(cfg.sim.Ts/cfg.sim.dt);
Ts = cfg.sim.Ts; nsub = round(Ts/dt);
x = [path.x(1); path.y(1); path.psi(1); path.v0; 0; 0; 0];
buf = zeros(1, rb.delay_steps);
Nmax = ceil(cfg.sim.t_max/Ts) + 2;
names = {'t','X','Y','psi','vx','vy','r','delta','u','s','ey','epsi','vref','af','ar','ay','clear','comp'};
for q = 1:numel(names), L.(names{q}) = nan(Nmax, 1); end
L.pred_ey = nan(cfg.mpc.Np, Nmax);
status = 'finished'; i_true = 1; t = 0; n = 0;

% obstacle centres in world coordinates (for the clearance metric)
obs_xy = zeros(numel(path.obstacles), 3);
for q = 1:numel(path.obstacles)
    o = path.obstacles(q); j = path_index(path, o.s);
    obs_xy(q, :) = [path.x(j) - sin(path.psi(j))*o.d, path.y(j) + cos(path.psi(j))*o.d, o.r];
end

while true
    [i_true, s_true, ey_true] = project_to_path(path, x(1), x(2), i_true);
    if s_true >= path.s(end) - 2, break; end
    if abs(ey_true) > cfg.sim.fail_ey, status = 'left road'; break; end
    if t > cfg.sim.t_max, status = 'timeout'; break; end

    meas = struct('X', x(1), 'Y', x(2), 'psi', x(3), 'vx', x(4), 'vy', x(5), 'r', x(6), 'delta', x(7));
    if rb.noise
        meas.X = meas.X + randn*rb.sigma_pos;   meas.Y = meas.Y + randn*rb.sigma_pos;
        meas.psi = meas.psi + randn*rb.sigma_psi; meas.r = meas.r + randn*rb.sigma_r;
        meas.vy = meas.vy + randn*rb.sigma_vy;  meas.vx = meas.vx + randn*rb.sigma_vx;
    end
    tic;
    [u, mem] = ctrl(meas, path, cfg, mem);
    tc = toc;
    u = min(nom.delta_max, max(-nom.delta_max, u));
    buf = [buf, u]; dcmd = buf(1); buf(1) = [];       % steering command delay

    j = path_index(path, s_true);
    ax_cmd = cfg.sim.speed_kp*(path.v(j) - x(4)) + path.a(j);
    epsi = wrap_angle(x(3) - path.psi(i_true));
    [~, ~, af, ar] = axle_forces(x(4), x(5), x(6), x(7), plant);
    if isempty(obs_xy)
        clr = Inf;
    else
        clr = min(hypot(x(1) - obs_xy(:,1), x(2) - obs_xy(:,2)) - obs_xy(:,3) - nom.half_width);
    end

    n = n + 1;
    vals = {t, x(1), x(2), x(3), x(4), x(5), x(6), x(7), u, s_true, ey_true, epsi, path.v(j), af, ar, x(4)*x(6), clr, tc};
    for q = 1:numel(names), L.(names{q})(n) = vals{q}; end
    if isfield(mem, 'pred_ey'), L.pred_ey(:, n) = mem.pred_ey(:); end

    for q = 1:nsub
        x = plant_step_rk4(x, dcmd, ax_cmd, plant, dt);
    end
    t = t + Ts;
end
for q = 1:numel(names), L.(names{q}) = L.(names{q})(1:n); end
L.pred_ey = L.pred_ey(:, 1:n);

res.log = L;
res.status = status;
res.path = path;
res.plant = plant;
res.nominal = nom;
res.cfg = cfg;
res.scenario = scenario;
res.controller = controller;
res.metrics = compute_metrics(res);
end
