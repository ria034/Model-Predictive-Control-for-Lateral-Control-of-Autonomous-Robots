"""Python mirror of the MATLAB project, used only to verify the algorithms."""
import numpy as np
from math import *
from scipy.linalg import expm

# ---------------- config ----------------
def vehicle_params(preset='sedan'):
    if preset == 'repo':   # MATLAB Vehicle Body 3DOF block values from the original Simulink model
        p = dict(m=2000., Iz=4000., a=1.4, b=1.6, Cf=25114., Cr=20143.)
    else:                  # mid-size sedan
        p = dict(m=1600., Iz=2500., a=1.2, b=1.6, Cf=110e3, Cr=130e3)
    p.update(mu=1.0, g=9.81, delta_max=0.5, delta_rate_max=0.8, tau_steer=0.05,
             ax_max=3.0, ax_min=-8.0, half_width=0.9)
    p['L'] = p['a'] + p['b']
    return p

def default_config():
    return dict(
        vehicle=vehicle_params('sedan'),
        sim=dict(dt=0.002, Ts=0.05, t_max=60., fail_ey=6.0, speed_kp=2.0),
        robust=dict(noise=False, sigma_pos=0.10, sigma_psi=0.3*pi/180, sigma_r=0.01, sigma_vy=0.05, sigma_vx=0.05,
                    mismatch=False, m_scale=1.2, Iz_scale=1.2, C_scale=0.8, mu_scale=1.0,
                    delay_steps=0, compensate_delay=True, seed=1),
        pp=dict(k_ld=0.6, L0=3.0, Lmin=3.0, Lmax=25.0),
        stanley=dict(k=2.0, k_soft=1.0, k_yaw=0.2),
        lqr=dict(Q=[1.0, 0.0, 1.0, 0.0], R=10.0),
        mpc=dict(Np=40, Nc=10, q_ey=1.0, q_epsi=1.0, r_ddelta=20.0, r_delta=0.0, w_slack=1e4, rho_slack=1e3,
                 v_design=15.0, use_obstacles=True),
        nmpc=dict(iters=1, fd_eps=1e-5),
    )

# ---------------- tyres and plant ----------------
def fiala(alpha, C, Fz, mu):
    ta = tan(alpha); th = C/(3*mu*Fz)
    if abs(ta) < 1/th:
        return 3*mu*Fz*th*ta*(1 - abs(th*ta) + (th*ta)**2/3)
    return mu*Fz*np.sign(alpha)

def axle_forces(vx, vy, r, delta, p):
    vxs = max(vx, 1.0)
    af = delta - atan2(vy + p['a']*r, vxs); ar = -atan2(vy - p['b']*r, vxs)
    Fzf = p['m']*p['g']*p['b']/p['L']; Fzr = p['m']*p['g']*p['a']/p['L']
    return fiala(af, p['Cf'], Fzf, p['mu']), fiala(ar, p['Cr'], Fzr, p['mu']), af, ar

def plant_deriv(x, delta_cmd, ax_cmd, p):
    X, Y, psi, vx, vy, r, delta = x
    Fyf, Fyr, _, _ = axle_forces(vx, vy, r, delta, p)
    vxs = max(vx, 1.0)
    ddelta = min(p['delta_rate_max'], max(-p['delta_rate_max'], (delta_cmd - delta)/p['tau_steer']))
    ax = min(p['ax_max'], max(p['ax_min'], ax_cmd))
    return np.array([vx*cos(psi) - vy*sin(psi), vx*sin(psi) + vy*cos(psi), r,
                     ax + vy*r - Fyf*sin(delta)/p['m'],
                     (Fyf*cos(delta) + Fyr)/p['m'] - vxs*r,
                     (p['a']*Fyf*cos(delta) - p['b']*Fyr)/p['Iz'], ddelta])

def rk4(x, dc, ax, p, h):
    k1 = plant_deriv(x, dc, ax, p); k2 = plant_deriv(x+h/2*k1, dc, ax, p)
    k3 = plant_deriv(x+h/2*k2, dc, ax, p); k4 = plant_deriv(x+h*k3, dc, ax, p)
    return x + h/6*(k1+2*k2+2*k3+k4)

# ---------------- paths ----------------
def finish_path(x, y, ds=0.1):
    d = np.hypot(np.diff(x), np.diff(y)); s = np.concatenate([[0], np.cumsum(d)])
    S = np.arange(0, s[-1], ds); X = np.interp(S, s, x); Y = np.interp(S, s, y)
    psi = np.unwrap(np.arctan2(np.gradient(Y), np.gradient(X)))
    kap = np.gradient(psi, ds)
    k = 15; kap = np.convolve(np.pad(kap, k, mode='edge'), np.ones(2*k+1)/(2*k+1), mode='valid')
    return dict(s=S, x=X, y=Y, psi=psi, kappa=kap, ds=ds)

def speed_profile(path, v_max, ay_max, a_acc, a_dec, v0):
    v = np.minimum(v_max, np.sqrt(ay_max/np.maximum(np.abs(path['kappa']), 1e-6)))
    v[0] = min(v[0], v0); ds = path['ds']
    for i in range(1, len(v)): v[i] = min(v[i], sqrt(v[i-1]**2 + 2*a_acc*ds))
    for i in range(len(v)-2, -1, -1): v[i] = min(v[i], sqrt(v[i+1]**2 + 2*a_dec*ds))
    return v

def make_scenario(name):
    obs = []; half = 3.5; mu = 1.0
    if name == 'lane_change':      # ISO 3888-1 style double lane change at 20 m/s
        xs = np.arange(-30, 220, 0.05); sig = lambda q: 1/(1+np.exp(-q))
        ys = 3.5*(sig((xs-50)/6.0) - sig((xs-120)/6.0))
        path = finish_path(xs, ys); v = np.full_like(path['s'], 20.0); v0 = 20.0
    elif name == 's_curves':       # curvature changes, speed limited by lateral acceleration
        xs = np.arange(0, 400, 0.05); ys = 12*np.sin(2*pi*xs/120)*np.minimum(1, xs/60)
        path = finish_path(xs, ys); v = speed_profile(path, 25, 0.6*9.81, 2.5, 4.0, 12.0); v0 = v[0]
    elif name == 'hairpin_wet':    # tight bends on a wet road (mu = 0.5)
        mu = 0.5
        pts = [(0,0),(60,0)]; th = np.linspace(-pi/2, pi/2, 120); pts += list(zip(60+18*np.cos(th), 18+18*np.sin(th)))
        pts += [(20,36)]; th = np.linspace(3*pi/2, pi/2, 120); pts += list(zip(20+14*np.cos(th), 50+14*np.sin(th)))
        pts += [(80,64)]
        P = np.array(pts); t = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
        tt = np.arange(0, t[-1], 0.05); xs = np.interp(tt, t, P[:,0]); ys = np.interp(tt, t, P[:,1])
        path = finish_path(xs, ys); v = speed_profile(path, 20, 0.8*mu*9.81, 2.0, 0.6*mu*9.81, 10.0); v0 = v[0]
    elif name == 'obstacles':      # two-lane road with parked cars in the way
        xs = np.arange(0, 300, 0.05); ys = 4*np.sin(2*pi*xs/250)
        path = finish_path(xs, ys); v = np.full_like(path['s'], 15.0); v0 = 15.0
        obs = [dict(s=60., d=0.3, r=1.0), dict(s=130., d=-0.4, r=1.2), dict(s=200., d=0.5, r=1.0)]
    path['v'] = v; path['a'] = v*np.gradient(v, path['ds']); path['half_width'] = half; path['obstacles'] = obs; path['mu'] = mu; path['v0'] = v0; path['name'] = name
    return path

def path_point(path, s):
    i = int(np.clip(round(s/path['ds']), 0, len(path['s'])-1))
    return i

def project(path, X, Y, i_hint):
    lo = max(0, i_hint-300); hi = min(len(path['s']), i_hint+300)
    d2 = (path['x'][lo:hi]-X)**2 + (path['y'][lo:hi]-Y)**2; i = lo + int(np.argmin(d2))
    ps = path['psi'][i]; ex, ey = X - path['x'][i], Y - path['y'][i]
    e_y = -sin(ps)*ex + cos(ps)*ey; s = path['s'][i] + cos(ps)*ex + sin(ps)*ey
    return i, s, e_y

def corridor(path, s, p):
    """Lateral bounds [lo, hi] for the vehicle centre at arc length s, and a smooth target offset.
    Bounds are hard (softened by the MPC slack) next to an obstacle; the target ramps in over
    20 m before it and back out over 15 m after it."""
    hw = p['half_width']; road = path['half_width'] - hw
    lo, hi, ref = -road, road, 0.0
    for o in path['obstacles']:
        s0, s1 = o['s'] - o['r'] - 3.0, o['s'] + o['r'] + 3.0
        if o['d'] >= 0:
            bound = o['d'] - o['r'] - hw - 0.3; target = max(-road, bound - 0.4)
        else:
            bound = o['d'] + o['r'] + hw + 0.3; target = min(road, bound + 0.4)
        if s0 <= s <= s1:
            if o['d'] >= 0: hi = min(hi, bound)
            else: lo = max(lo, bound)
        if s0 - 20.0 < s < s0: w = 0.5 - 0.5*cos(pi*(s - (s0 - 20.0))/20.0)
        elif s0 <= s <= s1: w = 1.0
        elif s1 < s < s1 + 15.0: w = 0.5 + 0.5*cos(pi*(s - s1)/15.0)
        else: w = 0.0
        if abs(w*target) > abs(ref): ref = w*target
    return lo, hi, ref

# ---------------- linear algebra helpers ----------------
def c2d(A, B, Ts):
    n, m = A.shape[0], B.shape[1]; M = np.zeros((n+m, n+m)); M[:n,:n] = A*Ts; M[:n,n:] = B*Ts
    E = expm(M); return E[:n,:n], E[:n,n:]

def error_model(vx, p):
    m, a, b, Iz, Cf, Cr = p['m'], p['a'], p['b'], p['Iz'], p['Cf'], p['Cr']; vx = max(vx, 1.0)
    A = np.array([[0,1,0,0],
                  [0,-(Cf+Cr)/(m*vx),(Cf+Cr)/m,(-Cf*a+Cr*b)/(m*vx)],
                  [0,0,0,1],
                  [0,-(Cf*a-Cr*b)/(Iz*vx),(Cf*a-Cr*b)/Iz,-(Cf*a*a+Cr*b*b)/(Iz*vx)]])
    B = np.array([[0],[Cf/m],[0],[Cf*a/Iz]])
    E = np.array([[0],[-(Cf*a-Cr*b)/(m*vx)-vx],[0],[-(Cf*a*a+Cr*b*b)/(Iz*vx)]])
    return A, B, E

def error_model_act(vx, p):
    A, B, E = error_model(vx, p); tau = p['tau_steer']
    Aa = np.zeros((5, 5)); Aa[:4,:4] = A; Aa[:4, 4] = B[:, 0]; Aa[4, 4] = -1/tau
    Ba = np.zeros((5, 1)); Ba[4, 0] = 1/tau
    Ea = np.zeros((5, 1)); Ea[:4, 0] = E[:, 0]
    return Aa, Ba, Ea

def dlqr(A, B, Q, R, iters=500):
    P = Q.copy()
    for _ in range(iters):
        K = np.linalg.solve(R + B.T@P@B, B.T@P@A); Pn = Q + A.T@P@(A - B@K)
        if np.max(np.abs(Pn-P)) < 1e-9*max(1, np.max(np.abs(P))): P = Pn; break
        P = Pn
    return np.linalg.solve(R + B.T@P@B, B.T@P@A)

def qp_ip(H, f, G, h, iters=50, tol=1e-8):
    """Primal-dual interior point (Mehrotra) for min 1/2 z'Hz + f'z s.t. Gz <= h."""
    n = len(f); mI = len(h)
    z = np.zeros(n); s = np.maximum(h - G@z, 1.0); lam = np.ones(mI)
    for _ in range(iters):
        rd = H@z + f + G.T@lam; rp = G@z + s - h; mu = s@lam/mI
        if np.linalg.norm(rd) < tol and np.linalg.norm(rp) < tol and mu < tol: break
        W = lam/s
        M = H + G.T@(W[:,None]*G) + 1e-10*np.eye(n)
        def solve(rc):  # rc: complementarity residual
            rhs = -rd - G.T@((lam*rp - rc)/s)
            dz = np.linalg.solve(M, rhs); ds = -rp - G@dz; dl = (-rc - lam*ds)/s
            return dz, ds, dl
        dz, ds, dl = solve(s*lam)
        def step(v, dv):
            idx = dv < 0
            return min(1.0, np.min(-v[idx]/dv[idx])) if np.any(idx) else 1.0
        a = min(step(s, ds), step(lam, dl)); mu_aff = (s+a*ds)@(lam+a*dl)/mI; sig = (mu_aff/mu)**3
        dz, ds, dl = solve(s*lam + ds*dl - sig*mu)
        a = 0.99*min(step(s, ds), step(lam, dl))
        z += a*dz; s = np.maximum(s + a*ds, 1e-14); lam = np.maximum(lam + a*dl, 1e-14)
    return z

# ---------------- controllers ----------------
def ctrl_pure_pursuit(meas, path, cfg, mem):
    p = cfg['vehicle']; c = cfg['pp']
    Xr = meas['X'] - p['b']*cos(meas['psi']); Yr = meas['Y'] - p['b']*sin(meas['psi'])   # rear axle
    i, s, _ = project(path, Xr, Yr, mem.get('i', 0)); mem['i'] = i
    Ld = min(c['Lmax'], max(c['Lmin'], c['k_ld']*meas['vx'] + c['L0']))
    j = path_point(path, s + Ld); gx, gy = path['x'][j], path['y'][j]
    alpha = atan2(gy - Yr, gx - Xr) - meas['psi']
    Ld_act = max(1.0, hypot(gx-Xr, gy-Yr))
    return atan(2*p['L']*sin(alpha)/Ld_act), mem

def ctrl_stanley(meas, path, cfg, mem):
    p = cfg['vehicle']; c = cfg['stanley']
    Xf = meas['X'] + p['a']*cos(meas['psi']); Yf = meas['Y'] + p['a']*sin(meas['psi'])  # front axle
    i, s, ey = project(path, Xf, Yf, mem.get('i', 0)); mem['i'] = i
    epsi = atan2(sin(meas['psi'] - path['psi'][i]), cos(meas['psi'] - path['psi'][i]))
    r_ref = meas['vx']*path['kappa'][i]
    return -epsi - atan(c['k']*ey/(c['k_soft'] + meas['vx'])) - c['k_yaw']*(meas['r'] - r_ref), mem

def error_state(meas, path, mem):
    i, s, ey = project(path, meas['X'], meas['Y'], mem.get('i', 0)); mem['i'] = i
    epsi = atan2(sin(meas['psi'] - path['psi'][i]), cos(meas['psi'] - path['psi'][i]))
    kap = path['kappa'][i]
    x = np.array([ey, meas['vy'] + meas['vx']*epsi, epsi, meas['r'] - meas['vx']*kap])
    return x, s, kap

def compensate(x, mem, cfg, vx, kap):
    """Roll the linear error model forward over the commands still waiting in the delay buffer."""
    if not cfg['robust']['compensate_delay'] or cfg['robust']['delay_steps'] == 0: return x
    A, B, E = error_model(vx, cfg['nominal']); Ad, Bd = c2d(A, np.hstack([B, E]), cfg['sim']['Ts'])
    for u in mem.get('pending', []): x = Ad@x + Bd[:,0]*u + Bd[:,1]*vx*kap
    return x

def push_pending(mem, u, cfg):
    d = cfg['robust']['delay_steps']
    if d == 0: return
    q = mem.setdefault('pending', [])
    q.append(u)
    while len(q) > d: q.pop(0)

def ctrl_lqr(meas, path, cfg, mem):
    p = cfg['nominal']; c = cfg['lqr']
    x, s, kap = error_state(meas, path, mem)
    vx = max(meas['vx'], 1.0); x = compensate(x, mem, cfg, vx, kap)
    A, B, E = error_model(vx, p); Ad, Bd = c2d(A, B, cfg['sim']['Ts'])
    K = dlqr(Ad, Bd, np.diag(c['Q']), np.array([[c['R']]]))[0]
    # steady-state feedforward (Rajamani 3.2)
    Kv = p['m']*(p['b']*p['Cr'] - p['a']*p['Cf'])/(p['L']*p['Cf']*p['Cr'])
    dff = p['L']*kap + Kv*vx*vx*kap - K[2]*(p['b']*kap - p['a']*p['m']*vx*vx*kap/(p['Cr']*p['L']))
    u = -K@x + dff
    u = float(np.clip(u, -p['delta_max'], p['delta_max'])); push_pending(mem, u, cfg)
    return u, mem

def lane_bounds(path, s, p, use_obstacles):
    if use_obstacles: return corridor(path, s, p)
    return -path['half_width'] + p['half_width'], path['half_width'] - p['half_width'], 0.0

def horizon_refs(path, s, vx_now, cfg, adaptive):
    c = cfg['mpc']; Ts = cfg['sim']['Ts']; p = cfg['nominal']
    ss = s; A_l = []; B_l = []; w_l = []; lo = []; hi = []; refs = []
    for k in range(c['Np']):
        i = path_point(path, ss)
        vk = max(path['v'][i], 1.0) if adaptive else c['v_design']
        A, B, E = error_model_act(vk, p); Ad, Bd = c2d(A, np.hstack([B, E]), Ts)
        A_l.append(Ad); B_l.append(Bd[:, :1]); w_l.append(Bd[:, 1]*vk*path['kappa'][i])
        ss += (path['v'][i] if adaptive else vx_now)*Ts
        l, h, rf = lane_bounds(path, ss, p, c['use_obstacles'])
        lo.append(l); hi.append(h); refs.append(rf)
    return A_l, B_l, w_l, lo, hi, refs

def ctrl_mpc(meas, path, cfg, mem, adaptive):
    p = cfg['nominal']
    x, s, kap = error_state(meas, path, mem)
    vx = max(meas['vx'], 1.0)
    A_l, B_l, w_l, lo, hi, refs = horizon_refs(path, s, vx, cfg, adaptive)
    u_prev = mem.get('u', 0.0)
    x = np.append(x, meas['delta'])
    if cfg['robust']['compensate_delay'] and cfg['robust']['delay_steps'] > 0:
        A, B, E = error_model_act(vx, p); Ad, Bd = c2d(A, np.hstack([B, E]), cfg['sim']['Ts'])
        for uq in mem.get('pending', []): x = Ad@x + Bd[:,0]*uq + Bd[:,1]*vx*kap
    U, eps, pred = mpc_qp_general(x, u_prev, A_l, B_l, w_l, lo, hi, cfg, p, [c_q(cfg)[0], 0, c_q(cfg)[1], 0, 0], 0, refs)
    u = float(U[0]); mem['u'] = u; mem['pred'] = pred[:, 0]; push_pending(mem, u, cfg)
    return u, mem

def c_q(cfg): return cfg['mpc']['q_ey'], cfg['mpc']['q_epsi']

def ctrl_lmpc(meas, path, cfg, mem): return ctrl_mpc(meas, path, cfg, mem, adaptive=False)
def ctrl_ampc(meas, path, cfg, mem): return ctrl_mpc(meas, path, cfg, mem, adaptive=True)

# ----- nonlinear MPC (real-time iteration: linearize the nonlinear model along the last solution, solve one QP) -----
def nl_path_deriv(xi, u, vx, kap, p):
    ey, epsi, vy, r, delta = xi
    Fyf, Fyr, _, _ = axle_forces(vx, vy, r, delta, p)
    sdot = (vx*cos(epsi) - vy*sin(epsi))/(1 - kap*ey)
    return np.array([vx*sin(epsi) + vy*cos(epsi), r - kap*sdot,
                     (Fyf*cos(delta) + Fyr)/p['m'] - vx*r, (p['a']*Fyf*cos(delta) - p['b']*Fyr)/p['Iz'],
                     (u - delta)/p['tau_steer']])

def nl_step(xi, delta, vx, kap, p, Ts, sub=2):
    h = Ts/sub
    for _ in range(sub):
        k1 = nl_path_deriv(xi, delta, vx, kap, p); k2 = nl_path_deriv(xi+h/2*k1, delta, vx, kap, p)
        k3 = nl_path_deriv(xi+h/2*k2, delta, vx, kap, p); k4 = nl_path_deriv(xi+h*k3, delta, vx, kap, p)
        xi = xi + h/6*(k1+2*k2+2*k3+k4)
    return xi

def ctrl_nmpc(meas, path, cfg, mem):
    p = cfg['nominal']; c = cfg['mpc']; Np, Nc = c['Np'], c['Nc']; Ts = cfg['sim']['Ts']; eps = cfg['nmpc']['fd_eps']
    i, s, ey = project(path, meas['X'], meas['Y'], mem.get('i', 0)); mem['i'] = i
    epsi = atan2(sin(meas['psi'] - path['psi'][i]), cos(meas['psi'] - path['psi'][i]))
    xi0 = np.array([ey, epsi, meas['vy'], meas['r'], meas['delta']]); vx = max(meas['vx'], 1.0)
    if cfg['robust']['compensate_delay']:
        for u in mem.get('pending', []): xi0 = nl_step(xi0, u, vx, path['kappa'][i], p, Ts)
    # horizon speed, curvature, corridor
    ss = s; vks = []; kks = []; lo = []; hi = []; refs = []
    for k in range(Np):
        j = path_point(path, ss); vks.append(max(path['v'][j], 1.0)); kks.append(path['kappa'][j]); ss += path['v'][j]*Ts
        l, h, rf = lane_bounds(path, ss, p, c['use_obstacles'])
        lo.append(l); hi.append(h); refs.append(rf)
    Ug = mem.get('U', np.zeros(Nc)); Ug = np.concatenate([Ug[1:], Ug[-1:]])   # shifted guess
    u_prev = mem.get('u', 0.0)
    for _ in range(cfg['nmpc']['iters']):
        # rollout along guess and linearize
        xg = xi0.copy(); A_l = []; B_l = []; w_l = []
        for k in range(Np):
            uk = Ug[min(k, Nc-1)]
            fx = nl_step(xg, uk, vks[k], kks[k], p, Ts)
            A = np.zeros((5, 5))
            for q in range(5):
                d = np.zeros(5); d[q] = eps; A[:, q] = (nl_step(xg+d, uk, vks[k], kks[k], p, Ts) - fx)/eps
            Bv = (nl_step(xg, uk+eps, vks[k], kks[k], p, Ts) - fx)/eps
            A_l.append(A); B_l.append(Bv[:, None]); w_l.append(fx - A@xg - Bv*uk)
            xg = fx
        # LTV QP in the full state (ey, epsi, vy, r): reuse mpc_qp with a reordered weight
        U, sl, pred = mpc_qp_general(xi0, u_prev, A_l, B_l, w_l, lo, hi, cfg, p, [cfg['mpc']['q_ey'], cfg['mpc']['q_epsi'], 0, 0, 0], 0, refs)
        Ug = U
    u = float(Ug[0]); mem['U'] = Ug; mem['u'] = u; mem['pred'] = pred[:, 0]; push_pending(mem, u, cfg)
    return u, mem

def mpc_qp_general(x0, u_prev, A_list, B_list, w_list, lo, hi, cfg, p, Qdiag, iy, refs):
    c = cfg['mpc']; Np, Nc = c['Np'], c['Nc']; Ts = cfg['sim']['Ts']; n = len(x0)
    Sx = np.zeros((Np, n)); Su = np.zeros((Np, n, Nc)); xf = x0.copy(); Phi = np.zeros((n, Nc))
    for k in range(Np):
        j = min(k, Nc-1); xf = A_list[k]@xf + w_list[k]; Phi = A_list[k]@Phi; Phi[:, j] += B_list[k][:, 0]
        Sx[k] = xf; Su[k] = Phi
    nz = Nc+1; H = np.zeros((nz, nz)); f = np.zeros(nz); Q = np.diag(Qdiag)
    for k in range(Np):
        dev = Sx[k].copy(); dev[iy] -= refs[k]
        H[:Nc,:Nc] += Su[k].T@Q@Su[k]; f[:Nc] += Su[k].T@Q@dev
    D = np.eye(Nc) - np.eye(Nc, k=-1)
    H[:Nc,:Nc] += c['r_ddelta']*D.T@D + c['r_delta']*np.eye(Nc); f[0] -= c['r_ddelta']*u_prev
    H[Nc, Nc] = c['w_slack']; f[Nc] = c['rho_slack']; H += 1e-8*np.eye(nz)
    G = []; hv = []; dmax = p['delta_max']; dr = p['delta_rate_max']*Ts
    for i in range(Nc):
        e = np.zeros(nz); e[i] = 1; G += [e, -e]; hv += [dmax, dmax]
        g = np.zeros(nz); g[i] = 1
        if i > 0: g[i-1] = -1; G += [g, -g]; hv += [dr, dr]
        else: G += [g, -g]; hv += [dr + u_prev, dr - u_prev]
    for k in range(Np):
        row = np.zeros(nz); row[:Nc] = Su[k][iy]; row[Nc] = -1; G.append(row); hv.append(hi[k] - Sx[k][iy])
        row2 = np.zeros(nz); row2[:Nc] = -Su[k][iy]; row2[Nc] = -1; G.append(row2); hv.append(Sx[k][iy] - lo[k])
    e = np.zeros(nz); e[Nc] = -1; G.append(e); hv.append(0.0)
    z = qp_ip(H, f, np.array(G), np.array(hv))
    pred = np.array([Sx[k] + Su[k]@z[:Nc] for k in range(Np)])
    return z[:Nc], z[Nc], pred

CONTROLLERS = dict(pure_pursuit=ctrl_pure_pursuit, stanley=ctrl_stanley, lqr=ctrl_lqr,
                   lmpc=ctrl_lmpc, ampc=ctrl_ampc, nmpc=ctrl_nmpc)

# ---------------- simulation ----------------
def run_scenario(cfg, scen, ctrl_name):
    import time
    path = make_scenario(scen)
    nom = dict(cfg['vehicle']); nom['mu'] = path['mu']          # what the controllers believe
    plant = dict(nom); rb = cfg['robust']
    if rb['mismatch']:
        plant['m'] *= rb['m_scale']; plant['Iz'] *= rb['Iz_scale']; plant['Cf'] *= rb['C_scale']; plant['Cr'] *= rb['C_scale']
        plant['mu'] *= rb['mu_scale']
    cfg = dict(cfg); cfg['nominal'] = nom
    rng = np.random.default_rng(rb['seed'])
    ctrl = CONTROLLERS[ctrl_name]; mem = {}
    dt, Ts = cfg['sim']['dt'], cfg['sim']['Ts']; nsub = int(round(Ts/dt))
    x = np.array([path['x'][0], path['y'][0], path['psi'][0], path['v0'], 0, 0, 0.])
    buf = [0.0]*rb['delay_steps']; dcmd = 0.0; it = 0; log = []; i_true = 0; status = 'finished'; comp = []
    t = 0.0
    while True:
        i_true, s_true, ey_true = project(path, x[0], x[1], i_true)
        if s_true >= path['s'][-1] - 2: break
        if abs(ey_true) > cfg['sim']['fail_ey']: status = 'left road'; break
        if t > cfg['sim']['t_max']: status = 'timeout'; break
        meas = dict(X=x[0], Y=x[1], psi=x[2], vx=x[3], vy=x[4], r=x[5], delta=x[6])
        if rb['noise']:
            meas['X'] += rng.normal()*rb['sigma_pos']; meas['Y'] += rng.normal()*rb['sigma_pos']
            meas['psi'] += rng.normal()*rb['sigma_psi']; meas['r'] += rng.normal()*rb['sigma_r']
            meas['vy'] += rng.normal()*rb['sigma_vy']; meas['vx'] += rng.normal()*rb['sigma_vx']
        t0 = time.perf_counter(); u, mem = ctrl(meas, path, cfg, mem); comp.append(time.perf_counter()-t0)
        u = float(np.clip(u, -nom['delta_max'], nom['delta_max']))
        buf.append(u); dcmd = buf.pop(0)
        j = path_point(path, s_true); ax_cmd = cfg['sim']['speed_kp']*(path['v'][j] - x[3]) + path['a'][j]
        psi_p = path['psi'][i_true]; epsi = atan2(sin(x[2]-psi_p), cos(x[2]-psi_p))
        _, _, af, ar = axle_forces(x[3], x[4], x[5], x[6], plant)
        clear = min([hypot(x[0]-path['x'][path_point(path,o['s'])]+sin(path['psi'][path_point(path,o['s'])])*o['d'],
                           x[1]-path['y'][path_point(path,o['s'])]-cos(path['psi'][path_point(path,o['s'])])*o['d']) - o['r'] - nom['half_width']
                     for o in path['obstacles']] or [np.inf])
        log.append(dict(t=t, X=x[0], Y=x[1], psi=x[2], vx=x[3], vy=x[4], r=x[5], delta=x[6], u=u, s=s_true, ey=ey_true, epsi=epsi,
                        vref=path['v'][j], af=af, ar=ar, ay=x[3]*x[5], clear=clear))
        for _ in range(nsub): x = rk4(x, dcmd, ax_cmd, plant, dt)
        t += Ts
    return dict(log=log, status=status, path=path, comp=np.array(comp), plant=plant)

def metrics(res):
    L = res['log']; ey = np.array([e['ey'] for e in L]); ep = np.array([e['epsi'] for e in L]); d = np.array([e['delta'] for e in L])
    cl = np.array([e['clear'] for e in L]); ay = np.array([e['ay'] for e in L])
    return dict(status=res['status'], rms_ey=float(np.sqrt(np.mean(ey**2))), max_ey=float(np.max(np.abs(ey))),
                rms_epsi_deg=float(np.degrees(np.sqrt(np.mean(ep**2)))), max_delta_deg=float(np.degrees(np.max(np.abs(d)))),
                steer_rate_rms=float(np.sqrt(np.mean(np.diff(d)**2))/0.05), max_ay=float(np.max(np.abs(ay))),
                min_clear=float(np.min(cl)) if np.isfinite(cl).any() else np.nan,
                t_end=L[-1]['t'], comp_ms=float(1000*np.mean(res['comp'])))
