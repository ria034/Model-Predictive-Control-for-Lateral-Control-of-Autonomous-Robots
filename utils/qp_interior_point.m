function z = qp_interior_point(H, f, G, h)
%QP_INTERIOR_POINT Dense primal-dual interior-point QP solver (Mehrotra predictor-corrector).
%   z = argmin 0.5*z'*H*z + f'*z  subject to  G*z <= h
%   Written so the project needs no Optimization Toolbox. Sized for the
%   small QPs here (about 11 variables, 120 constraints).
n = numel(f); mI = numel(h);
f = f(:); h = h(:);
z = zeros(n,1);
s = max(h - G*z, 1.0);
lam = ones(mI,1);
tol = 1e-8;
for it = 1:50
    rd = H*z + f + G'*lam;
    rp = G*z + s - h;
    mu = (s'*lam)/mI;
    if norm(rd) < tol && norm(rp) < tol && mu < tol
        break
    end
    W = lam./s;
    M = H + G'*(W.*G) + 1e-10*eye(n);
    % affine (predictor) step
    [dz, ds, dl] = newton_step(M, G, rd, rp, s, lam, s.*lam);
    a = min(max_step(s, ds), max_step(lam, dl));
    mu_aff = ((s + a*ds)'*(lam + a*dl))/mI;
    sigma = (mu_aff/mu)^3;
    % corrector step
    [dz, ds, dl] = newton_step(M, G, rd, rp, s, lam, s.*lam + ds.*dl - sigma*mu);
    a = 0.99*min(max_step(s, ds), max_step(lam, dl));
    z = z + a*dz;
    s = max(s + a*ds, 1e-14);
    lam = max(lam + a*dl, 1e-14);
end
end

function [dz, ds, dl] = newton_step(M, G, rd, rp, s, lam, rc)
rhs = -rd - G'*((lam.*rp - rc)./s);
dz = M \ rhs;
ds = -rp - G*dz;
dl = (-rc - lam.*ds)./s;
end

function a = max_step(v, dv)
idx = dv < 0;
if any(idx)
    a = min(1.0, min(-v(idx)./dv(idx)));
else
    a = 1.0;
end
end
