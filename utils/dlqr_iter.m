function K = dlqr_iter(A, B, Q, R)
%DLQR_ITER Discrete LQR gain by Riccati iteration (no Control System Toolbox needed).
%   u = -K*x minimizes sum x'Qx + u'Ru for x(k+1) = A x(k) + B u(k).
P = Q;
for it = 1:500
    K = (R + B'*P*B) \ (B'*P*A);
    Pn = Q + A'*P*(A - B*K);
    if max(abs(Pn(:) - P(:))) < 1e-9*max(1, max(abs(P(:))))
        P = Pn;
        break
    end
    P = Pn;
end
K = (R + B'*P*B) \ (B'*P*A);
end
