function [Ad, Bd] = c2d_zoh(A, B, Ts)
%C2D_ZOH Exact zero-order-hold discretization using the matrix exponential.
n = size(A,1); m = size(B,2);
M = zeros(n+m);
M(1:n,1:n) = A*Ts;
M(1:n,n+1:end) = B*Ts;
E = expm(M);
Ad = E(1:n,1:n);
Bd = E(1:n,n+1:end);
end
