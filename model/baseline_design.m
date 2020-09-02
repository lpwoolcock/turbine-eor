clear;

tp = turbine_params_5MW();

M_rated = 1e-6 * tp.P_rated / tp.Omega_r_rated;

K_T = 0.5e-6*pi*tp.rho*tp.R^5*tp.C_p_star/(tp.lambda_star^3);

Omega_1 = tp.Omega_r_rated * 0.9;
Omega_2 = tp.Omega_r_rated;

M_1 = K_T * Omega_1^2;

%{
Omega = 0:0.001:2;
M_g = zeros(1,length(Omega));

for k = 1:length(Omega)
    if Omega(k) < Omega_1
        M_g(k) = K_T * Omega(k)^2;
    elseif Omega(k) < Omega_2
        M_g(k) = M_1 + (M_rated - M_1) / (Omega_2 - Omega_1) * (Omega(k) - Omega_1);
    else
        M_g(k) = M_rated;
    end
end

plot(Omega,M_g);
%}

vx0 = 12;
[x_star, u_star, A, B, H] = eq_region3_cont(tp, vx0);
C = [1 0 0 0 0];
G = ss(A,B,C,[0]);

K_I = 0.1;
K_P = 0.3;
s = tf('s');
Ctrl = K_I/s + K_P;

margin(-G*Ctrl);

lambda = @(Omega_r, vx) Omega_r * tp.R / vx;
M_a = @(Omega_r, vx, theta) 0.5 * tp.rho * pi * tp.R^3 * vx^2 ...
    * interp2(tp.theta_grid, tp.lambda_grid, tp.C_p, theta, lambda(Omega_r, vx), 'cubic') ...
    / lambda(Omega_r, vx);
d_theta = 0.001;
beta = @(theta) (M_a(x_star(1), vx0, theta + d_theta)...
    - M_a(x_star(1), vx0, theta)) / d_theta;

beta_0 = beta(u_star(1));
KK = fzero(@(theta) beta(theta)-2*beta_0,[0 0.4]);

