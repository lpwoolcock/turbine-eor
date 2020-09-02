function [x_star, u_star, A, B, H] = eq_region3_cont(tp, vx0)
    Omega_r_star = tp.Omega_r_rated;
    Ma_star = tp.M_g_rated;
    
    lambda_0 = Omega_r_star * tp.R / vx0;
    
    f = @(theta) 0.5e-6 * tp.rho * pi * tp.R^3 ...
    * interp2(tp.theta_grid, tp.lambda_grid, tp.C_p, theta, lambda_0, 'cubic')...
    * vx0^2 / lambda_0 - Ma_star;

    theta_star = fzero(f, [tp.theta_bp(1) tp.theta_bp(end)]);
    
    x_star = [Omega_r_star; Ma_star/(1e-9*tp.K_d); Omega_r_star; theta_star; 0];
    u_star = [theta_star; Ma_star];
    
    % note that THIS M_a is in Nm
    lambda = @(Omega_r, vx) Omega_r * tp.R / vx;
    M_a = @(Omega_r, vx, theta) 0.5 * tp.rho * pi * tp.R^3 * vx^2 ...
        * interp2(tp.theta_grid, tp.lambda_grid, tp.C_p, theta, lambda(Omega_r, vx), 'cubic') ...
        / lambda(Omega_r, vx);
    
    d_vx = 0.001;
    alpha = (M_a(Omega_r_star, vx0 + d_vx, theta_star)...
        - M_a(Omega_r_star, vx0, theta_star)) / d_vx;
    
    d_theta = 0.001;
    beta = (M_a(Omega_r_star, vx0, theta_star + d_theta)...
        - M_a(Omega_r_star, vx0, theta_star)) / d_theta;
    
    d_Omega_r = 0.001;
    gamma = (M_a(Omega_r_star + d_Omega_r, vx0, theta_star)...
        - M_a(Omega_r_star, vx0, theta_star)) / d_Omega_r;
    

    A = [(gamma-tp.C_d)/tp.J_r -tp.K_d/(1000*tp.J_r)       tp.C_d/tp.J_r        beta/tp.J_r 0;
           1000                   0                         -1000                 0           0;
           tp.C_d/(tp.g*tp.J_g)   tp.K_d/(1000*tp.g*tp.J_g) -tp.C_d/(tp.g*tp.J_g) 0           0;
           0                      0                          0                    0           1;
           0                      0                          0                   -tp.omega^2 -2*tp.zeta*tp.omega];
    B = [0;
           0; 
           0;
           0;
           tp.omega^2];
    H = [alpha/tp.J_r;
           0;
           0;
           0;
           0];
end

