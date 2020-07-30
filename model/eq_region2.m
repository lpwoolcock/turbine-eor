function [x_star, u_star, A, B, C, H, K] = eq_region2(tp, vx0, T)
    Omega_r_star = tp.lambda_star * vx0 / tp.R;
    Ma_star = 0.5e-6 * tp.rho * pi * tp.R^3 ...
              * tp.C_p_star / tp.lambda_star ...
              * vx0^2; % in MNm
    
    % K_d in Nm/rad, phi in mrad
    x_star = [Omega_r_star; Ma_star/(1e-9*tp.K_d); Omega_r_star; 0; 0];
    u_star = [0; Ma_star];
    
    % note that THIS M_a is in Nm
    lambda = @(Omega_r, vx) Omega_r * tp.R / vx;
    M_a = @(Omega_r, vx, theta) tp.rho * pi * tp.R^3 * vx^2 ...
        * interp2(tp.theta_grid, tp.lambda_grid, tp.C_p, theta, lambda(Omega_r, vx), 'cubic') ...
        / lambda(Omega_r, vx);
    
    d_vx = 0.001;
    alpha = (M_a(Omega_r_star, vx0 + d_vx, 0)...
        - M_a(Omega_r_star, vx0, 0)) / d_vx;
    
    d_Omega_r = 0.001;
    gamma = (M_a(Omega_r_star + d_Omega_r, vx0, 0)...
        - M_a(Omega_r_star, vx0, 0)) / d_Omega_r;
    

    A_c = [(gamma-tp.C_d)/tp.J_r -tp.K_d/(1000*tp.J_r)       tp.C_d/tp.J_r;
           1000                   0                         -1000      ;
           tp.C_d/(tp.g*tp.J_g)   tp.K_d/(1000*tp.g*tp.J_g) -tp.C_d/(tp.g*tp.J_g)];

    B_c = [0;
         0;
         -1e6/(tp.g*tp.J_g)];
    H_c = [alpha/tp.J_r;
         0;
         0];
     
    A = expm(A_c*T);
    B = A_c\(A-eye(size(A)))*B_c;
    C = [1 0 0];
    H = A_c\(A-eye(size(A)))*H_c;
    
    % needs to be using discretised versions
    coder.extrinsic('lqr');
    [K,~,~] = lqr(A_c,B_c,C.'*C,1e-3);
end