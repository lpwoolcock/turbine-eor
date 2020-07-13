function [x_star, u_star, A, B, H] = eq_region2(td, vx0)
    Omega_r_star = td.lambda_star * vx0 / td.R;
    Ma_star = 0.5 * td.rho * pi * td.R^3 ...
              * td.Cp(td.lambda_star,0) / td.lambda_star ...
              * vx0^2;
          
    x_star = [Omega_r_star; -Ma_star/td.Kd; Omega_r_star; 0; 0];
    u_star = [0; -Ma_star];
    
    
    d_Omega_r = 0.001;
    Omega_rp = Omega_r_star + d_Omega_r;
    lambda_p = td.R*Omega_rp/vx0;
    Ma_p = 0.5 * td.rho * pi * td.R^3 ...
              * td.Cp(lambda_p,0) / lambda_p ...
              * vx0^2;
    gamma = (Ma_p - Ma_star) / d_Omega_r;
    
    d_vx = 0.001;
    vx_p = vx0 + d_vx;
    lambda_p = td.R*Omega_r_star/vx_p;
    Ma_p = 0.5 * td.rho * pi * td.R^3 ...
              * td.Cp(lambda_p,0) / lambda_p ...
              * vx_p^2;
    alpha = (Ma_p - Ma_star) / d_vx;
    
    A = [(gamma-td.Cd)/td.Jr -1/td.Jr td.Cd/td.Jr;
         td.Kd 0 -td.Kd;
         td.Cd/td.Jg 1/td.Jg -td.Cd/td.Jg];
    B = [0; 0; -1/td.Jg];
    H = [alpha/td.Jr; 0; 0];
end