function [simin_out] = BLCTRL_init(simin)
    rho = 1.225;
    R = 63;
    
    [CP_star, theta_star, lambda_star] = CP_find_max('CP_surface/BL5MW.mat');
    
    K_T = 0.5e-6*pi*rho*R^5*CP_star/(lambda_star^3);

    Omega_r_rated = 12.1*pi / 30;
    P_rated = 5; % MW
    
    
    Omega_1 = Omega_r_rated * 0.99;
    Omega_2 = Omega_r_rated;

    M_1 = K_T * Omega_1^2;
    M_2 = P_rated / Omega_r_rated;

    K_I = 0.1;
    K_P = 0.3;
    
    KK = 0.284;
    
    simin_out = simin.setVariable('BLCTRL_P_rated', P_rated);
    simin_out = simin_out.setVariable('BLCTRL_Omega_1', Omega_1);
    simin_out = simin_out.setVariable('BLCTRL_Omega_2', Omega_2);
    simin_out = simin_out.setVariable('BLCTRL_Omega_rated', Omega_r_rated);
    simin_out = simin_out.setVariable('BLCTRL_M_1', M_1);
    simin_out = simin_out.setVariable('BLCTRL_M_2', M_2);
    simin_out = simin_out.setVariable('BLCTRL_K_T', K_T);
    simin_out = simin_out.setVariable('BLCTRL_K_I', K_I);
    simin_out = simin_out.setVariable('BLCTRL_K_P', K_P);
    simin_out = simin_out.setVariable('BLCTRL_KK', KK);
    simin_out = simin_out.setVariable('BLCTRL_torque_rate', 0.015 * 97); % 15kNm/s at generator
    simin_out = simin_out.setVariable('BLCTRL_theta_star', theta_star);
    simin_out = simin_out.setVariable('BLCTRL_theta_max', pi/2); % 90deg
    simin_out = simin_out.setVariable('BLCTRL_theta_rate', 8*pi/180); % 8deg/s
    simin_out = simin_out.setVariable('BLCTRL_theta_sw', pi/180); % 1deg
end