function [simin_out] = BLCTRL_init(simin, turbine_model_path)
    rho = 1.225;

    s = what(turbine_model_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    turbine_model_name = path_parts{end};
    turbine_model_path = strcat(s.path, '\');
    
    load(strcat(turbine_model_path, turbine_model_name, '.mat'));
    load(strcat(turbine_model_path, turbine_model_name, '_CP.mat'));
    
    [CP_star, theta_star, lambda_star] = CP_find_max(CP, CP_lambda, CP_theta);
    
    K_T = 0.5e-6*pi*rho*R^5*CP_star/(lambda_star^3);
    
    Omega_1 = Omega_rated * 0.99;
    Omega_2 = Omega_rated;

    M_1 = K_T * Omega_1^2;
    M_2 = 1e-6 * P_rated / Omega_rated;

    %K_I = 0.1;
    K_I = 0.008068634 * 180 / pi;
    %K_P = 0.3;
    K_P = 0.01882681 * 180 / pi;
    
    KK = 0.284;
    
    simin_out = simin.setVariable('BLCTRL_P_rated', P_rated * 1e-6);
    simin_out = simin_out.setVariable('BLCTRL_Omega_1', Omega_1);
    simin_out = simin_out.setVariable('BLCTRL_Omega_2', Omega_2);
    simin_out = simin_out.setVariable('BLCTRL_Omega_rated', Omega_rated);
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
    simin_out = simin_out.setVariable('BLCTRL_T', 0.1);
end