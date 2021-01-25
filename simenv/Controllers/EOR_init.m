function [simin_out] = EOR_init(simin, turbine_model_path)
    rho = 1.225;

    s = what(turbine_model_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    turbine_model_name = path_parts{end};
    turbine_model_path = strcat(s.path, '\');
    
    load(strcat(turbine_model_path, turbine_model_name, '.mat'));
    load(strcat(turbine_model_path, turbine_model_name, '_CP.mat'));
    
    v_x_breakpoints = 0:0.1:25;
    
    [~, theta_star, lambda_star] = CP_find_max(CP, CP_lambda, CP_theta);

    M_rated = 1e-6 * P_rated / Omega_rated;
    
    Omega_star_lookup = min(v_x_breakpoints * lambda_star / R, Omega_rated);

    [theta_grid, lambda_grid] = meshgrid(CP_theta, CP_lambda);
    
    lambda = @(vx) Omega_rated * R / vx;
    f = @(vx, theta) 0.5e-6 * rho * pi * R^3 * vx^2 ...
        * interp2(theta_grid, lambda_grid, CP, theta, lambda(vx), 'spline') ...
        / lambda(vx) - M_rated;
    
    
    theta_star_lookup(1) = theta_star;
    % start from 2 to skip divide by zero
    for k = 2:length(v_x_breakpoints)

        g = @(theta) f(v_x_breakpoints(k),theta);
        try
            theta_sol = fzero(g, [CP_theta(1)-0.001 CP_theta(end)]);
            theta_star_lookup(k) = theta_sol;
        catch
            theta_star_lookup(k) = theta_star;
        end
    end
    
    simin_out = simin.setVariable('EOR_v_x_breakpoints', v_x_breakpoints);
    simin_out = simin_out.setVariable('EOR_Omega_star_lookup', Omega_star_lookup);
    simin_out = simin_out.setVariable('EOR_theta_star_lookup', theta_star_lookup);
    
    T = 0.1;
    
    simin_out = simin_out.setVariable('EOR_T', T);
    simin_out = simin_out.setVariable('EOR_T_avg', 120);
    simin_out = simin_out.setVariable('EOR_T_solve', 1);
    
    A = [0 1; -omega^2 -2*zeta*omega];
    B = [0; omega^2];
    
    A_d = expm(A*T);
    B_d = A\(A_d-eye(size(A_d)))*B;
    
    simin_out = simin_out.setVariable('EOR_pitch_A', A_d);
    simin_out = simin_out.setVariable('EOR_pitch_B', B_d);
    simin_out = simin_out.setVariable('EOR_pitch_Ce', [1 0]);
    simin_out = simin_out.setVariable('EOR_pitch_Dew', [-1 0 0]);
    simin_out = simin_out.setVariable('EOR_pitch_F', [0 0]);
    
    simin_out = simin_out.setVariable('EOR_torque_LQR_R', 10);
    
    %simin_out = simin_out.setVariable('EOR_filt_tau', 0.5);
    simin_out = simin_out.setVariable('EOR_filt_b', [0 0.5699 0 86.6992]);
    simin_out = simin_out.setVariable('EOR_filt_a', [1.0 8.6787 37.4972 86.6992]);
    
    simin_out = simin_out.setVariable('EOR_pitch_thresh', 1 * pi / 180);
    simin_out = simin_out.setVariable('EOR_torque', M_rated);
    
    simin_out = BLCTRL_init(simin_out, turbine_model_path);
end

