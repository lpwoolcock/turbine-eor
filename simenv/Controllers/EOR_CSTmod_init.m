function [simin_out] = EOR_CST_init(simin, turbine_model_path)
    s = what(turbine_model_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    turbine_model_name = path_parts{end};
    turbine_model_path = strcat(s.path, '\');
    
    load(strcat(turbine_model_path, turbine_model_name, '.mat'));
    load(strcat(turbine_model_path, turbine_model_name, '_CP.mat'));
    
    % "windspeed" is mean windspeed, determines linearisation point
    windspeed = simin.getVariable('windspeed');
    % rated torque derived from rated power and rotor speed
    M_rated = 1e-6 * P_rated / Omega_rated;
    % optimal aerodynamic operating conditions
    [CP_opt, theta_opt, lambda_opt] = CP_find_max(CP, CP_lambda, CP_theta);

    % calculate equilibrium values of theta for different windspeeds
    [theta_grid, lambda_grid] = meshgrid(CP_theta, CP_lambda);
    lambda = @(vx) Omega_rated * R / vx;
    f = @(vx, theta) 0.5e-6 * rho * pi * R^3 * vx^2 ...
        * interp2(theta_grid, lambda_grid, CP, theta, lambda(vx), 'cubic') ...
        / lambda(vx) - M_rated;
    v_x_breakpoints = 0:0.05:25;
    theta_star_lookup(1) = theta_opt;
    % start from 2 to skip divide by zero
    for k = 2:length(v_x_breakpoints)

        g = @(theta) f(v_x_breakpoints(k),theta);
        try
            theta_sol = fzero(g, [CP_theta(1) CP_theta(end)]);
            theta_star_lookup(k) = theta_sol;
        catch
            theta_star_lookup(k) = theta_opt;
        end
    end
    
    % calculate equilibrium operating points corresponding to windspeed
    Omega_r_star = min(lambda_opt * windspeed / R, Omega_rated);
    theta_star = interp1(v_x_breakpoints, theta_star_lookup, windspeed);
    M_g_star = min(0.5e-6 * rho * pi * R^3 * CP_opt / lambda_opt * windspeed^2, M_rated); % in MNm
    u_star = [theta_star; M_g_star];
    x_star = [Omega_r_star; M_g_star/(1e-9*K_d); Omega_r_star; theta_star; 0];

    
    % calculate linearisation parameters corresponding to operating point
    lambda = @(Omega_r, vx) Omega_r * R / vx;
    M_a = @(Omega_r, vx, theta) 0.5 * rho * pi * R^3 * vx^2 ...
        * interp2(theta_grid, lambda_grid, CP, theta, lambda(Omega_r, vx), 'cubic') ...
        / lambda(Omega_r, vx);

    d_vx = 0.001;
    alpha = (M_a(Omega_r_star, windspeed + d_vx, theta_star)...
        - M_a(Omega_r_star, windspeed, theta_star)) / d_vx;

    d_Omega_r = 0.001;
    gamma = (M_a(Omega_r_star + d_Omega_r, windspeed, theta_star)...
        - M_a(Omega_r_star, windspeed, theta_star)) / d_Omega_r;

    theta_star_interp = @(v_x) interp1(v_x_breakpoints, theta_star_lookup, v_x);
    d_theta_star = (theta_star_interp(windspeed+0.001) - theta_star_interp(windspeed))/0.001;
    
    % generate linearised system matrices for region 2 & 3
    g = gearbox_ratio;
    T = 0.1;
    
    A2_c = [(gamma - C_d)/J_r -K_d/(1000*J_r) C_d/J_r;
           1000 0 -1000;
            C_d/(g*J_g) K_d/(1000*g*J_g) -C_d/(g*J_g)];
    B2_c = [0;
           0;
          -1e6/(g*J_g)];
    E2w_c = [alpha/J_r 0 0; 0 0 0; 0 0 0];
    
    A2 = expm(A2_c*T);
    B2 = A2_c\(A2-eye(size(A2)))*B2_c;
    C2e = [1 0 0];
    E2w = A2_c\(A2-eye(size(A2)))*E2w_c;
    D2eu = 0;
    D2ew = [-lambda_opt/R 0 0];
    [K2, ~, ~] = dlqr(A2, B2, eye(3), 10);
    F2 = -K2;
    
    A3_c = [0 1; -omega^2 -2*zeta*omega];
    B3_c = [0; omega^2];
    
    A3 = expm(A3_c*T);
    B3 = A3_c\(A3-eye(size(A3)))*B3_c;
    C3e = [1 0];
    E3w = zeros(2,3);
    D3eu = 0;
    D3ew = [-d_theta_star 0 0];
    F3 = zeros(1,2);
    
    % find "rated" windspeed and decide on region 2/3 operation based on it

    
    simin_out = simin.setVariable('EOR_T', T);
    simin_out = simin_out.setVariable('EOR_T_setup', 60);
    simin_out = simin_out.setVariable('EOR_T_solve', 1);
    
    simin_out = simin_out.setVariable('EOR_x_star', x_star);
    simin_out = simin_out.setVariable('EOR_u_star', u_star);
    
    simin_out = simin_out.setVariable('EOR_A2', A2);
    simin_out = simin_out.setVariable('EOR_B2', B2);
    simin_out = simin_out.setVariable('EOR_C2e', C2e);
    simin_out = simin_out.setVariable('EOR_E2w', E2w);
    simin_out = simin_out.setVariable('EOR_D2eu', D2eu);
    simin_out = simin_out.setVariable('EOR_D2ew', D2ew);
    simin_out = simin_out.setVariable('EOR_F2', F2);
    
    simin_out = simin_out.setVariable('EOR_A3', A3);
    simin_out = simin_out.setVariable('EOR_B3', B3);
    simin_out = simin_out.setVariable('EOR_C3e', C3e);
    simin_out = simin_out.setVariable('EOR_E3w', E3w);
    simin_out = simin_out.setVariable('EOR_D3eu', D3eu);
    simin_out = simin_out.setVariable('EOR_D3ew', D3ew);
    simin_out = simin_out.setVariable('EOR_F3', F3);
    
    simin_out = simin_out.setVariable('EOR_LQR_R2', 10);
    
    simin_out = simin_out.setVariable('EOR_filt_b', [0 0.5699 0 86.6992]);
    simin_out = simin_out.setVariable('EOR_filt_a', [1.0 8.6787 37.4972 86.6992]);

    
    simin_out = BLCTRL_init(simin_out, turbine_model_path);
end

