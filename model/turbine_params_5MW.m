function [tp] = turbine_params_5MW()
    tp.R = 63;
    tp.g = 97;
    
    tp.K_d = 867e6;
    tp.C_d = 6.2e6;
    tp.J_r = 11.77e6;
    tp.J_g = 534;

    tp.omega = 2*pi;
    tp.zeta = 0.7;

    tp.rho = 1.225;

    load("cp_surface_5mw.mat");
    tp.C_p = Cp;
    [tp.theta_grid, tp.lambda_grid] = meshgrid(pitch_cmds, lambda_cmds);
    tp.theta_bp = pitch_cmds;
    tp.lambda_bp = lambda_cmds;
    %tp.C_p = griddedInterpolant({lambda_cmds,pitch_cmds},Cp,'spline');

    % maximizing value of C_p (found at theta=0) and the corresponding
    % maximizing value of lambda
    lambda0 = (lambda_cmds(1) + lambda_cmds(end))/2;
    C_p_lambda = @(lambda) -interp2(tp.theta_grid, tp.lambda_grid, tp.C_p, 0, lambda, 'spline');
    [tp.lambda_star, minus_C_p_star] = fminunc(C_p_lambda,lambda0);
    tp.C_p_star = -minus_C_p_star;
    
    % rated v_x, Omega_r don't line up with nominal values, calculate
    % directly from c_p curve and rated power
    tp.P_rated = 5e6;
    tp.v_x_rated = (2*tp.P_rated/(tp.rho*pi*tp.R^2*tp.C_p_star))^(1/3);
    tp.Omega_r_rated = tp.lambda_star*tp.v_x_rated/tp.R;
    % in MNm
    tp.M_g_rated = 1e-6*tp.P_rated/tp.Omega_r_rated;
    
    % need to generate theta_star curve for region 3
    lambda = @(vx) tp.Omega_r_rated * tp.R / vx;
    f = @(vx, theta) 0.5 * tp.rho * pi * tp.R^3 * vx^2 ...
        * interp2(tp.theta_grid, tp.lambda_grid, tp.C_p, theta, lambda(vx), 'spline') ...
        / lambda(vx) - tp.M_g_rated;

    tp.v_x_bp = 0:0.1:25;
    for k=1:length(tp.v_x_bp)
        if tp.v_x_bp(k) < tp.v_x_rated
            tp.theta_v_x = 0;
        else
            g = @(theta) f(tp.v_x_bp(k),theta);
            tp.theta_v_x(k) = fzero(g, [-0.001 tp.theta_bp(end)]);
        end
    end

    Simulink.Bus.createObject(tp);
end