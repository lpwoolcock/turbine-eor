function [tp, tp_bus] = turbine_params_5MW()
    tp.R = 63;

    tp.g = 97;
    tp.K_d = 867e6;
    tp.C_d = 6.2e6;
    tp.J_r = 11.77e6;
    tp.J_g = 534;

    tp.omega = 2*pi;
    tp.zeta = 0.7;

    tp.rho = 1.225;

    load("cp_surface.mat");
    pitch_cmds = pitch_cmds * pi/180; % stored as degrees
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
    
    tp_bus = Simulink.Bus.createObject(tp);
end