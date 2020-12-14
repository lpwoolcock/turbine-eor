function [CP_star, theta_star, lambda_star] = CP_find_max(CP_filename)
    load(CP_filename);
    [theta_grid, lambda_grid] = meshgrid(pitch_cmds, lambda_cmds);
    
    lambda0 = (min(lambda_cmds) + max(lambda_cmds))/2;
    theta0 = (min(pitch_cmds) + max(pitch_cmds))/2;
    CP_surf = @(x) -interp2(theta_grid, lambda_grid, Cp, x(1), x(2), 'spline');
    
    A = [1 0; -1 0; 0 1; 0 -1];
    b = [max(pitch_cmds); -min(pitch_cmds); max(lambda_cmds); -min(lambda_cmds)];
    
    [x_star, minus_CP_star] = fmincon(CP_surf, [theta0;lambda0], A, b); 
    theta_star = x_star(1);
    lambda_star = x_star(2);
    CP_star = -minus_CP_star;
end

