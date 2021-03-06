function [CP_star, theta_star, lambda_star] = CP_find_max(CP, CP_lambda, CP_theta)
    [theta_grid, lambda_grid] = meshgrid(CP_theta, CP_lambda);
    
    lambda0 = (min(CP_lambda) + max(CP_lambda))/2;
    theta0 = (min(CP_theta) + max(CP_theta))/2;
    CP_surf = @(x) -interp2(theta_grid, lambda_grid, CP, x(1), x(2), 'spline');
    
    A = [1 0; -1 0; 0 1; 0 -1];
    b = [max(CP_theta); -min(CP_theta); max(CP_lambda); -min(CP_lambda)];
    
    [x_star, minus_CP_star] = fmincon(CP_surf, [theta0;lambda0], A, b); 
    theta_star = x_star(1);
    lambda_star = x_star(2);
    CP_star = -minus_CP_star;
end

