
function [v_0, delta_h, delta_v] = REWS(velocity, y, z, dt, zHub, mffws, grid_width, radius) 

    % velocity is a 4D array with dimensions in time, (U, V, W components), y,
    % z positions. 
    % assume odd grid number, the hub position is in the middle of the grid

    y_hub_position = (y(1)+y(end))/2; %assume center of grid
    z_hub_position = zHub;

    [Y_mesh,Z_mesh] = meshgrid(y,z);
    dist_sqr = (Y_mesh-y_hub_position).^2 + (Z_mesh-z_hub_position).^2;
    within_radius = dist_sqr < radius^2;
    
    Y = Y_mesh(within_radius) - y_hub_position;
    Z = Z_mesh(within_radius) - z_hub_position;
    
    j_offset = ceil((grid_width/mffws)/dt);
    N = size(velocity, 1) - (j_offset - 1);
    M = size(Y, 1);
    
    v_0 = zeros(N, 1);
    delta_h = zeros(N, 1);
    delta_v = zeros(N, 1);
    
    for j = j_offset:N
        slice = velocity(j,1,:,:);
        m = slice(within_radius);
        A = [ones(M,1) Y Z];
        
        s = A\m;
        v_0(j - (j_offset-1)) = s(1);
        delta_h(j - (j_offset-1)) = s(2);
        delta_v(j - (j_offset-1)) = s(3);
    end

end

