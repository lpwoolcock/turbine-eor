
function [Vh, delta, Vz] = REWS(velocity, y, z, zHub, radius) 

    % velocity is a 4D array with dimensions in time, (U, V, W components), y,
    % z positions. 
    % assume odd grid number, the hub position is in the middle of the grid

    num_rews_pts = 24;

    y_hub_position = (y(1)+y(end))/2; %assume center of grid
    z_hub_position = zHub;


    %get the query points of the circle, each row is a [y, z] coordinate
    rews_idx = zeros(num_rews_pts,2);
    a = 0;                      

    for i=1:num_rews_pts
        a = a+2*pi/num_rews_pts;
        y_coord = y_hub_position+radius*cos(a);
        z_coord = z_hub_position+radius*sin(a);
        
        y_err = abs(y - y_coord);
        [~, y_idx] = min(y_err);
        
        z_err = abs(z - z_coord);
        [~, z_idx] = min(z_err);
        
        rews_idx(i,:) = [y_idx, z_idx];
    end

    N = size(velocity, 1);
    M = size(rews_idx, 1);
    
    delta = zeros(N, 1);
    Vh = zeros(N, 1);
    Vz = zeros(N, 1);
    
    for j = 1:N
        vel = zeros(3,1);
        for k = 1:M
            vel = vel + (1/M) * velocity(j, :, rews_idx(k, 1), rews_idx(k, 2)).';
        end
        
        delta(j) = (180/pi) * atan(vel(2) / vel(1));
        Vh(j) = sqrt(vel(1)^2 + vel(2)^2);
        Vz(j) = vel(3);
    end

end

