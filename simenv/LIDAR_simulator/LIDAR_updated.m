
%%
function u_lidar = LIDAR_updated(velocity, y, z, nz, ny, dt, zHub, focal_distance, grid_width, mffws, time, LIDAR_angle, num_lidar_pts, measurment_noise) 

    % velocity is a 4D array with dimensions in time, (U, V, W components), y,
    % z positions. 
    % assume odd grid number, the hub position is in the middle of the grid
    %y_hub_position = idivide(1 + ny, int32(2), 'round');
    %z_hub_position = idivide(1 + nz, int32(2), 'round')+1;

    radius = tan(LIDAR_angle)*focal_distance;

    y_hub_position = (y(1)+y(end))/2; %assume center of grid
    z_hub_position = zHub;

    %get the query points of the circle, each row is a [y, z] coordinate
    lidar_pts = zeros(num_lidar_pts,2);
    a = 0;                      

    for i=1:num_lidar_pts
        a = a+2*pi/num_lidar_pts;
        lidar_pts(i,:) = [y_hub_position+radius*cos(a), z_hub_position+radius*sin(a)];
    end


    %interpolate U,V,W at these points at time t

    t = ceil((time+grid_width/mffws)/dt); %convert from realtime to sample, ceil to keep matlab happy.

    %flatten into a vector, i split the data so i dont go mad with indexes
    U_t = zeros(ny*nz, 1);
    V_t = zeros(ny*nz, 1);
    W_t = zeros(ny*nz, 1);
    grid_pts = zeros(ny*nz,2);

     for i = 1:nz
        for j = 1:ny 
            U_t(j+(i-1)*nz+1,1) = velocity(t,1,i,j); %indexing in matlab is the worst 
            V_t(j+(i-1)*nz+1,1) = velocity(t,2,i,j);
            W_t(j+(i-1)*nz+1,1) = velocity(t,3,i,j);
            grid_pts(j+(i-1)*nz+1,:) = [y(j),z(i)];
        end
     end

    %interpolate surfaces for each dimention, quick and dirty linear
    %interpolation this will run alot and get very expensive
    U_l = scatteredInterpolant(grid_pts(:,1),grid_pts(:,2),U_t);
    V_l = scatteredInterpolant(grid_pts(:,1),grid_pts(:,2),V_t);
    W_l = scatteredInterpolant(grid_pts(:,1),grid_pts(:,2),W_t);

    %extract the points
    U_l_pts = zeros(num_lidar_pts,1);
    V_l_pts = zeros(num_lidar_pts,1);
    W_l_pts = zeros(num_lidar_pts,1);

    for i=1:num_lidar_pts

        U_l_pts(i,1) = U_l(lidar_pts(i,1),lidar_pts(i,2));
        V_l_pts(i,1) = V_l(lidar_pts(i,1),lidar_pts(i,2));
        W_l_pts(i,1) = W_l(lidar_pts(i,1),lidar_pts(i,2));

    end

    %get the vector projection along the lazer
    v_mes = zeros(num_lidar_pts,1);
    for i = 1:num_lidar_pts
        PO = [focal_distance,lidar_pts(i,1)-y_hub_position,lidar_pts(i,2)-z_hub_position];
        v_p = [U_l_pts(i,1),V_l_pts(i,1),W_l_pts(i,1)];
        v_mes(i,1) = dot(PO/norm(PO),v_p);
    end

    %resolve u component assuming V=W=0
    U_est = v_mes*sqrt(radius^2+focal_distance^2)/focal_distance;
    U_avg = 0;

    %add measurement noise
    U_est = U_est+(rand()-0.5)*measurment_noise;

    %average the mesurment to cancel out shear wind. 
    %TO DO: Conider more soffisticated analysis.
    for i=1:num_lidar_pts
        U_avg = U_avg+U_est(i)/num_lidar_pts;
    end
    u_lidar = U_avg;

end
