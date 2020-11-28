function [] = LIDAR_simulator(turbsim_input, csv_output, time, t_s, N)
    LIDAR_focal_distance = 60;      %focal distance of the LIDAR
    grid_width = 65;                %simulation width for FAST
    t=0:t_s:time; 

    [velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(turbsim_input);
    u_inf = zeros(numel(t),1);
    for i = 1:numel(t)
        u_inf(i,1) = LIDAR(velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws,LIDAR_focal_distance,grid_width, t(i));
    end
    
    % Now filter the scanned measurements
    Bw = 87/(LIDAR_focal_distance^2); % cut off frequency in m^-1
    omega_c = 2*pi*Bw;%*mffws % cut off frequency in rad/s
    wc = omega_c*t_s; % cut off freq in rad/sample, should be multiplied by ts ...

    u_inf_filtered = FIR_lowpass(N, wc, u_inf);
    writematrix([t',u_inf_filtered], csv_output);
end




