function LIDAR_Simulation(turbsim_file) 
% Takes a turbsim file (.bts) and produces simulated wind speed measurements 
% using LIDAR at hub height

[velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(turbsim_file);

% velocity is a 4D array with dimensions in time, (U, V, W components), y,
% z positions. 
% assume odd grid number, the hub position is in the middle of the grid
y_hub_position = idivide(1 + ny, int32(2), 'round');
z_hub_position = idivide(1 + nz, int32(2), 'round')+1;

hub_velocities = velocity(:,1,y_hub_position, z_hub_position);
sample_number = 0:numel(hub_velocities)-1;
time = dt*sample_number';

% Insert Filter Here


LIDAR_hubheight_high = timeseries(hub_velocities, time);
save 'LIDAR_hubheight_high.mat' -v7.3 LIDAR_hubheight_high
end