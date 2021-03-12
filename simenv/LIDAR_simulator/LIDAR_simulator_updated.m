%==========================================================================
% Takes a turbsim file and produces simulated wind speed measurements 
% using a LIDAR at hub height, 
% Approximates the spacial averageing along the lazer with an FIR 
% Non-causal filter. Assumes that the input windfile deviates around a
% fixed mean windspeed. 
% Make sure the wind file > TMax + N*t_s/2 long
%==========================================================================

function [] = LIDAR_simulator_updated(turbsim_input, csv_output, time, t_s, N)

    LIDAR_focal_distance = 50;                          %focal distance of the LIDAR
    LIDAR_angle = 0.2618;                               %angle of the LIDAR in radians
    num_lidar_pts = 50;                                 %the number of regions sampled by the LIDAR
    rayleight_length = 0.000485*LIDAR_focal_distance^2; %rayleigh length of the lidar, proportinal to the focal depth. Effects the bandwidth of the LIDAR.
    grid_width = 130;                                   %simulation width for FAST
    measurment_noise = 0.1;                             %measurement noise in +-m/s

    %define the root directory for absolute paths. 
    %It makes MATLAB happy relative paths make MATLAB sad.
%{
    current_path = pwd;
    k = strfind(current_path,'ESC_Turbine_Project');
    root = strcat(current_path(1:k-1),'ESC_Turbine_Project');
    fprintf('Using: %s as root directory\n\n',root);
    addpath(genpath(root));

    wind_cache_path = strcat(root,'\Wind_Simulators\Wind_Cache');

    LIDAR_wind_path = wind_cache_path;
    [tmp_filepath,tmp_name,tmp_ext] = fileparts(windfile_name);
    LIDAR_wind_name = sprintf("LIDAR_wind_%s",tmp_name);
%}
    t=1:t_s:time; 

    [velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(turbsim_input);

    u_inf = zeros(numel(t),1);

    mean = 0;
    for i = 1:numel(t)

        u_inf(i,1) = LIDAR_updated(velocity, y, z, nz, ny, dt, zHub,LIDAR_focal_distance,grid_width,mffws,t(i),LIDAR_angle,num_lidar_pts,measurment_noise); %TO DO: NEED MFFWS
        mean = mean+u_inf(i,1);
    end
    mean = mean/numel(t); %get average windspeed. TO DO: Consider doing this periodically with a moving average filter or similar.

    % Now filter the scanned measurements

    omega_c = -mean*log(0.5)/(2*rayleight_length); % cut off frequency in rad/s
    wc = omega_c*t_s;                              % cut off freq in rad/sample

    u_inf_filtered = FIR_lowpass(N, wc, u_inf);
    writematrix([t',u_inf_filtered], csv_output);
end
