%==========================================================================
% Takes a turbsim file and produces simulated wind speed measurements 
% using a LIDAR at hub height, 
% Approximates the spacial averageing along the lazer with an FIR 
% Non-causal filter. Assumes that the input windfile deviates around a
% fixed mean windspeed. 
% Make sure the wind file > TMax + N*t_s/2 long
%==========================================================================

windfile_name = "turbulent6.wnd";                   %TO DO: make this inherit from fast or vice versa

t_s = 0.02;    %50Hz                                %sample rate of LIDAR
TMax = 6000;                                        %simulation time comment out to inherit from workspace 
LIDAR_focal_distance = 50;                          %focal distance of the LIDAR
LIDAR_angle = 0.2618;                               %angle of the LIDAR in radians
num_lidar_pts = 50;                                 %the number of regions sampled by the LIDAR
rayleight_length = 0.000485*LIDAR_focal_distance^2; %rayleigh length of the lidar, proportinal to the focal depth. Effects the bandwidth of the LIDAR.
grid_width = 165;                                   %simulation width for FAST
N = 3000;                                           %FIR filter length
measurment_noise = 0.1;                             %measurement noise in +-m/s

%define the root directory for absolute paths. 
%It makes MATLAB happy relative paths make MATLAB sad.

current_path = pwd;
k = strfind(current_path,'ESC_Turbine_Project');
root = strcat(current_path(1:k-1),'ESC_Turbine_Project');
fprintf('Using: %s as root directory\n\n',root);
addpath(genpath(root));

wind_cache_path = strcat(root,'\Wind_Simulators\Wind_Cache');

LIDAR_wind_path = wind_cache_path;
[tmp_filepath,tmp_name,tmp_ext] = fileparts(windfile_name);
LIDAR_wind_name = sprintf("LIDAR_wind_%s",tmp_name);

t=1:t_s:TMax; 
sprintf("%s/%s",wind_cache_path,windfile_name)
[velocity, y, z, nz, ny, dz, dy, dt, zHub, z1, SummVars,mffws] = ...
    readBLgrid(convertStringsToChars(sprintf("%s/%s",wind_cache_path,windfile_name))); %NOTE: modified from Jonkmans script to return mffws

u_inf = zeros(numel(t),1);

mean = 0;
for i = 1:numel(t)

    u_inf(i,1) = LIDAR(velocity, y, z, nz, ny, dt, zHub,LIDAR_focal_distance,grid_width,mffws,t(i),LIDAR_angle,num_lidar_pts,measurment_noise); %TO DO: NEED MFFWS
    mean = mean+u_inf(i,1);
end
mean = mean/numel(t); %get average windspeed. TO DO: Consider doing this periodically with a moving average filter or similar.

% Now filter the scanned measurements

omega_c = -mean*log(0.5)/(2*rayleight_length); % cut off frequency in rad/s
wc = omega_c*t_s;                              % cut off freq in rad/sample

u_inf_filtered = FIR_lowpass(N, wc, u_inf);
lidar_wind_vals = timeseries(u_inf_filtered,t);
save(strcat(LIDAR_wind_path,'/',LIDAR_wind_name,'.mat'),'lidar_wind_vals','-v7.3');

%%
function u_lidar = LIDAR(velocity, y, z, nz, ny, dt, zHub,focal_distance,grid_width,mffws,time,LIDAR_angle,num_lidar_pts,measurment_noise) 

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

function filtered_signal = FIR_lowpass(N, wc, signal) 
% Takes a window length N (even!), a cut off frequency wc in rad/sample and a signal and
% produces the low-pass filtered signal (non-causal)

h_filter = zeros(N + 1, 1); % index 1 is k = -N/2

% Impulse response for ideal low-pass filter
for i = 1:N+1
    if (i-1) == N/2
        h_filter(i) = wc/pi;
    else
        h_filter(i) = sin(wc*(i-1-N/2))/(pi*(i- 1 -N/2));
    end
end

num_samples = numel(signal);
filtered_signal = zeros(num_samples, 1);

for i = 1:num_samples
    for j = -N/2:N/2
        % if x[i - j] index is too small or too large..
        if (i - j) < 1 || (i-j) > num_samples
            break
        else
            filtered_signal(i) = filtered_signal(i) + h_filter(j+1+N/2)*signal(i - j);
        end
    end
end

end