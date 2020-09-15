
%mean_wind_speeds = [12 14 22 24];  %windspeeds to simulate over comment out to inherit from workspace

t_s = 0.02;                  %sample rate of LIDAR
%sim_time = 60;                  %simulation time comment out to inherit from workspace 
LIDAR_focal_distance = 60;      %focal distance of the LIDAR
grid_width = 70;                % simulation width for FAST
N = 3000;                         % FIR filter length

t=1:t_s:sim_time; 
windfile_path = "./5MW_Baseline/Wind/multi_wind/";
LIDAR_wind_path = "./5MW_Baseline/Wind/LIDAR_wind/";


parfor j = 1:numel(mean_wind_speeds)
    fprintf("Processing Lidar for %sm/s wind",mean_wind_speeds(j))
    windfile_name = sprintf("unsteady_tmp_%d.bts",mean_wind_speeds(j));
    LIDAR_wind_name = sprintf("LIDAR_wind_%d.csv",mean_wind_speeds(j));
    [velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(convertStringsToChars(sprintf("%s%s",windfile_path,windfile_name)));
    u_inf = zeros(numel(t),1);
    for i = 1:numel(t)
    
        u_inf(i,1) = LIDAR(velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws,LIDAR_focal_distance,grid_width, t(i));

    end
    % Now filter the scanned measurements
    Bw = 87/(LIDAR_focal_distance^2); % cut off frequency in m^-1
    omega_c = 2*pi*Bw;%*mffws; % cut off frequency in rad/s
    wc = omega_c*t_s; % cut off freq in rad/sample, should be multiplied by ts ...
    
    u_inf_filtered = FIR_lowpass(N, wc, u_inf);
    
    
    writematrix([t',u_inf_filtered],sprintf("%s%s",LIDAR_wind_path,LIDAR_wind_name));
%{
    figure()
    
    hold on
    plot(t, u_inf)
    plot(t, u_inf_filtered)
    Data = load(sprintf('./Logged_Outdata/outfile_for_%d_wind.mat',windspeeds(1,j))).data;
    data = Data.data;
    time = Data.time;
    plot(time(1:end),data(1:end,1));
    legend('LIDAR unfiltered','LIDAR', 'U inf')
                
  %}       
         
 
end
%%
function u_lidar = LIDAR(velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws,focal_distance,grid_width, time) 
% Takes a turbsim file (.bts) and produces simulated wind speed measurements 
% using LIDAR at hub height, requires mean wind speed of the windfile and.
% distance ahead of the turbine of interest.


% velocity is a 4D array with dimensions in time, (U, V, W components), y,
% z positions. 
% assume odd grid number, the hub position is in the middle of the grid
%y_hub_position = idivide(1 + ny, int32(2), 'round');
%z_hub_position = idivide(1 + nz, int32(2), 'round')+1;


radius = 63;
num_lidar_pts = 24;


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

%average the mesurments(wasnt sure if this is what you had in mind here Vinny)
for i=1:num_lidar_pts
    U_avg = U_avg+U_est(i)/num_lidar_pts;
end
u_lidar = U_avg;


%hub_velocities = velocity(:,1,y_hub_position, z_hub_position);
%sample_number = 0:numel(hub_velocities)-1;
%time = dt*sample_number';

% Insert Filter Here


% LIDAR_hubheight_high = timeseries(hub_velocities, time);
% save 'LIDAR_hubheight_high.mat' -v7.3 LIDAR_hubheight_high
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