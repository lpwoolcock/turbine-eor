% make sure the FASTv8\bin directory is in the MATLAB path
%    (relative path names are not recommended in addpath()):
% cd ..\..\
% FASTv8_root_directory = pwd;
% cd Simulink\Samples
% addpath([ FASTv8_root_directory '\bin']);


clear;

%% wind speeds input
%This script allows for the batch simulation over a range of wind speeds
%it generates apropriate wind files, using turbsim automatically,
%and catchs them to speed up subsequent runs.
%to change wind properties except mean speed update:
%model\5MW_Baseline\Wind\turbsim\unsteady_10min_multi.inp
%If you make a change make sure to also clear the catch at 
%model\5MW_Baseline\Wind of files unsteady_10min_*.bts
%to itterate over different properties update 
%line_in_file in generate_wind_file(mean_wind_speed) function.

mean_wind_speeds = [2 4 6 8]; %change this to change wind speeds simulated over; ints only sorry
num_loads = 5;                %MyT MxT MyB MxB LSS

%% Initialize System
 
generator_params;
tp = turbine_params_5MW();
DT = 0.00625;
T_mean = 60;         % Time to calculate mean windspeed
T_cal = T_mean + 10; % Total calibration time (i.e additional time required for parameter estimation)
T_s = 0.1;           % note this is 16*DT

% baseline control region 2
% units of MNm/(rads^-1)^2
K_T = 0.5e-6*pi*tp.rho*tp.R^5*tp.C_p_star/(tp.lambda_star^3);

% no. of samples in wind exosystem
N_w = 3;

% system parameters
R = 63;
rho = 1.225;
g = 97;
J_r = 11.77e6;
J_g = 534;
K_d = 867e6;
C_d = 6.2e6;

% parameters required for the S-Function block:
FAST_InputFileName = 'NREL_Baseline_multi.fst';
TMax = 600;

% Initial conditions
Omega_r_0 = 1;
phi_0 = 0;
Omega_g_0 = 1;

%% Simple Kalman Filter
% scale factor to handle numerical issues
s = 1e6;
m = 1e3;
% Augmented Plant Model
A_aug = [-C_d/J_r -K_d/(m*J_r) C_d/J_r s/J_r; m 0 -m 0; C_d/(g*J_g) K_d/(m*g*J_g) -C_d/(g*J_g) 0; 0 0 0 0];
B_aug = [0; 0; -s/(g*J_g); 0];
C_aug = [1 0 0 0; 0 0 1 0];
D_aug = [0; 0];
rank(obsv(A_aug,C_aug));
[ABAR,BBAR,CBAR,T,K] = obsvf(A_aug,B_aug,C_aug);

%% Tuning parameters
% Measurement uncertainty
alpha = 0.000015;
V = alpha*eye(2);

% Model uncertainty - Don't tune unless necessary
G = [1; 0; 1; 3];
W = 1;

[KEST, L_kalman, P] = kalman(ss(A_aug, [B_aug G], C_aug, 0), W, V);

ini_torque = 0; %used in state estimator

%% PI Kalman Filter
% scale factor to handle numerical issues
s = 1e6;
m = 1e3;
% Augmented Plant Model
A_aug_PI = [-C_d/J_r -K_d/(m*J_r) C_d/J_r s/J_r; m 0 -m 0; C_d/(g*J_g) K_d/(m*g*J_g) -C_d/(g*J_g) 0; 0 0 0 0];
B_aug_PI = [0; 0; -s/(g*J_g); 0];
C_aug_PI = [0 0 1 0];
D_aug_PI = [0];

% Non-augmented Model
A_PI = [-C_d/J_r -K_d/(m*J_r) C_d/J_r; m 0 -m; C_d/(g*J_g) K_d/(m*g*J_g) -C_d/(g*J_g)];
B_G_PI = [0; 0; -s/(g*J_g)];
B_A_PI = [s/J_r; 0; 0];
C_PI = [0 0 1];
D_PI = [0];

% Tuning parameters
% Measurement uncertainty
alpha = 0.000015; %Same as simple kalman

V = alpha;

% Model uncertainty - Don't tune unless necessary
G = [1; 0; 1; 3];
W = 1;

[KEST, L_kalman_PI, P] = kalman(ss(A_aug_PI, [B_aug_PI G], C_aug_PI, 0), W, V);

% PI parameters
K_i = L_kalman_PI(4);
K_p = 50;

% Reset kalman gains
L_kalman_PI = L_kalman_PI(1:3);

ini_torque = 0; %used in state estimator

%% Reduced Order Kalman Filter
s = 1e6;
m = 1e3;

A_aa = [-C_d/J_r C_d/J_r; C_d/(g*J_g) -C_d/(g*J_g)];
A_ab = [-K_d/(m*J_r) s/J_r; K_d/(m*g*J_g) 0];
A_ba = [m -m; 0 0]; 
A_bb = [0 0; 0 0];
B_a = [0; -s/(g*J_g)];
B_b = [0; 0];

% Measurement uncertainty
alpha = 0.05;
V = alpha*eye(2);

% Model uncertainty - Don't tune unless necessary
G = [5; 4.5];
W = 1;

[KEST, L_kal_red, P] = kalman(ss(A_bb, [eye(2) G], A_ab, 0), W, V);

ini_torque = 0; %used in state estimator

%% Simple Reduced Order Observer

L_unknown = 10; %s = -10 pole


%% Immersion and Invariance 
% Tuning parameter
gamma = 500;
ini_wind_speed = 0;

%% Region 3 Pitch Rate Observer
omega = 2*pi;
zeta = 0.70;
A_pitch = [0 1; omega^2 -2*zeta*omega];
B_pitch = [0; omega^2];
C_pitch = [1 0];

% Measurement uncertainty
alpha = 0.05;
V = alpha; %tune this

% Model uncertainty - Don't tune unless necessary
G = [0; 1];
W = 1;

[KEST, L_kal_pitch, P] = kalman(ss(A_pitch, [B_pitch G], C_pitch, 0), W, V);

ini_pitch = 0; %used in state estimator


%% run the model
loads = zeros(size(mean_wind_speeds,2),num_loads);

%itterate over the wind speeds running the model and generating
%wind files as necissary
for i=1:size(mean_wind_speeds,2)
    
    run_sim(mean_wind_speeds(1,i),TMax);
    avg = get_weighted_load(OutList);
    loads(i,:) = avg';
end


%save the output
writematrix(loads,'DELs.csv');

%% weighted by wind freq DELs 
Del_eq=zeros(1,num_loads);
if(size(mean_wind_speeds,2)>1)
    wind_speed_diff = abs(mean_wind_speeds(2)-mean_wind_speeds(1));
else
    wind_speed_diff = 2; %default value
end

for i=1:size(mean_wind_speeds,2)
    Del_eq = Del_eq + weight_by_windspeed(mean_wind_speeds(1,i),wind_speed_diff,loads(i,:));
    
end
    Del_eq = Del_eq/size(mean_wind_speeds,2)

%calculates the dels for the last simulation run
function avg = get_weighted_load(OutList)

    %load the data
    
    Data = importdata('NREL_Baseline_multi.SFunc.out','\t',8).data;

    T = Data(:,find(contains(OutList,'Time'))); %#ok<*FNDSB,*USENS>
    MyT = Data(:,find(contains(OutList,'TwrBsMyt')));
    MxT = Data(:,find(contains(OutList,'TwrBsMxt')));
    MyB = Data(:,find(contains(OutList,'RootMyb1')));
    MxB = Data(:,find(contains(OutList,'RootMxb1')));
    LSS = Data(:,find(contains(OutList,'RotTorq')));

    moments = [MyT MxT MyB MxB LSS];   
    wohler = [4 4 10 10 4];             %weights to calculate the DELs

    size(moments,2);
    avg = zeros(size(moments,2),1);
    
    for i=1:size(moments,2)
        avg(i,1) = get_average_moment(rainflow(moments(1:end,i)),wohler(i),T(end)-T(1));
    end

end 

%weights the del by the probability of that wind speed
function del = weight_by_windspeed(speed,dis_mes,loads)
    if(speed>0)
      w = get_weibull(speed-dis_mes/2,speed+dis_mes/2);
    else
      w = get_weibull(0,speed+dis_mes/2);
      
    end
    del = w*loads;
end

%gets the discrete approximation between lower and upper bounds
function y = get_weibull(lower,upper) 
  y = weibull_CDF(upper)-weibull_CDF(lower);
end

%its what you think it is
function p = weibull_CDF(x)
    C = 12; %https://d-nb.info/1118369653/34 source for these numbers
    k = 2; %parrameters for the wind distribution
    
    p = 1-exp(-(x/C)^k);
end

%average moments
function sum = get_average_moment(c,m,t)

sum = 0;

    for i=1:size(c,1)
        sum = sum + c(i,1)*c(i,2)^m/t;
    end
    sum = sum^(1/m);

end


%generate a .bts wind file with turbsim ouput in the 5MW_Baseline/Wind directory
function generate_wind_file(mean_wind_speed)
    
    %update input file
    line_in_file = 36; % change this to itterate over other properties, NOTE:line indexes start at 0.
    cmd = sprintf('cd ./5MW_Baseline/Wind/turbsim & replace_number_on_line.exe %d %d unsteady_10min_multi.inp',line_in_file,mean_wind_speed);
    system(cmd)
    

    %runturbsim
    system('cd ./5MW_Baseline/Wind/turbsim & turbsim.exe unsteady_10min_multi.inp')

    %move and rename file
    new_path = sprintf('./5MW_Baseline/Wind/unsteady_10min_%d.bts',mean_wind_speed);
    movefile('./5MW_Baseline/Wind/turbsim/unsteady_10min_multi.bts',new_path);

end

%run a simulation for the given mean_wind_speed
function run_sim(mean_wind_speed,TMax)

    %update the input file
    new_name = sprintf('Wind/unsteady_10min_%d.bts',mean_wind_speed);
    cmd = sprintf('replace_string.exe windfile_placeholder %s ./5MW_Baseline/NRELOffshrBsline5MW_InflowWind_unsteady_multi.dat',new_name);
    system(cmd)
    new_path = sprintf('./5MW_Baseline/%s',new_name);
    if(~isfile(new_path))
        generate_wind_file(mean_wind_speed)
    end
    
    %run simulation
    sim('Top_Model',[0,TMax]);
    
    %tidy up
    cmd = sprintf('replace_string.exe %s  windfile_placeholder ./5MW_Baseline/NRELOffshrBsline5MW_InflowWind_unsteady_multi.dat',new_name);
    system(cmd)
end


