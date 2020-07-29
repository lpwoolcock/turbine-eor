% make sure the FASTv8\bin directory is in the MATLAB path
%    (relative path names are not recommended in addpath()):
% cd ..\..\
% FASTv8_root_directory = pwd;
% cd Simulink\Samples
% addpath([ FASTv8_root_directory '\bin']);

clear;
 
generator_params;
tp = turbine_params_5MW();
DT = 0.00625;
T_cal = 60;
T_s = 0.1; % note this is 16*DT

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
FAST_InputFileName = 'NREL_Baseline.fst';
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
sim('Top_Model',[0,TMax]);


