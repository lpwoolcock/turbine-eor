% BEWARE: this makes a big mess in your folder

clear;

GenEff   =  100.0;          % - Generator efficiency [ignored by the Thevenin and user-defined generator models] (%)
GBRatio  =   22.5;          % - Gearbox ratio (-)
SIG_SlPc =    1.5125;       % - Rated generator slip percentage [>0] (%)              Now HSS side!
SIG_SySp = 1200.0;          % - Synchronous (zero-torque) generator speed [>0] (rpm)  Now HSS side!
SIG_RtTq = 13679;          % - Rated torque [>0] (N-m)                               Now HSS side!
SIG_PORt =    2.0;          % - Pull-out ratio (Tpullout/Trated) [>1] (-)% 
SIG_SySp = SIG_SySp*pi/30;  % convert to rad/s
SIG_RtSp = SIG_SySp*(1.0+0.01*SIG_SlPc);
SIG_POS1=SIG_PORt*(SIG_RtSp-SIG_SySp);
SIG_POTq=SIG_RtTq*SIG_PORt;
SIG_Slop=SIG_RtTq/(SIG_RtSp - SIG_SySp);

Ts = 40;
rho = 1.225;
R = 63;
model = "NREL_5MW_Controller";
load_system(model);

lambda_cmds = 0:2:20;
pitch_cmds = 0;
%pitch_cmds = 0:0.02:0.54;

n = length(lambda_cmds);
m = length(pitch_cmds);
Cp = zeros(n,m);

lambda_idx = 1:n;
pitch_idx = 1:m;

lambda_col = reshape(repmat(lambda_idx,m,1),n*m,1);
pitch_col = repmat(pitch_idx.',n,1);

idx = [lambda_col pitch_col];
N = length(idx);

simIn(1:N) = Simulink.SimulationInput(model);

U_inf = 15;

for k=1:N
    simIn(k) = simIn(k).setVariable('GenEff',GenEff);
    simIn(k) = simIn(k).setVariable('GBRatio',GBRatio);
    simIn(k) = simIn(k).setVariable('SIG_SlPc', SIG_SlPc);    
    simIn(k) = simIn(k).setVariable('SIG_RtTq', SIG_RtTq);  
    simIn(k) = simIn(k).setVariable('SIG_PORt', SIG_PORt);  
    simIn(k) = simIn(k).setVariable('SIG_SySp', SIG_SySp);  
    simIn(k) = simIn(k).setVariable('SIG_RtSp', SIG_RtSp);  
    simIn(k) = simIn(k).setVariable('SIG_POS1', SIG_POS1);  
    simIn(k) = simIn(k).setVariable('SIG_POTq', SIG_POTq);  
    simIn(k) = simIn(k).setVariable('SIG_Slop', SIG_Slop);  

    simIn(k) = simIn(k).setVariable('TMax', 60);
    simIn(k) = simIn(k).setVariable('rho', rho);
    simIn(k) = simIn(k).setVariable('R', R);
    
    simIn(k) = simIn(k).setVariable('rotor_cmd', lambda_cmds(idx(k,1))*U_inf*30/(R*pi));
    simIn(k) = simIn(k).setVariable('pitch_cmd', pitch_cmds(idx(k,2)));

    copyfile("NREL_Baseline.fst", pwd + "\infile_" + num2str(k) + ".fst");
    simIn(k) = simIn(k).setVariable('FAST_InputFileName', char(pwd + "\infile_" + num2str(k) + ".fst"));
end

simOut = parsim(simIn);
delete infile*;

load("OutList.mat");

for k=1:N
    OutData = simOut(k).OutData;
    Time = simOut(k).Time;
    power_data = OutData(:,strmatch('RotPwr',OutList));
    omega_data = OutData(:,strmatch('RotSpeed',OutList))*2*pi/60;
    wind_data = OutData(:,strmatch('Wind1VelX',OutList));
    P = mean(power_data(Time > Ts))*1000;
    V = mean(wind_data(Time > Ts));
    omega = mean(omega_data(Time > Ts));
    Cp(idx(k,1), idx(k,2)) = 2 * P / (pi*rho*R^2*V^3);
end
