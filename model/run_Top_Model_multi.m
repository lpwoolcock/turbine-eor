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
%and caches them to speed up subsequent runs.
%to change wind properties except mean speed update:
%model\5MW_Baseline\Wind\turbsim\unsteady_10min_multi.inp
%If you make a change make sure to also clear the cache at 
%model\5MW_Baseline\Wind\multi_wind of files unsteady_tmp_*.bts
%to itterate over different properties update 
%line_in_file in generate_wind_file(mean_wind_speed) function.

mean_wind_speeds = [8 10]; %change this to change wind speeds simulated over; ints only sorry
num_loads = 5;                 %MyT MxT MyB MxB LSS
sim_time = 60;                 %set the simulation time, if you change this set clear cache to true for first run
start_time = 120;              %when we start calculating DELs from
clear_cache = false;           %flag to clear the wind file cache
runLIDAR = true;               %flag to run the LIDAR simulator
%% Toggle flags to turn on and off which data is plotted

plotting = true;    %toggle to turn on\off data plotting

%toggle to turn on/off which data is plotted
U_inf_flag = true;
U_inf_hat_flag = true;
M_a_flag = false;
M_g_flag = false;
phi_hat_flag = false;
Lambda_flag = false;
Lambda_star_flag = false;
theta_flag = false;
theta_c_flag = false;
power_flag = true;
dels_flag = true;

plot_flags = [U_inf_flag; U_inf_hat_flag; M_a_flag; M_g_flag; phi_hat_flag; Lambda_flag; Lambda_star_flag; theta_flag; theta_c_flag;power_flag ;dels_flag];
outdata_names = {'Uinf', 'UinfHat', 'M_a', 'M_g', 'phiHat', 'Lambda', 'Lambda*', 'theta', 'theta_c','DELs'};
del_names = {'MyT', 'MxT', 'MyB', 'MxB', 'LSS'};

loads = zeros(size(mean_wind_speeds,2),num_loads);

%% generate input files
if(clear_cache)
     delete 5MW_Baseline/Wind/multi_wind/unsteady_tmp_*;
end

parfor i=1:size(mean_wind_speeds,2)
   new_windfile_name = sprintf('Wind/multi_wind/unsteady_tmp_%d.bts',mean_wind_speeds(i));
   pause(i);
   if(~isfile(sprintf('./5MW_Baseline/%s',new_windfile_name)))
        generate_wind_file(mean_wind_speeds(i), sim_time);
   end
end

% if(runLIDAR)
%     run('LIDAR_simulator.m');
% end

for i=1:size(mean_wind_speeds,2)
    
   tmp_iw_name = sprintf('5MW_Baseline/NREL_inflowWind_tmp_%d.dat',mean_wind_speeds(i));
   copyfile('./5MW_Baseline/NRELOffshrBsline5MW_InflowWind_unsteady_multi.dat',sprintf('./%s',tmp_iw_name));
   new_windfile_name = sprintf('Wind/multi_wind/unsteady_tmp_%d.bts',mean_wind_speeds(i));
   cmd = sprintf('replace_string.exe windfile_placeholder  %s %s',new_windfile_name,tmp_iw_name);
   
   system(cmd)
   
   tmp_inp_name = sprintf('NREL_input_tmp_%d.fst',mean_wind_speeds(i));
   copyfile('NREL_Baseline_multi.fst',tmp_inp_name);
   cmd = sprintf('replace_string.exe inflow_placeholder %s %s',tmp_iw_name,tmp_inp_name);
   system(cmd)
end

%% set up model input

model = 'Top_Model';
load_system(model)
set_param(model, 'StopTime', num2str(sim_time));
save_system Top_Model Top_Model.slx
load_system(model)

simIn(1:size(mean_wind_speeds,2)) = Simulink.SimulationInput(model);

for i = 1:size(mean_wind_speeds,2)
    
   simIn(i) = simIn(i).setVariable('FAST_InputFileName',sprintf('C:\\Users\\lpwoolcock\\Documents\\turbine-eor\\model\\NREL_input_tmp_%d.fst',mean_wind_speeds(1,i))); 
   lidar_path = sprintf("5MW_Baseline/Wind/LIDAR_wind/LIDAR_wind_%d.csv", mean_wind_speeds(1,i));
   lidar_dat = csvread(lidar_path);
   simIn(i) = simIn(i).setVariable('lidar_data',  lidar_dat);
end


 
%% run the model

simOut = parsim(simIn);

for i=1:size(mean_wind_speeds,2)
    movefile(sprintf('simulink_outfile_%d.mat',i),sprintf('./Logged_Outdata/outfile_for_%d_wind.mat',mean_wind_speeds(1,i)));
end

%save and aggregate the data 
 for i=1:size(mean_wind_speeds,2)
     
     outfile_path = sprintf('NREL_input_tmp_%d.SFunc.out',mean_wind_speeds(1,i));
     avg = get_weighted_load(outfile_path, start_time);
     loads(i,:) = avg';
     movefile(outfile_path,sprintf('./Logged_Outdata/%s',outfile_path))
 end
 
 writematrix(loads,'./Logged_Outdata/DELs.csv');
 
 outfile_paths = cell(size(mean_wind_speeds,2),1);
 for i = 1:size(mean_wind_speeds,2)
    outfile_paths(i,1) = cellstr(sprintf('NREL_input_tmp_%d.SFunc.out',mean_wind_speeds(1,i)));
 end
 
 %Power = get_power(OutList,outfile_paths,mean_wind_speeds);
 %writematrix(Power,'./Logged_Outdata/Power.csv')
 
 delete NREL_input_tmp_*;
 delete 5MW_Baseline/NREL_inflowWind_tmp_*;
 

%% plot output
if(plotting)
   plot_data(plot_flags,mean_wind_speeds,outdata_names);
end

if(dels_flag)
   plot_dels(mean_wind_speeds,del_names); 
end

% if(power_flag)
%    plot_power(mean_wind_speeds); 
% end

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
    %normalise the distribution over the region simulated
    Del_eq = Del_eq/get_weibull(mean_wind_speeds(1,1)-wind_speed_diff/2,mean_wind_speeds(1,end)+wind_speed_diff/2); 
    
    writematrix(Del_eq,'./Logged_Outdata/DELs_eq.csv');

function OutList = get_OutList(outfile_path)
    opts = delimitedTextImportOptions('DataLines', [7 7], 'Delimiter', '\t', 'Whitespace', ' ');
    OutListT = readtable(outfile_path, opts);
    OutList = strtrim(table2cell(OutListT));
end
    
%calculates the dels for the last simulation run
function avg = get_weighted_load(outfile_path, t_start)

    %load the data

    Data = importdata(outfile_path,'\t',8).data;
    OutList = get_OutList(outfile_path);
    
    T = Data(:,find(contains(OutList,'Time'))); %#ok<*FNDSB,*USENS>
    k_start = find(T>t_start,1);
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
        avg(i,1) = get_average_moment(rainflow(moments(k_start:end,i)),wohler(i),T(end)-T(k_start));
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

%average moments DEL function
function sum = get_average_moment(c,m,t)

sum = 0;
secs_in_20yrs = 6.307*10^8;
nLCref = 2*10^6;
    for i=1:size(c,1)
        sum = sum + (c(i,1)*c(i,2)^m);
    end
    %weigting assume life time of 20years 
    sum = ((secs_in_20yrs*sum)/(nLCref*t))^(1/m);

end


%generate a .bts wind file with turbsim ouput in the 5MW_Baseline/Wind/multi_wind directory
function generate_wind_file(mean_wind_speed,sim_time)
    
    tol=10; %add 10 seconds to handle lidar delay
    sim_time=sim_time+tol;
    
    %update input file
  
    tmp_name = sprintf('unsteady_tmp_%d',mean_wind_speed);
    tmp_path = sprintf('5MW_Baseline\\Wind\\turbsim\\%s',tmp_name);
    %Change sim time at the top not this file directly, just so its clearer what settings
    %are being run
    copyfile('./5MW_Baseline/Wind/turbsim/unsteady_60min_multi.inp',sprintf('%s.inp',tmp_path));
    
    
    line_in_file = 21; % simulation time
    cmd = sprintf('5MW_Baseline\\Wind\\turbsim\\replace_number_on_line.exe %d %d %s.inp',line_in_file,sim_time,tmp_path);
    system(cmd)
    line_in_file = 20; % simulation time
    cmd = sprintf('5MW_Baseline\\Wind\\turbsim\\replace_number_on_line.exe %d %d %s.inp',line_in_file,sim_time,tmp_path);
    system(cmd)
    
    
    line_in_file = 36; % change this to itterate over other properties, NOTE:line indexes start at 0.
    cmd = sprintf('5MW_Baseline\\Wind\\turbsim\\replace_number_on_line.exe %d %d %s.inp',line_in_file,mean_wind_speed,tmp_path);
    system(cmd)
    
    %runturbsim
    
    cmd = sprintf('cd ./5MW_Baseline/Wind/turbsim & turbsim.exe %s.inp',tmp_name);
    system(cmd)
    
    new_path = sprintf('./5MW_Baseline/Wind/multi_wind/%s.bts',tmp_name);
    movefile(sprintf('%s.bts',tmp_path),new_path);


end
%returns the power from the generator torue and velocity since FAST is not
%outputing G power directly
function P = get_power(outfile_paths,mean_wind_speeds)
    
    OutList = get_OutList(outfile_path);
    Data = importdata(outfile_paths{1},'\t',8).data;
    T = Data(:,find(contains(OutList,'Time'))); %#ok<*FNDSB,*USENS>
    P = zeros(size(T,1),size(mean_wind_speeds,2)+1);
    P(:,1) = T;
    for i=1:size(mean_wind_speeds)

        Data = importdata(outfile_paths{i},'\t',8).data;
        Tq = Data(:,find(contains(OutList,'GenTq')));
        omega_g = Data(:,find(contains(OutList,'GenSpeed')));
        P = zeros(size(T,1),1);
        for i=1:size(T)
           P(:,i+1) = Tq.*omega_g*2*pi/60; %for some reason FAST outputs rpm 
        end
    end
    
    
end


%Plots data from the Logged_Outdata directory
function plot_data(plot_flags,windspeeds,names)

     for j = 1:size(windspeeds,2)


         Data = load(sprintf('./Logged_Outdata/outfile_for_%d_wind.mat',windspeeds(1,j))).data;
         data = Data.data;
         time = Data.time;


         for i=1:size(plot_flags,1)-2
            if(plot_flags(i))
                figure()
                plot(time,data(:,i));
                title(sprintf('%s windspeed %d',char(names(i)),windspeeds(j)))
            end
         end
     end
end

%plots the dels as a bar graph
function plot_dels(windspeeds,del_names)
 loads = csvread('./Logged_Outdata/DELs.csv');
 for i = 1:size(loads,2)
    figure()
    bar(windspeeds,loads(:,i)) 
    title(sprintf('DELs for %s',char(del_names(i))))
    xlabel windspeeds
    ylabel DEL
     
 end
     

end

%plots the power from the last simulation run
function plot_power(mean_wind_speeds)
    P = readmatrix('./Logged_Outdata/Power.csv');
    for i=1:size(mean_wind_speeds,2)
        figure()
        plot(P(:,1),P(:,i+1));
        title(sprintf('Power for windspeed %d',mean_wind_speeds(i)));
        xlabel power
        ylabel time

    end
end
