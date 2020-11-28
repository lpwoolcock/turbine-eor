function [] = gen_wind_scenario(name, turbsim_template_filename, windspeeds, time)   
    template_text = read_text(turbsim_template_filename);
    K = length(windspeeds);
    
    N_lidar = 20000;
    t_s_lidar = 0.02;
    t_fudge = N_lidar * t_s_lidar / 2;
    
    path = strcat(name, '/');
    
    turbsim_filenames = cell(K,1);
    mkdir(name);
    
    fprintf('[%s] Wind scenario generation for "%s" starting.\n', string(datetime), name);
    for k = 1:K
        turbsim_inp = strrep(template_text, '<<WIND>>', num2str(windspeeds(k)));
        turbsim_inp = strrep(turbsim_inp, '<<TIME>>', num2str(time + t_fudge));
        
        turbsim_filenames{k} = sprintf('%s_turbsim_%d', name, k);
        write_text(strcat(path, turbsim_filenames{k}, '.inp'), turbsim_inp);
    end
    
    parfor k = 1:K
        command = sprintf('@bin\\TurbSim64.exe %s', strcat(path, turbsim_filenames{k}, '.inp'));
        [~,~] = system(command);
        delete(strcat(path, turbsim_filenames{k}, '.inp'));
        delete(strcat(path, turbsim_filenames{k}, '.sum'));
        
        fprintf('[%s] [%d/%d] Generated AeroDyn inflow file "%s"\n', string(datetime), k, K, strcat(turbsim_filenames{k}, '.bts'));
    end

    lidar_filenames = cell(K,1);
    parfor k = 1:K
        lidar_filenames{k} = sprintf('%s_lidar_%d.csv', name, k);
        LIDAR_simulator(strcat(path, turbsim_filenames{k}, '.bts'), strcat(path, lidar_filenames{k}), time + t_fudge, t_s_lidar, N_lidar);
        
        fprintf('[%s] [%d/%d] Generated LIDAR data file "%s"\n', string(datetime), k, K, lidar_filenames{k});
    end
    
    turbsim_filenames = strcat(turbsim_filenames, '.bts');
    save(strcat(name, '/', name, '.mat'), 'turbsim_filenames', 'lidar_filenames', 'windspeeds');
    fprintf('[%s] Wind scenario generation for "%s" complete.\n', string(datetime), name);
end