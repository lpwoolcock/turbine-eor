function [] = gen_wind_scenario(path, turbsim_template_filename, windspeeds, time)   
    template_text = read_text(turbsim_template_filename);
    K = length(windspeeds);
    
    N_lidar = 20000;
    t_s_lidar = 0.02;
    t_fudge = N_lidar * t_s_lidar / 2;
    
    mkdir(path);
    s = what(path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    name = path_parts{end};
    path = strcat(s.path, '\');
    
    turbsim_filenames = cell(K,1);
    
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
    
    uniform_filenames = cell(K,1);
    parfor k = 1:K
        uniform_filenames{k} = sprintf('%s_uniform_%d.csv', name, k);
        REWS_simulator(strcat(path, turbsim_filenames{k}, '.bts'), strcat(path, uniform_filenames{k}), 63); %hmm
        
        fprintf('[%s] [%d/%d] Generated uniform inflow file "%s"\n', string(datetime), k, K, uniform_filenames{k});
    end
    
    turbsim_filenames = strcat(turbsim_filenames, '.bts');
    save(strcat(path, name, '.mat'), 'turbsim_filenames', 'lidar_filenames', 'uniform_filenames', 'windspeeds', 'time', 't_s_lidar');
    fprintf('[%s] Wind scenario generation for "%s" complete.\n', string(datetime), name);
end