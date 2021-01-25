function [] = sim_multi(sim_name, wind_scenario_path, wind_type,... 
    turbine_model_path, controller_name, observer_name)
    
    s = what(wind_scenario_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    wind_scenario_name = path_parts{end};
    wind_scenario_path = strcat(s.path, '\');
    
    load(strcat(wind_scenario_path, wind_scenario_name, '.mat'));
    
    N = length(windspeeds);
    
    for k=1:N
        [simIn(k), model_path(k)] = gen_simin(wind_scenario_path, wind_type,...
            k, turbine_model_path, controller_name, observer_name);
    end
    
    simOut = parsim(simIn);
    
    s = what(turbine_model_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    turbine_model_name = path_parts{end};
    
    mkdir(strcat('Results\', sim_name));
    for k=1:N
        results_filenames(k) = strcat(sim_name, '_results_', string(k), '.mat');
        save_results(model_path(k), turbine_model_name, simOut(k),...
            strcat('Results\', sim_name, '\', results_filenames(k)));
    end
    
    save(strcat('Results\', sim_name, '\', sim_name, '.mat'), 'windspeeds',...
        'results_filenames', 'wind_scenario_path', 'turbine_model_path',...
        'controller_name', 'observer_name');
end

