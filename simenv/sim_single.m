function [] = sim_single(sim_name, wind_scenario_path, scenario_n, turbine_model_path, ...
    controller_name, observer_name)

    [simin, mdlcpy_path] = gen_simin(wind_scenario_path, scenario_n, turbine_model_path,...
        controller_name, observer_name);
    
    simOut = sim(simin);

    s = what(turbine_model_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    turbine_model_name = path_parts{end};
    
    mkdir(strcat('Results\', sim_name));
    save_results(mdlcpy_path, turbine_model_name, simOut,...
        strcat('Results\', sim_name, '\', sim_name, '_', string(scenario_n), '.mat'));
end

