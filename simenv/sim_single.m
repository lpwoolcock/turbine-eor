function [] = sim_single(sim_name, wind_scenario_name, scenario_n, turbine_model_name, ...
    controller_name, observer_name)

    simin = gen_simin(wind_scenario_name, scenario_n, turbine_model_name,...
        controller_name, observer_name);
    
    sim(simin);
    
    mkdir(strcat('Results/', sim_name));
    
    %{
    mdlcpy_path = strcat('simtmp/mdl', string(scenario_n), '/');
    load(strcat(mdlcpy_path, turbine_model_name, '.mat'));
    s = split(model_filename, '.');
    fstname = s(1);
    
    
    [data, data_names, ~, ~, ~] = ReadFASTbinary(strcat('simtmp/mdl', ...
        string(scenario_n), '/', fstname, '.SFunc.outb'));
    
    for k = 1:length(data)
        data_struct(data_names(k)) = data(k,:);
    end
    
    save(strcat('Results/', sim_name, 
    %}
end

