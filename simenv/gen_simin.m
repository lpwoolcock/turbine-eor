function [simin] = gen_simin(wind_scenario_name, n, turbine_model_name, ...
    controller_name)
    
    mdlcpy_path = strcat('simtmp/mdl', string(n), '/');

    mkdir(mdlcpy_path);
    
    copyfile(strcat('FAST_Models/', turbine_model_name, '/*'), mdlcpy_path);
    
    load(strcat(mdlcpy_path, turbine_model_name, '.mat'));
    
    load(strcat('Wind_Scenarios/', wind_scenario_name, '/', wind_scenario_name, '.mat'));
    
    model = read_text(strcat(mdlcpy_path, model_filename));
    model = strrep(model, '<<TMAX>>', string(time));
    model = strrep(model, '<<DT>>', string(DT));
    write_text(strcat(mdlcpy_path, model_filename), model);
    
    s = what(strcat('Wind_Scenarios/', wind_scenario_name));
    wind_path = s.path;
    
    inflow = read_text(strcat(mdlcpy_path, inflow_filename));
    inflow = strrep(inflow, '<<WIND>>', strcat(wind_path, '\', turbsim_filenames{n}));
    write_text(strcat(mdlcpy_path, inflow_filename), inflow);
    
    s = what(mdlcpy_path);
    model_file = strcat(s.path, '\', model_filename);
    
    lidar_data = readmatrix(strcat(wind_path, '\', lidar_filenames{n}));
    lidar_data = timeseries(lidar_data(:,2), lidar_data(:,1));
    
    simin = Simulink.SimulationInput('Top_Model');
    simin = simin.setVariable('lidar_data', lidar_data);
    simin = simin.setVariable('TMax', time);
    simin = simin.setVariable('DT', DT);
    
    simin = simin.setVariable('g', gearbox_ratio);
    simin = simin.setVariable('omega', omega);
    simin = simin.setVariable('zeta', zeta);
    simin = simin.setVariable('FAST_InputFileName', model_file);
    
    %% Set up controller block
    
    simin = simin.setBlockParameter('Top_Model/Controller', 'ModelFile', controller_name);
    simin = feval(strcat(controller_name, '_init'), simin);
end

