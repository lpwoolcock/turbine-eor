function [simin, mdlcpy_path] = gen_simin(wind_scenario_path, n, turbine_model_path, ...
    controller_name, observer_name)
    
    mdlcpy_path = strcat(tempname, '\mdl', string(n), '\');

    mkdir(mdlcpy_path);
    
    s = what(turbine_model_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    turbine_model_name = path_parts{end};
    turbine_model_path = strcat(s.path, '\');
    
    copyfile(strcat(turbine_model_path, '*'), mdlcpy_path);
    
    load(strcat(mdlcpy_path, turbine_model_name, '.mat'));
    load(strcat(mdlcpy_path, turbine_model_name, '_CP.mat'));
    
    s = what(wind_scenario_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    wind_scenario_name = path_parts{end};
    wind_scenario_path = strcat(s.path, '\');
    
    load(strcat(wind_scenario_path, wind_scenario_name, '.mat'));
    
    model = read_text(strcat(mdlcpy_path, model_filename));
    model = strrep(model, '<<TMAX>>', string(time));
    model = strrep(model, '<<DT>>', string(DT));
    write_text(strcat(mdlcpy_path, model_filename), model);
    
    inflow = read_text(strcat(mdlcpy_path, inflow_filename));
    inflow = strrep(inflow, '<<WIND>>', strcat(wind_scenario_path, turbsim_filenames{n}));
    write_text(strcat(mdlcpy_path, inflow_filename), inflow);
    
    s = what(mdlcpy_path);
    model_file = strcat(s.path, '\', model_filename);
    
    lidar_data = readmatrix(strcat(wind_scenario_path, lidar_filenames{n}));
    lidar_data = timeseries(lidar_data(:,2), lidar_data(:,1));
    
    simin = Simulink.SimulationInput('Top_Model');
    simin = simin.setVariable('lidar_data', lidar_data);
    simin = simin.setVariable('TMax', time);
    simin = simin.setVariable('DT', DT);
    
    simin = simin.setVariable('g', gearbox_ratio);
    simin = simin.setVariable('omega', omega);
    simin = simin.setVariable('zeta', zeta);
    simin = simin.setVariable('R', R);
    simin = simin.setVariable('rho', rho);
    simin = simin.setVariable('Omega_rated', Omega_rated);
    
    simin = simin.setVariable('C_d', C_d);
    simin = simin.setVariable('K_d', K_d);
    simin = simin.setVariable('J_g', J_g);
    simin = simin.setVariable('J_r', J_r);
    simin = simin.setVariable('FAST_InputFileName', model_file);
    
    simin = simin.setVariable('CP', CP);
    simin = simin.setVariable('CP_lambda', CP_lambda);
    simin = simin.setVariable('CP_theta', CP_theta);
    [CP_theta_grid, CP_lambda_grid] = meshgrid(CP_theta, CP_lambda);
    simin = simin.setVariable('CP_lambda_grid', CP_lambda_grid);
    simin = simin.setVariable('CP_theta_grid', CP_theta_grid);
    [CP_star, theta_star, lambda_star] = CP_find_max(CP, CP_lambda, CP_theta);
    simin = simin.setVariable('CP_star', CP_star);
    simin = simin.setVariable('theta_star', theta_star);
    simin = simin.setVariable('lambda_star', lambda_star);
    
    %% Set up controller block
    
    simin = simin.setBlockParameter('Top_Model/Controller', 'ModelFile', controller_name);
    simin = feval(strcat(controller_name, '_init'), simin, turbine_model_path);
    simin = simin.setBlockParameter('Top_Model/Observer', 'ModelFile', observer_name);
    simin = feval(strcat(observer_name, '_init'), simin, turbine_model_path);
end

