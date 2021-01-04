function [] = save_results(mdlcpy_path, mdl_name, simOut, results_path)

    load(strcat(mdlcpy_path, mdl_name, '.mat'));
    
    s = split(model_filename, '.');
    fast_name = s(1);
    
    [data, data_names, ~, ~, ~] = ReadFASTbinary(strcat(mdlcpy_path, fast_name, '.SFunc.outb'));
    
    for k = 1:size(data, 2)
        data_struct.(string(data_names(k))) = data(:,k);
    end
    
    len = size(data, 1);
    
    data_struct.U_inf_hat = reshape(simOut.U_inf_hat.Data, [len 1]);
    
    data_struct.Omega_r_hat = reshape(simOut.Omega_r_hat.Data, [len 1]);
    data_struct.phi_hat = reshape(simOut.phi_hat.Data, [len 1]);
    data_struct.Omega_g_hat = reshape(simOut.Omega_g_hat.Data, [len 1]);
    data_struct.theta_hat = reshape(simOut.theta_hat.Data, [len 1]);
    data_struct.theta_dot_hat = reshape(simOut.theta_dot_hat.Data, [len 1]);
    
    save(results_path, '-struct', 'data_struct');
end

