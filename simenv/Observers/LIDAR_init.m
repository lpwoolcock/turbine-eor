function [simin_out] = LIDAR_init(simin, turbine_model_path)
    simin_out = simin.setVariable('LIDAR_phi0', 0);
    
    simin_out = Pitch_Rate_init(simin_out, turbine_model_path);
end

