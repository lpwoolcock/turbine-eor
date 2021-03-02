function [simin_out] = Immersion_Invariance_init(simin, turbine_model_path)
    simin_out = simin.setVariable('II_gamma', 30);
    simin_out = simin_out.setVariable('II_vx0', 0);
    simin_out = simin_out.setVariable('II_phi0', 0);
    % 20 -> 2.6
    simin_out = simin_out.setVariable('II_theta_tau', 1.7);
    
    simin_out = Pitch_Rate_init(simin_out, turbine_model_path);
end

