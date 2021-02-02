function [power] = calc_power(results_path, t_start)
    %calc_DELs('Results\BL5MW_1min', ["TwrBsMyt" "TwrBsMxt" "RootMyb1" "RootMxb1" "RotTorq"], [4 4 10 10 4], 2)    

    s = what(results_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    results_name = path_parts{end};
    results_path = strcat(s.path, '\');
    
    load(strcat(results_path, results_name, '.mat'));
    N = length(windspeeds);
    
    power = zeros(N,1);
    
    for k = 1:N
        data_struct = load(strcat(results_path, results_filenames(k)));
        DT = data_struct.Time(end)/length(data_struct.Time);
        j_start = ceil(t_start / DT);
        
        % from rpm to rad/s
        omega_g = data_struct.GenSpeed(j_start:end) * pi / 30;
        % from kNm to Nm
        tau_g = data_struct.GenTq(j_start:end) * 1000;
    
        power(k) = mean(omega_g .* tau_g);
        
    end
    


end

