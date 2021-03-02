function [omega_r_std] = calc_omega_r_std(results_path, t_start, t_end)
    %calc_DELs('Results\BL5MW_1min', ["TwrBsMyt" "TwrBsMxt" "RootMyb1" "RootMxb1" "RotTorq"], [4 4 10 10 4], 2)    

    s = what(results_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    results_name = path_parts{end};
    results_path = strcat(s.path, '\');
    
    load(strcat(results_path, results_name, '.mat'));
    N = length(windspeeds);
    
    omega_r_std = zeros(N,1);
    
    for k = 1:N
        data_struct = load(strcat(results_path, results_filenames(k)));
        DT = data_struct.Time(end)/length(data_struct.Time);
        j_start = ceil(t_start / DT);
        
        if nargin < 3
            j_end = length(Time);
        else
            j_end = ceil(t_end / DT);
        end
        
        % from rpm to rad/s
        omega_r = data_struct.RotSpeed(j_start:j_end) * pi / 30;
    
        omega_r_std(k) = std(omega_r);
        
    end
    
end
