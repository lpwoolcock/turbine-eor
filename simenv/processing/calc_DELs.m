function [DELs] = calc_DELs(results_path, moment_names, wohler_exponents, t_start)
    %calc_DELs('Results\BL5MW_1min', ["TwrBsMyt" "TwrBsMxt" "RootMyb1" "RootMxb1" "RotTorq"], [4 4 10 10 4], 2)    

    s = what(results_path);
    path_parts = split(s.path, '\');
    path_parts = path_parts(strlength(path_parts) > 0);
    results_name = path_parts{end};
    results_path = strcat(s.path, '\');
    
    load(strcat(results_path, results_name, '.mat'));
    
    N = length(windspeeds);
    M = length(moment_names);
    
    for k = 1:N
        data_struct = load(strcat(results_path, results_filenames(k)));
        DT = data_struct.Time(2) - data_struct.Time(1);
        j_start = ceil(t_start / DT);
        T = data_struct.Time(end) - data_struct.Time(j_start);
        
        for l = 1:M
            moment = data_struct.(moment_names(l));
          
            DELs(l, k) = get_average_moment(rainflow(moment(j_start:end)),...
                wohler_exponents(l), T);
        end
        
    end

end

