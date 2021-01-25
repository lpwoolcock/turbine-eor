function [] = plot_DELs(windspeeds, scenario_names, DEL_names, DELs)
    % DELs is NxMxO
    % N is no. DELs, M is no. windspeeds, O is no. scenarios
    
    O = length(scenario_names);
    N = length(DEL_names);
    M = length(windspeeds);
    
    for n = 1:N
        
        data = zeros(M, O);
        for o = 1:O
            data(:, o) = DELs(n, :, o);
        end
        
        figure;
        bar(windspeeds, data);
        title(DEL_names{n});
        legend(scenario_names);
    end
end

