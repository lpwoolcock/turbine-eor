function [H] = plot_bargraph(data, windspeeds, scenario_names, ylabel_text)

    % data is NxM, N=no. windspeeds, M=no. scenarios
    
    H = figure;
    figure(H);
    
    bar(windspeeds, data);
    xlabel('Mean Windspeed (m/s)');
    ylabel(ylabel_text);
    legend(scenario_names);
    
end

