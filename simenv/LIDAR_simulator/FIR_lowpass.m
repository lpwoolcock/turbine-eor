function filtered_signal = FIR_lowpass(N, wc, signal) 
    % Takes a window length N (even!), a cut off frequency wc in rad/sample and a signal and
    % produces the low-pass filtered signal (non-causal)

    h_filter = zeros(N + 1, 1); % index 1 is k = -N/2

    % Impulse response for ideal low-pass filter
    for i = 1:N+1
        if (i-1) == N/2
            h_filter(i) = wc/pi;
        else
            h_filter(i) = sin(wc*(i-1-N/2))/(pi*(i- 1 -N/2));
        end
    end

    num_samples = numel(signal);
    filtered_signal = zeros(num_samples, 1);

    for i = 1:num_samples
        for j = -N/2:N/2
            % if x[i - j] index is too small or too large..
            if (i - j) < 1 || (i-j) > num_samples
                break
            else
                filtered_signal(i) = filtered_signal(i) + h_filter(j+1+N/2)*signal(i - j);
            end
        end
    end
end