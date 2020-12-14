function [simin_out] = Pitch_Rate_init(simin)
    %% Region 3 Pitch Rate Observer
    omega = 2*pi;
    zeta = 0.70;
    A_pitch = [0 1; -omega^2 -2*zeta*omega];
    B_pitch = [0; omega^2];
    C_pitch = [1 0];

    % Measurement uncertainty
    alpha = 0.05;
    V = alpha; %tune this

    % Model uncertainty - Don't tune unless necessary
    G = diag(0,1);
    W = eye(2);

    [~, L_kal_pitch, ~] = kalman(ss(A_pitch, [B_pitch G], C_pitch, 0), W, V);

    simin_out = simin.setVariable('PR_L', L_kal_pitch);
    simin_out = simin_out.setVariable('PR_A', A_pitch);
    simin_out = simin_out.setVariable('PR_B', B_pitch);
    simin_out = simin_out.setVariable('PR_C', C_pitch);
    simin_out = simin_out.setVariable('PR_theta0', 0);
end

