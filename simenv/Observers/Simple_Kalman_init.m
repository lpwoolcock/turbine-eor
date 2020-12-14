function [simin_out] = Simple_Kalman_init(simin)
    
    %% Simple Kalman Filter
    % scale factor to handle numerical issues
    s = 1e6;
    m = 1e3;
    % Augmented Plant Model
    A_aug = [-C_d/J_r -K_d/(m*J_r) C_d/J_r s/J_r; m 0 -m 0; C_d/(g*J_g) K_d/(m*g*J_g) -C_d/(g*J_g) 0; 0 0 0 0];
    B_aug = [0; 0; -s/(g*J_g); 0];
    C_aug = [1 0 0 0; 0 0 1 0];

    %% Tuning parameters

    % Measurement uncertainty
    alpha = 0.000015;
    V = alpha*eye(2);

    % Model uncertainty - Don't tune unless necessary
    G = diag([1.4 0 1.4 4.5]);
    W = eye(4);

    [~, L_kalman, ~] = kalman(ss(A_aug, [B_aug G], C_aug, 0), W, V);
    
    simin_out = simin.setVariable('SK_L', L_kalman);
    simin_out = simin_out.setVariable('SK_A', A_aug);
    simin_out = simin_out.setVariable('SK_B', B_aug);
    simin_out = simin_out.setVariable('SK_C', C_aug);
    simin_out = simin_out.setVariable('SK_x0', zeros(4,1));
    simin_out = simin_out.setVariable('SK_rho', 1.225);
    simin_out = simin_out.setVariable('SK_R', 63);
    simin_out = Pitch_Rate_init(simin_out);
end

