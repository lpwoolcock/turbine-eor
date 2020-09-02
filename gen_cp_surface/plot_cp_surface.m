[pitch, lambda] = meshgrid(pitch_cmds, lambda_cmds);
surf(pitch, lambda, max(0,Cp));
xlabel("\theta (rad)");
ylabel("\lambda");
zlabel("c_p");

%% For various pitch angles, plot Cp/lambda^3. We want to observe monotonicity
% issues when performing root finding for lambda

lambda = lambda_cmds';
hold on
plot(lambda, Cp(:,1)./(lambda.^3), 'LineWidth', 1, 'Color', 'r')
plot(lambda, Cp(:,5)./(lambda.^3), 'LineWidth', 1, 'Color', [1 0.6 0])
plot(lambda, Cp(:,9)./(lambda.^3), 'LineWidth', 1, 'Color', [0 0.2 1])
plot(lambda, Cp(:,13)./(lambda.^3), 'LineWidth', 1, 'Color', [0.6 0 0.8])

xlab = xlabel('Tip Speed Ratio: $\lambda$');
ylab = ylabel('$C_p(\lambda)\cdot\lambda^{-3}$');
ylim([-0.0025, 0.02])
xlim([0, 8])
leg = legend('$\theta = 0^{\circ}$', '$\theta = 4.58^{\circ}$', '$\theta = 9.17^{\circ}$', '$\theta = 13.75^{\circ}$');
% tit = title('Plot of $C_p(\lambda)\cdot\lambda^{-3}$ to $\lambda$ for varying $\theta$');
set(ylab,'Interpreter','latex');
set(xlab,'Interpreter','latex');
% set(tit,'Interpreter','latex');
set(leg, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex')

% Vertical lines indicate operating \lambda values for a given pitch
plot(7.55, 0.001104, 'LineWidth', 2, 'HandleVisibility', 'off', 'MarkerSize', 10, 'Marker', 's', 'Color', 'r');
plot(6.5728, 0.001287, 'LineWidth', 2, 'HandleVisibility', 'off', 'MarkerSize', 10, 'Marker', 's', 'Color', [1 0.6 0]);
plot(5.5833, 0.001287, 'LineWidth', 2, 'HandleVisibility', 'off', 'MarkerSize', 10, 'Marker' ,'s', 'Color', [0 0.2 1]);
plot(4.6256, 0.001287, 'LineWidth', 2, 'HandleVisibility', 'off', 'MarkerSize', 10, 'Marker' ,'s', 'Color', [0.6 0 0.8]);



%% Attempting to find operating lambda in Region 3 for various pitch angles
P_rated = 5e6;
R = 63;
rho = 1.225;
Omega_rated = 12.1*2*pi/60;
C = P_rated/(0.5*rho*pi*R^5*Omega_rated^3); % intercept for Cp/lambda^3 = C

[theta_grid, lambda_grid] = meshgrid(pitch_cmds, lambda_cmds);

theta = 0.24; 

 f = @(lambda) interp2(theta_grid, lambda_grid, Cp, theta, lambda, 'cubic')/(lambda^3) - C;

lambda_star = fzero(f, 5.5833);
 
cp_lambda_star = interp2(theta_grid, lambda_grid, Cp, theta, lambda_star, 'cubic')/(lambda_star^3);