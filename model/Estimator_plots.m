%% Observer Plots

figure(1)
hold on
plot(Ma, 'LineWidth', 0.75, 'Color', 'r');
plot(Ma_hat, 'LineWidth', 1, 'Color', 'b');

xlab = xlabel('time (seconds)');
ylab = ylabel('aerodynamic torque (MNm)');
leg = legend('$M_a$', '$\hat{M}_a$');
ylim([0.5, 4])
% tit = title('Plot of $C_p(\lambda)\cdot\lambda^{-3}$ to $\lambda$ for varying $\theta$');
set(ylab,'Interpreter','latex');
set(xlab,'Interpreter','latex');
% set(tit,'Interpreter','latex');
set(leg, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex')
hold off


figure(2)
hold on
plot(U_inf, 'LineWidth', 0.75, 'Color', 'r');
plot(U_inf_hat, 'LineWidth', 1, 'Color', 'b');

xlab = xlabel('time (seconds)');
ylab = ylabel('horizontal wind speed (m/s)');
leg = legend('hub-height wind speed: $U_\infty$', 'estimated wind speed: $\hat{U}_\infty$');

% tit = title('Plot of $C_p(\lambda)\cdot\lambda^{-3}$ to $\lambda$ for varying $\theta$');
set(ylab,'Interpreter','latex');
set(xlab,'Interpreter','latex');
% set(tit,'Interpreter','latex');
set(leg, 'Interpreter', 'latex');
set(gca,'TickLabelInterpreter','latex')