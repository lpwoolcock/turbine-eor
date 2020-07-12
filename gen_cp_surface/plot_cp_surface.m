[pitch, lambda] = meshgrid(pitch_cmds, lambda_cmds);
surf(pitch, lambda, max(0,Cp));
xlabel("\theta (rad)");
ylabel("\lambda");
zlabel("c_p");