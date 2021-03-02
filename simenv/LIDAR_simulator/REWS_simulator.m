function [] = REWS_simulator(turbsim_input, csv_output, R)


    [velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(turbsim_input);
    [Vh, HLinShr, VLinShr] = REWS(velocity, y, z, dt, zHub, mffws, 130, R);
    
    t = dt * (0:(size(Vh, 1)-1)).';
    delta = zeros(length(t), 1);
    VShr = zeros(length(t), 1);
    Vgust = zeros(length(t), 1);
    Vz = zeros(length(t), 1);
    
    data = [t Vh delta Vz HLinShr VShr VLinShr Vgust];
    
    writematrix(data, csv_output);
end
