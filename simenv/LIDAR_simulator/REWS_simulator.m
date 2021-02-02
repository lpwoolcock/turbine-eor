function [] = REWS_simulator(turbsim_input, csv_output, R)


    [velocity, twrVelocity, y, z, zTwr, nz, ny, dz, dy, dt, zHub, z1,mffws] = readTSgrid(turbsim_input);
    [Vh, delta, Vz] = REWS(velocity, y, z, dt, zHub, mffws, 130, R);
    t = dt * (0:(size(Vh, 1)-1)).';
    
    HLinShr = zeros(length(t), 1);
    VShr = repmat(0.14, [length(t) 1]); % assuming IECKAI/IECVKM with NTM
    VLinShr = zeros(length(t), 1);
    Vgust = zeros(length(t), 1);
    
    data = [t Vh delta Vz HLinShr VShr VLinShr Vgust];
    
    writematrix(data, csv_output);
end
