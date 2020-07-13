vx = 12:25;

G = {};

for v=vx
    [~,~,A,B,~] = eq_region3(tp, v);
    C = [1 0 0 0 0];
    D = [0 0];
    
    mdl = ss(A,B,C,D);
    G{end+1} = -1 * mdl(1);
end

K_p = 0.5;
K_i = 0.35;

s = tf('s');
C = K_p + K_i / s;

Gm = [];
Pm = [];
Wcp = [];
for k=1:numel(G)
    [gm,pm,~,wcp] = margin(C*G{k});
    gm = 20*log10(gm);
    Gm(end+1) = gm;
    Pm(end+1) = pm;
    Wcp(end+1) = wcp;
end

subplot(1,3,1);
plot(vx,Gm);
xlabel("Windspeed (m/s)");
ylabel("Gain margin (dB)");
subplot(1,3,2);
plot(vx,Pm);
xlabel("Windspeed (m/s)");
ylabel("Phase margin (degrees)");
subplot(1,3,3);
plot(vx,Wcp);
xlabel("Windspeed (m/s)");
ylabel("Crossover freq (rads)");