
clear             

load('u_inf_hat')
u_hat = U_inf_hat.Data
u_hat = u_hat(100:end); %input data
delta = 0.00635;        %sampling time
n=5;                    %order of the regression
sigma = 1000;           %inital weighting P
lambda = 1;             %forgeting factor
T = 5                   %Time constant for low pass

%%

plot(u_hat)
ylim([7 12])

hold on
filter_out = zeros(size(u_hat));
filter_out(1) = u_hat(1);
for i=2:size(u_hat)
    filter_out(i) = low_pass(u_hat(i),filter_out(i-1),delta,T);  
end
plot(filter_out)
hold off


%% Initialize regression nth order hold 
m = 2000;        %number of points considered
num_for = 2000;   %number of points forcast

Psi = zeros(m,n);
for i=1:m
    for j=1:n
        Psi(i,j) = ((m-i)*delta)^(n-j);
    end
end

Psi_inv = (Psi'*Psi)\Psi'; %Psudoinverse of Psi

%% Regression
out = zeros(size(filter_out,1),1);

for i= 1:size(filter_out,1)-m
    y = filter_out(i:m-1+i);
    y = flip(y);
    pts = get_hold_points(Psi_inv,y,n,delta,num_for);
    out(i,1) = pts(num_for,1); 
end

plot(out)

%% plot

plot(out)
hold on 
plot(filter_out)
plot(u_hat)
hold off
ylim([7.5 12])
legend('future projection','filtered','u_inf');




%returns the next m points from the current timestep
%usage: Psi_inv Psudo-Inverse,
%       y last data points (note: rows = cols of Psi_inv), 
%       n order of regression,
%       delta saple rate,
%       m number of points forcast,
function pts = get_hold_points(Psi_inv,y,n,delta,num_for)

    Theta = Psi_inv*y;
    pts = zeros(num_for,1);
    for i=1:num_for
       for j = 1:n 
        pts(i,1) = pts(i,1)+Theta(j)*((i+n-1)*delta)^(n-j);
       end
    end
end

%returns low pass signal
%usage: x current input,
%       x_last last output,
%       x_hat next output,
%       delta sample time,
%       time constant in radians,
function x_hat = low_pass(x,x_last,delta,T)
    if(isnan(x))
       x_hat=x_last; 
    else
        x_hat = x*(delta)/(T+delta)+x_last*(T/(T+delta));
    end
end

% plot(out(1:end-1))



