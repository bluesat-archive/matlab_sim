%%%%%%%%%
%Damping constants calculator
%x_an and x_nu are giving the same results

close all;

m = 1;      %mass (kg)
g = 9.81;   %gravity (ms^-2)
c = 0.01;   %damping coefficient

tSpan = [0:0.0005:10];

x_an = @(t) (m*g/c)*t+(g*(m^2)/(c^2))*exp((-c/m)*t)-(g*(m^2)/(c^2))

%x'' = g -(c/m)x'

x_dot = @(t,x) [x(2);g-(c/m)*x(2)];

IC = [0;0];

[t_nu,x_nu] = ode45(x_dot,tSpan,IC);

x_an(tSpan)
hold on;
plot(tSpan, x_nu(:,1))
plot(tSpan,x_an(tSpan))

grid on;
legend('x_an', 'x_nu');