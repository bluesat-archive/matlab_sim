close all;

m = 1;
g = 9.81;
c = 0.01;

tSpan = [0:0.0005:10];

x_an = @(t) (m*g/c)*t+(g*(m^2)/(c^2))*exp((-c/m)*t)-(g*(m^2)/(c^2));

%x'' = g -(c/m)x'

x_dot = @(t,x) [x(2);g-(c/m)*x(2)];

IC = [0;0];

[t_nu,x_nu] = ode45(x_dot,tSpan,IC);

hold on;
plot(tSpan,x_an(tSpan))
plot(tSpan,x_nu(:,1))
