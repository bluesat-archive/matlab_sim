%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% WHEEL SUSPENSION STATIC ANALYSIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% CODE DEVELOPED BY: Chris Miller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% CODE DEVELOPED ON BEHALF OF BLUESAT UNSW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% GEOMETRIC RELATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

thetaSweep = [-45:0.5:45];
theta = thetaSweep; %Angle of lower arm, measured from horizontal
l_l_abs = 175; %(mm)
l_l = [cosd(theta);sind(theta);0.*theta].*l_l_abs;
l_l_t = 0.*l_l;

l_u = l_l;

for i = 1:length(theta)
    
           l_l_t([1,2],i) = [0,1;-1,0]*l_l([1,2],i);

end

S_l_n = 87.50.*normc(l_l); %(mm)
S_l_t = 15.*normc(l_l_t); %(mm)
S_l = S_l_n + S_l_t;

S_u_n = [0*theta;96.30*ones(1,length(theta));0*theta]; %(mm)
S_u_t = [15*ones(1,length(theta));0*theta;0*theta]; %(mm)
S_u = S_u_n + S_u_t;

h_i = [0*theta;112*ones(1,length(theta));0*theta];
h_o = h_i;

l_s = S_u - S_l;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PLOTTING TEST %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% This will only work when theta only takes a single value %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if length(theta) == 1
% 
%     plotVect = @(V1,V2) plot([V1(1);V1(1)+V2(1)],[V1(2);V1(2)+V2(2)]);
% 
%     figure(1);
%     hold on;
%     grid on; grid minor;
% 
%     plotVect([0;0;0],l_l)
%     plotVect([0;0;0],h_i)
%     plotVect(h_i,l_l)
%     plotVect(l_l,h_o)
%     plotVect([0;0;0],S_l_n)
%     plotVect(S_l_n,S_l_t)
%     plotVect([0;0;0],S_u_n)
%     plotVect(S_u_n,S_u_t)
%     plotVect(S_l,l_s)
% 
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% STATIC SOLVER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

m = 30;
F0 = [0;(1/4)*m*9.81;0];
k = 1800; %(N/m)
l_s0 = 140; %mm
x_s0 = 100; %mm
x_dash = 94; %mm
x_off = 26; %mm E [0mm,26mm]

%need to loop over the entries :/ Damnit. 

l_s_static = zeros(1,length(theta)); %Preallocate spring length array
l_s_norm = l_s_static;
x_s = l_s_static;
F_s = zeros(3,length(theta));
M_S = l_s_static; %Preallocate spring induced moment arrany
M_F = l_s_static; %Preallocate force induced moment arrany

%When these two forces are equal, the system is in its statically loaded
%conficuration.

for i = 1:length(theta)

    x_s(i) = norm(l_s(:,i)) - (l_s0 - x_dash) - x_off;
    
    F_s(:,i) = k*(x_s0 - x_s(i))*-(l_s(:,i)./norm(l_s(:,i)));
    
    M_S(i) = (10^-3)*norm(cross(S_l(:,i),F_s(:,i)));
    
    M_F(i) = norm(cross(l_l(:,i),F0));
    
end

Intercepted = 0;
Intercept = 0;

for i = 1:length(theta-1)
    if M_S(i)<M_F(i) && M_S(i+1)>M_F(i+1)
        fprintf('Moments are equal for theta = [%1d,%1d]\n\n',theta(i),theta(i+1))
        Intercept = i;
        Intercepted = 1;
    elseif i == length(theta-1) && Intercepted == 0
        fprintf('No intercept was found, please review plots');
    end
end

M_S_Intercept = linspace(M_S(Intercept),M_S(Intercept+1),1000);
M_F_Intercept = linspace(M_F(Intercept),M_F(Intercept+1),1000);
theta_Intercept = linspace(theta(Intercept),theta(Intercept+1),1000);

Intercepted = 0;
Intercept = 0;

for i = 1:length(theta_Intercept-1)

    if M_S_Intercept(i)<M_F_Intercept(i) && M_S_Intercept(i+1)>M_F_Intercept(i+1)
        fprintf('(Refined) Moments are equal for theta = [%1d,%1d]\n\n',theta_Intercept(i),theta_Intercept(i+1))
        Intercept = i;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% RESOLVE SYSTEM FOR SOLUTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta = theta_Intercept(Intercept); %Angle of lower arm, measured from horizontal
l_l_abs = 175; %(mm)
l_l = [cosd(theta);sind(theta);0.*theta].*l_l_abs;
l_l_t = 0.*l_l;

l_u = l_l;

for i = 1:length(theta)
    
           l_l_t([1,2],i) = [0,1;-1,0]*l_l([1,2],i);

end

S_l_n = 87.50.*normc(l_l); %(mm)
S_l_t = 15.*normc(l_l_t); %(mm)
S_l = S_l_n + S_l_t;

S_u_n = [0*theta;96.30*ones(1,length(theta));0*theta]; %(mm)
S_u_t = [15*ones(1,length(theta));0*theta;0*theta]; %(mm)
S_u = S_u_n + S_u_t;

h_i = [0*theta;112*ones(1,length(theta));0*theta];
h_o = h_i;

l_s = S_u - S_l;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PLOT SOLUTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(1)
subplot(1,2,1)
plot(thetaSweep,M_S,thetaSweep,M_F,[theta,theta],[0,max([M_S,M_F])]);
grid on; grid minor;
legend('M_S','M_F');


subplot(1,2,2)

AngleEqual = sprintf('Equalibrum Angle, theta = %1.2f degrees',theta);
title(AngleEqual);

if length(theta) == 1

    plotVect = @(V1,V2) plot([V1(1);V1(1)+V2(1)],[V1(2);V1(2)+V2(2)]);

    figure(1);
    hold on;
    grid on; grid minor;

    plotVect([0;0;0],l_l)
    plotVect([0;0;0],h_i)
    plotVect(h_i,l_l)
    plotVect(l_l,h_o)
    plotVect([0;0;0],S_l_n)
    plotVect(S_l_n,S_l_t)
    plotVect([0;0;0],S_u_n)
    plotVect(S_u_n,S_u_t)
    plotVect(S_l,l_s)

end

axis equal;



