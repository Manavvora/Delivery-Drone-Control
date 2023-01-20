clc
clear all;
close all;

%% initialisation
m = 4.493;
Ixx = 0.177;
Iyy = 0.177;
Izz = 0.334;
J = [Ixx 0 0;0 Iyy 0;0 0 Izz];
g = 9.81;
Kpz = 50;
Kdz = 10;
Kpxy = 20;
Kdxy = 10;
Kpeu = 40;
Kdeu = 10;

euler = [0 0 0]';
Xpd = [0 0 0]';
Xpd_dot = [0 0 0]';
Xpd_ddot = (1/m)*[0 0 0]';
Xad = [0 0 0]';
Xad_dot = [0 0 0]';
Xad_ddot = [0 0 0]';
xd1 = linspace(1,100,100);
yd1 = xd1;
xd2 = linspace(-1,1,100);
yd2 = sqrt(1 - xd2^2);
zd = 10;
XD1 = [xd1 yd1 zd]';
XD2 = [xd2 yd2 zd]';
epd1 = XD1 - Xpd;
epd1_dot = [1 1 0]' - Xpd_dot;
epd1_ddot = -Xpd_ddot;
ead1 = XD1 - Xad;
ead1_dot = [1 1 0]' - Xad_dot;
epd1_ddot = -Xad_ddot;
epd2 = XD2 - Xpd;
%epd2_dot = 
ead2 = XD2 - Xad;
theta_d1 = Kpxy*epd1(1)+Kdxy*epd1_dot(1);
%theta_d1 = zeros(1,1000);
phi_d1 = Kpxy*epd1(2)+Kdxy*epd1_dot(2);
%phi_d1 = zeros(1,1000);
theta_tilda_1 = theta_d1 - euler(2);
phi_tilda_1 = phi_d1 - euler(1);
theta_tilda_1_dot = Kpxy*epd1_dot(1) + Kdx*epd1_ddot(1);
phi_tilda_1_dot = Kpxy*epd1_dot(2) + Kdx*epd1_ddot(2);
tau = [Kpeu*theta_tilda_1+Kdeu*theta_tilda_1_dot; Kpeu*phi_tilda_1+Kdeu*phi_tilda_1_dot;0];
R = [cos(euler(2))*cos(euler(3)), sin(euler(1))*sin(euler(2))*cos(euler(3))-cos(euler(1))*sin(euler(3)), sin(euler(2))*cos(euler(1))*cos(euler(3))+sin(euler(3))*sin(euler(1)); cos(euler(2))*sin(euler(3)), sin(euler(1))*sin(euler(2))*sin(euler(3))-cos(euler(1))*cos(euler(3)), sin(euler(2))*cos(euler(1))*sin(euler(3))-cps(euler(3))*sin(euler(1)); -sin(euler(2)), cos(euler(2))*sin(euler(1)), cos(euler(2))*cos(euler(1))];
W = [0 0 0]';
W_hat = [0, -W(3), W(2); W(3), 0, -W(1); -W(2), W(1), 0];
W_dot = inv(J)*(cross(-W,J*W) + tau);
t = linspace(1,100,1000);
dt = 0.01;

%% disturbance
d1 = [1 0 0]';
d2 = [0 1 0]';
d3 = [1/sqrt(2) 1/sqrt(2) 0]';
d4 = [sin(2*pi/(40*t)) 0 0]';

%% Adaptive Parameters
gamma = 10;
Ax1 = -gamma*ead1(1);
Ax2 = -gamma*ead2(1);
Ay1 = -gamma*ead1(2);
Ay2 = -gamma*ead2(2);

%% Simulation
for k = 1:1000
    theta_d1(k) = Kpxy*epd1(1,k)+Kdxy*epd1_dot(1,k);
    phi_d1(k) = Kpxy*epd1(2,k)+Kdxy*epd1_dot(2,k);
    
