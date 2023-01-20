close all
clear all 
clc

global M g Ixx Iyy Izz l b d
M = 0.65;
g = 9.81;
l = 0.23;
b = 3.13*10^-5;
d = 7.5*10^-7;
Ixx = 0.015;
Iyy = 0.015;
Izz = 0.02;

I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

%% initial and desired parameters
euler = [15*pi/180 -10*pi/180, 0]';
eulerdot = [0.1 0.1 0.1]';
pos = [0 0 3]';
posdot = [0 0 0]';

psid = 2*pi/180;
xd = 5;
yd = 4;
zd = 8;

%% Simulation

a1 = (Iyy-Izz)/Ixx;
a2 = (Izz-Ixx)/Iyy;
a3 = (Ixx-Iyy)/Izz;
b1 = l/Ixx;
b2 = l/Iyy;
b3 = 1/Izz;

x = [euler(1);eulerdot(1);euler(2);eulerdot(2);euler(3);eulerdot(3);pos(3);posdot(3);pos(1);posdot(1);pos(2);posdot(2)];
xdes = [0;0;0;0;psid;0;zd;0;xd;0;yd;0];
xddes = [0;0;0;0;0;0;0;0;0;0;0;0];
xdddes = [0;0;0;0;0;0;0;0;0;0;0;0];

c1 = 2;
c2 = 2;
c3 = 2;
c4 = 2;
c5 = 2;
c6 = 2;
c7 = 2;
c8 = 2;
c9 = 2;
c10 = 2;
c11 = 2;
c12 = 2;

u1 = M*g;
u2 = 0;
u3 = 0;
u4 = 0;
ux = 0;
uy = 0;

dt = 0.01;
for k = 1:1000
    
    e1 = xdes(1)-x(1,k);
    e2 = x(2,k)-xddes(1)-c1*e1;
    e3 = xdes(3)-x(3,k);
    e4 = x(4,k)-xddes(3)-c3*e3;
    e5 = xdes(5)-x(5,k);
    e6 = x(6,k)-xddes(5)-c5*e5;
    e7 = xdes(7)-x(7,k);
    e8 = x(8,k)-xddes(7)-c7*e7;
    e9 = xdes(9)-x(9,k);
    e10 = x(10,k)-xddes(9)-c9*e9;
    e11 = xdes(11)-x(11,k);
    e12 = x(12,k)-xddes(11)-c11*e11;
    
    u2(k+1) = (e1 - a1*x(4,k)*x(6,k) + xdddes(1) - c2*e2 + c1*(xddes(1)-x(2,k)))./b1;
    u3(k+1) = (e3 - a2*x(2,k)*x(6,k) + xdddes(3) - c4*e4 + c3*(xddes(3)-x(4,k)))./b2;
    u4(k+1) = (e5 - a3*x(2,k)*x(4,k) + xdddes(5) - c6*e6 + c5*(xddes(5)-x(6,k)))./b3;
    u1(k+1) = M*(g + e7 + xdddes(7) - c8*e8 + c7*(xddes(7)-x(8,k)))./(cos(x(1,k))*cos(x(3,k)));
    
    ux(k+1) = M*(e9 + xdddes(9) - c10*e10 + c9*(xddes(9)-x(10,k)))./u1(k);
    uy(k+1) = M*(e11 + xdddes(11) - c12*e12 + c11*(xddes(11)-x(12,k)))./u1(k);
    
    xdot = [x(2,k); x(4,k)*x(6,k)*a1 + u2(k)*b1; x(4,k); x(2,k)*x(6,k)*a2 + u3(k)*b2; x(6,k); x(2,k)*x(4,k)*a3 + u4(k)*b3; x(8,k); u1(k)/M*cos(x(1,k))*cos(x(3,k)) - g; x(10,k); u1(k)*ux(k)/M; x(12,k); u1(k)*uy(k)/M];
    
    x(:,k+1) = x(:,k) + xdot*dt;
    
    euler(:,k+1) = [x(1,k+1);x(3,k+1);x(5,k+1)];
    eulerdot(:,k+1) = [x(2,k+1);x(4,k+1);x(6,k+1)];
    pos(:,k+1) = [x(9,k+1);x(11,k+1);x(7,k+1)];
    posdot(:,k+1) = [x(10,k+1);x(12,k+1);x(8,k+1)];
    
end

%% Plots

t = 0:0.01:10;
%numel(t)
figure(1)
plot(t,pos(1,:),t,pos(2,:),t,pos(3,:));
legend("x","y","z")
xlabel("time (s)")
ylabel("position (m)")

t = 0:0.01:10;
%numel(t)
figure(2)
plot(t,euler(1,:)*180/pi,t,euler(2,:)*180/pi,t,euler(3,:)*180/pi);
legend("phi","theta","psi")
xlabel("time (s)")
ylabel("euler angles (degrees)")

t = 0:0.01:10;
%numel(t)
figure(3)
plot(t,eulerdot(1,:)*180/pi,t,eulerdot(2,:)*180/pi,t,eulerdot(3,:)*180/pi);
legend("phidot","thetadot","psidot")
xlabel("time (s)")
ylabel("anglular velocities (degrees/s)")

t = linspace(0,10,1001);
%numel(t)
figure(4)
plot(t,u2,t,u3,t,u4);
legend("u2","u3","u4")
xlabel("time (s)")
ylabel("torque control input")

t = linspace(0,10,1001);
%numel(t)
figure(5)
plot(t,u1);
legend("u1")
xlabel("time (s)")
ylabel("thrust control input")