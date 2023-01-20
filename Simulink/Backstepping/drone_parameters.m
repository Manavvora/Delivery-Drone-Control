clc;
clear;

% Drone parameters
droneparam.m = 1;     %mass
droneparam.g = 9.8;   %gravity
droneparam.Kf=  0;      %translation drag coefficient
droneparam.Ix=  0.0119;    %moment of inertia
droneparam.Iy=  0.0119;
droneparam.Iz = 0.0223;
droneparam.Kax =   0.1;   %aerodynamic friction coefficients
droneparam.Kay = 0.1;
droneparam.Kaz = 0.1;
droneparam.Jr =    8e-4;   % rotor inertia
droneparam.l =   0.23;     %distance between the quadrotor center of mass and the rotation axis of propeller
droneparam.Kd =  0.1;     %drag coefficient.
droneparam.b= 7e-6;         %thrust factor
droneparam.d= 1.2e-7;       %drag factor

%initial conditions 
droneparam.q0 = [0 0 0];
droneparam.X0 = [0 0 0];

% Simulation time 
T = 50;
Ts = 0.002; % Fixed Time step 

%Desired position
x_d = 20;   y_d = 65;   z_d = 50;   si_d = 1*pi/180;

% PID gain for Trajectory generation
kp_d = 15;   kd_d = 8;   ki_d = 0.1;

% droneparam.zeta1 = -1 + 2*rand(1);
% droneparam.zeta2 = -1 + 2*rand(1);
% droneparam.zeta3 = -1 + 2*rand(1);
% droneparam.zeta4 = -1 + 2*rand(1);

droneparam.zeta1 = 0;
droneparam.zeta2 = 0;
droneparam.zeta3 = 0;
droneparam.zeta4 = 0;


% Backstepping
droneparam.c1 = 6; droneparam.c2 = 4;
droneparam.c3 = 5.2; droneparam.c4 = 3;
droneparam.c5 = 7; droneparam.c6 = 4.9; 
droneparam.c7 = 6.125; droneparam.c8 = 31.5;