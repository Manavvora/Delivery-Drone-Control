function Y = attitude_ctrl(phi_d,theta_d,si_d,ang_vel,orientation,droneparam)
Ix = droneparam.Ix;
Iy = droneparam.Iy;
Iz = droneparam.Iz;
l = droneparam.l;

lembda = droneparam.lembda;
zeta2 = droneparam.zeta2;
zeta3 = droneparam.zeta3;
zeta4 = droneparam.zeta4;

c1 = droneparam.c1;
c3 = droneparam.c3;
c5 = droneparam.c5;

phi_dot = ang_vel(1);
theta_dot = ang_vel(2);
si_dot = ang_vel(3);

U2_nom = Ix * ((theta_dot * si_dot * (Iz - Iy)/Ix) + c1*lembda * (phi_d(2) - phi_dot) + phi_d(3)) / l;
U3_nom = Iy * ((phi_dot * si_dot * (Ix - Iz)/Iy) + c3*lembda * (theta_d(2) - theta_dot) + theta_d(3)) / l;
U4_nom = Iz * ((theta_dot * phi_dot * (Iy - Ix)/Iz) + c5*lembda * (si_d(2) - si_dot) + si_d(3));

zeta_hat = 0.1;
F2 = zeta2 - zeta_hat;
F3 = zeta3 - zeta_hat;
F4 = zeta4 - zeta_hat;

eta = 12;

k2 = F2 + eta;
k3 = F3 + eta;
k4 = F4 + eta;
k = [k2 0 0;0 k3 0;0 0 k4];

P_hat = -orientation + [phi_d(1);theta_d(1);si_d(1)];
dP_hat = -ang_vel + [phi_d(2);theta_d(2);si_d(2)];
s = P_hat  + lembda * dP_hat;

U_dis = k*[sat(s(1));sat(s(2));sat(s(3))] + k*s;

 Y = [U2_nom;U3_nom;U4_nom] + U_dis; 
 
end

function Y=sat(s)
% % sat is the saturation function with unit limits and unit slope.
%     if s>1
%         Y=1;
%     elseif s<-1 
%         Y=-1;
%     else 
%         Y=s;
%     end
    Y = s./(abs(s)+0.001);
end