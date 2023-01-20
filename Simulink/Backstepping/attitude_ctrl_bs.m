function Y = attitude_ctrl_bs(phi_d,theta_d,psi_d,ang_vel,orientation,droneparam)
Ix = droneparam.Ix;
Iy = droneparam.Iy;
Iz = droneparam.Iz;
l = droneparam.l;

zeta2 = droneparam.zeta2;
zeta3 = droneparam.zeta3;
zeta4 = droneparam.zeta4;

phi = orientation(1);
theta = orientation(2);
si = orientation(3);

phi_dot = ang_vel(1);
theta_dot = ang_vel(2);
si_dot = ang_vel(3);

c1 = droneparam.c1;
c2 = droneparam.c2;
c3 = droneparam.c3;
c4 = droneparam.c4;
c5 = droneparam.c5;
c6 = droneparam.c6;

e_1 = phi_d(1) - phi;
e_2 = phi_dot - phi_d(2) - c1*e_1;
U2 = Ix *(e_1 + (theta_dot * si_dot * (Iz - Iy)/Ix) - (zeta2/Ix) + phi_d(3) - c2*e_2 + c1*(phi_d(2)- phi_dot))/l; 

e_3 = theta_d(1)  - theta;
e_4 = theta_dot -theta_d(2) - c3*e_3;
U3 = Iy*(e_3 + (phi_dot * si_dot * (Ix - Iz)/Iy)- (zeta3/Iy) + theta_d(3) -c4*e_4 +c3*(theta_d(2)-theta_dot))/l;

e_5 = psi_d(1)-si;
e_6 = si_dot - psi_d(2) - c5*e_5;
U4 = Iz*(e_5 + (theta_dot * phi_dot * (Iy - Ix)/Iz) - (zeta4/Iz)+ psi_d(3) -c6*e_6 +c5*(psi_d(2)-si_dot))/l;

 Y = [U2;U3;U4] ; 
