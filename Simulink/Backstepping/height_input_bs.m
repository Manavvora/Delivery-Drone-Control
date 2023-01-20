function U1 = height_input_bs(z_d,pos,lin_vel,orientation,droneparam)
kf = droneparam.Kf;
m = droneparam.m;
g = droneparam.g;
z_dot = lin_vel(3);
phi = orientation(1);
theta = orientation(2);

zeta1 = droneparam.zeta1;

c7 = droneparam.c7;
c8 = droneparam.c8;

e_7 = z_d(1) - pos(3);
e_8 = z_dot - z_d(2)- c7*e_7;

U1 = m* (g + e_7 + ((kf/m)*z_dot) - zeta1 + z_d(3) - c8*e_8 +c7*(z_dot - z_d(2))/ cos(phi)*cos(theta));

end