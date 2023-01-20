function U1 = height_input(z_d,pos,lin_vel,orientation,droneparam)
kf = droneparam.Kf;
m = droneparam.m;
g = droneparam.g;
z_dot = lin_vel(3);
phi = orientation(1);
theta = orientation(2);
lembda = droneparam.lembda;
zeta1 = droneparam.zeta1;

c7 = droneparam.c7;

U1_nom = m * ( z_d(3) + c7*lembda * z_d(2) + c7*((kf/m) - lembda) * z_dot + g ) / cos(phi)*cos(theta);

kf_hat = 1;
zeta_hat = 0.1;
F1 = (kf - kf_hat) * z_dot + zeta1 - zeta_hat;
eta = 12;

k1 = F1 + eta;

P_hat = -pos(3) + z_d(1);
dP_hat = -z_dot + z_d(2);
s = P_hat  + lembda * dP_hat;

U1_dis = k1*sat(s) + k1*s;

U1 = U1_nom + U1_dis;

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