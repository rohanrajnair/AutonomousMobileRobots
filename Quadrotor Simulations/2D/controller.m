function [ u1, u2 ] = controller(~, state, des_state, params)

kp_z = 50;
kd_z = 25;

kp_y = 10;
kd_y = 50;

kp_phi = 100;
kd_phi = 25;

y = state.pos(1);
z = state.pos(2);
y_dot = state.vel(1);
z_dot = state.vel(2);
phi = state.rot(1);
phi_dot = state.omega(1);

des_y = des_state.pos(1);
des_z = des_state.pos(2);
des_y_dot = des_state.vel(1);
des_z_dot = des_state.vel(2);
des_y_ddot = des_state.acc(1);
des_z_ddot = des_state.acc(2);

m = params.mass;
g = params.gravity; 

u1 = m * (g + des_z_ddot + kd_z * (des_z_dot - z_dot) + kp_z * (des_z - z));
phi_c = -1/g * (des_y_ddot + kd_y *(des_y_dot - y_dot) + kp_y * (des_y - y));
u2 = kp_phi * (phi_c - phi) + kd_phi * (0 - phi_dot);

end

