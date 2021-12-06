function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

Kp = 29;
Kv = 6.7;

m = params.mass;
g = params.gravity;
u_max = params.u_max;

pos = s(1);
vel = s(2);

pos_des = s_des(1);
vel_des = s_des(2);

e = pos_des - pos;
vel_e = vel_des - vel;

u = min(m * (Kp*e + Kv*vel_e + g), u_max);
end

