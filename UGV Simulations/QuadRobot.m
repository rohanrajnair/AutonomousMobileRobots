%% Homogeneous Transformation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Rohan Nair 
% AMR 2021 
% Date: 09/22/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [robot] = QuadRobot(x,y,theta)

center = [x y];

% Robot parallelogram shape

a = [0 -3];
b = [12 -1];
c = [12 1];
d = [0 3];

% Rotation Matrix

rotmat = [cos(theta) -sin(theta); sin(theta) cos(theta)];

rota = (rotmat * (a'));
rotb = (rotmat * (b'));
rotc = (rotmat * (c'));
rotd = (rotmat * (d'));

% Final Robot Configuration after transformation

robot1 = [rota(1) + center(1), rota(2) + center(2)];
robot2 = [rotb(1) + center(1), rotb(2) + center(2)];
robot3 = [rotc(1) + center(1), rotc(2) + center(2)];
robot4 = [rotd(1) + center(1), rotd(2) + center(2)];

robot = [robot1;robot2;robot3;robot4;robot1];
 
end


