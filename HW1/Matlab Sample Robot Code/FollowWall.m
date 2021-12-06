%% Following Wall

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Rohan Nair
% AMR 2021 
% Date: 09/22/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 200])
ylim([0 200])

%Velocity
max_vel = 5;

%Steering angle
steering = pi/4; 

%Initialize a vector of positions for the robot
x=[]; 
y=[];

theta = []; 

vels = [];

%% Robot Initial Pose

% Initial Orientation 
x(1) = 120;
y(1) = 120;

% Initial Orientation 
theta(1) = 2*pi*rand(1,1);

vels(1) = 0;

% Build Robot Model
robot = QuadRobot(x,y,theta(1));

% ax + by + c = 0
a = 1;
b = 1;
c = -160;

x_line = 0:0.1:1000;
y_line = (-a*x_line - c)/b;
plot(x_line,y_line)


plot(robot(:,1),robot(:,2),'-');
plot(x_line,y_line)
xlim([0 200])
ylim([0 200])
    
%% Move Robot

%time step
dt = 0.1;

K_p = 0.3;
K_i = 0.05; 
K_d = 0.01; 
K_t = 0.1;
K_h = 3; 

i = 1;

% acceptable tolerance for x/y position
threshold = 1;

v_ref = 3; 

prev_error = 0;
integral = 0; 
u = 0;

k = 0; % # of goals reached

nstep = 400; 

rel_dist = [];
t = [];

while (i < nstep)
    % calculating distance
    d = (a*x(i) + b*y(i) + c)/(sqrt(a^2 + b^2)) - 10;
    
    t(i) = i;
    % using to plot relative distance
    rel_dist(i) = (a*x(i) + b*y(i) + c)/(sqrt(a^2 + b^2));

    alpha_t = -K_t * d;

    theta_d = atan2(-a, b);
    
    alpha_h = K_h*atan2(sin(theta_d - theta(i)), cos(theta_d - theta(i)));
    gamma = alpha_t + alpha_h;
    % PID
    error = v_ref - vels(i); 
    integral = integral + error*dt; 
    derivative = (error - prev_error)/dt; 
    u = K_p * error + K_i * integral + K_d * derivative; 
    prev_error = error; 

    vels(i+1) = vels(i) + u - 0.01*vels(i);

    steering_change = min(steering, gamma);

    x(i+1) = x(i) + vels(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vels(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + steering_change * dt; 
    robot = QuadRobot(x(i),y(i),theta(i));
    i = i + 1;

    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    hold on
    plot(x_line,y_line);
    hold off
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
end
title('Follow Wall Trajectory');
hold on
robot = QuadRobot(x(1),y(1),theta(1));
plot(robot(:,1),robot(:,2),'-');

figure()
plot(t, rel_dist)
title('Distance to Line (d=10)');
xlabel('Time (s)');
ylabel('Distance (m)');

