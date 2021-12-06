%% Go To Goal

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
x(1) = 100;
y(1) = 100;

% Initial Orientation 
theta(1) = 2*pi*randn(1,1);

vels(1) = 0;


% Build Robot Model
robot = QuadRobot(x,y,theta(1));
hold on
plot(robot(:,1),robot(:,2),'-');
hold off
xlim([0 200])
ylim([0 200])
    
%% Move Robot

%time step
dt = 0.1;


x_g = abs(200*rand(1,1));
y_g =  abs(200*rand(1,1));

K_v = 0.1; 

i = 1;

K_prop = 1;

% acceptable tolerance for x/y position
threshold = 1;

t = [];
 
while (abs(x_g - x(i)) > threshold)  && (abs(y_g - y(i)) > threshold)
    
    theta_d = atan2(y_g - y(i), x_g - x(i));
    gamma = K_prop*atan2(sin(theta_d - theta(i)), cos(theta_d - theta(i)));
    goal_vel = K_v * sqrt((y_g - y(i))^2 + (x_g - x(i))^2);
    
    vels(i+1) = min(max_vel, goal_vel);
    steering_change = min(steering, gamma);
    
    x(i+1) = x(i) + vels(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vels(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + steering_change * dt; 
    robot = QuadRobot(x(i),y(i),theta(i));
    
    i = i + 1;
    
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    hold on
    plot(x_g, y_g, 'r*');
    hold off
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
end

title('Go-to-Goal Trajectory');
hold on
robot = QuadRobot(x(1),y(1),theta(1));
plot(robot(:,1),robot(:,2),'-');

% figure()
% plot(t, vels);
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Go-to-Goal Velocity');
