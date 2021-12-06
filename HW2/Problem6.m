
%% Non-Holonomic Vehicle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Rohan Nair 
% AMR 2021 
% Date: 10/12/2021
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 100])
ylim([0 100])

%Velocity
max_vel = 5;

%Steering angle
steering = pi/4; 

%Initialize a vector of positions for the robot
x=[]; 
y=[];

theta = []; 

theta(1) = 0;

vels = [];

%% Robot Initial Pose

% Initial Orientation 
x(1) = 10;
y(1) = 80;

% Initial Orientation 


% Build Robot Model
robot = QuadRobot(x,y,theta(1));
hold on
plot(robot(:,1),robot(:,2),'-');
hold off
xlim([0 100])
ylim([0 100])

dt = 0.1;
    

% start and goal coordinates

q_goal = [80, 20];

p1 = [30, 50];
p2 = [50, 50];
p3 = [50, 70];
p4 = [30, 70];

c = [(p1(1) + p2(1))/2, (p2(2) + p3(2))/2]; % calculating center of obstacle

d0 = 30;
xi = 1.5;
eta = 1000;

i = 1;
thresh = 2;

K_prop = 10;

% check distance from goal
while (sqrt((x(i) - q_goal(1))^2 + (y(i) - q_goal(2))^2) > thresh)
    % updating current x and y
    dist = sqrt((x(i) - c(1))^2 + (y(i) - c(2))^2);
    dist = dist/5 + 0.1;
    
    % F = negative gradient of U 
    gx = -xi*(x(i) - q_goal(1)) + eta*(1/dist - 1/d0)*(1/dist^2)*(x(i) - c(1))/dist;
    gy = -xi*(y(i) - q_goal(2)) + eta*(1/dist - 1/d0)*(1/dist^2)*(y(i) - c(2))/dist;
    
    goal_vel = sqrt(gx^2 + gy^2); % gradient norm 
    
    goal_vel = min(abs(goal_vel), max_vel)*sign(goal_vel); 
    
    theta_d = atan2(gy, gx);
    
    gamma = K_prop*atan2(sin(theta_d - theta(i)), cos(theta_d - theta(i)));
   
    steering_change = min(steering, gamma);
    
    x(i+1) = x(i) + goal_vel*cos(theta(i))*dt;
    y(i+1) = y(i) + goal_vel*sin(theta(i))*dt;
    theta(i+1) = theta(i) + steering_change*dt;
    robot = QuadRobot(x(i),y(i),theta(i));
    
    i = i + 1;
   
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    hold on
    plot(q_goal(1), q_goal(2), 'r*');
    hold off
    xlim([0 100])
    ylim([0 100])
    pause(0.001)
    
    hold on
    plot(x, y, 'r-')
    hold off
    xlim([0 100])
    ylim([0 100])
    drawnow
end
title("Non-Holonomic Robot Trajectory")







