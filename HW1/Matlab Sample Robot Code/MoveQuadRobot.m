%% Move Quad Robot

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

%Velocity (constant for this demo example) 
vel = 5;

%Steering angle
steering = pi/4; 

%Initialize a vector of positions for the robot
x=[]; 
y=[];

%% Robot Initial Pose

x(1) = 100;
y(1) = 100;

% Initial Orientation 
theta(1) = 2*pi*rand(1,1);

% Build Robot Model
robot = QuadRobot(x,y,theta);

hold on
plot(robot(:,1),robot(:,2),'-');
plot(x(1), y(1));
text(x(1), y(1), ['(' num2str(x(1)) ',' num2str(y(1)) ')']);
xlim([0 200])
ylim([0 200])


x(2) = x(1) + 20;
y(2) = y(1) + 30; 

theta(2) = theta(1) + pi/6;
robot = QuadRobot(x(2),y(2),theta(2));
plot(robot(:,1),robot(:,2),'-');
plot(x(2), y(2));
text(x(2), y(2), ['(' num2str(x(2)) ',' num2str(y(2)) ')']);
xlim([0 200])
ylim([0 200])
title('Homogeneous Transformation');
legend('initial', 'final');