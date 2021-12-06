%% Move To Pose

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
x(1) = 10;
y(1) = 10;

% Initial Orientation 
theta(1) = 0;

vels(1) = 0;


% Build Robot Model
robot = QuadRobot(x,y,theta(1));
hold on
plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])
hold off
    
%% Move Robot

%time step
dt = 0.1;

% goal x, y, theta 
x_g = 130;
y_g = 70;
theta_g = pi/4;

K_v = 0.1; 

i = 1;

K_prop = 1;

% acceptable tolerance for x/y position and theta
threshold = 1;

theta_threshold = 0.1;

t = [];

% scalars
K_rho = 0.3;
K_beta = -0.02;
K_alpha = 0.09;

 
while (abs(x_g - x(i)) > threshold  || abs(y_g - y(i)) > threshold || abs(theta_g - theta(i)) > theta_threshold)
    
    rho = sqrt((x_g - x(i))^2 + (y_g -y(i))^2);
    
    alpha = atan2(y_g - y(i), x_g - x(i)) - theta(i);
        
    beta = - atan2(sin(theta_g - alpha), cos(theta_g - alpha)); 
        
    v = K_rho * rho;
    
    gamma = K_alpha * alpha + K_beta * beta; 
    
    vels(i+1) = min(max_vel, v);
    steering_change = min(steering, gamma);
    x(i+1) = x(i) + vels(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vels(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + steering_change * dt; 
    robot = QuadRobot(x(i),y(i),theta(i));
    i = i + 1;
    t(i) = i;
    
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    hold on
    plot(x_g, y_g, 'r*');
    hold off
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
end

title('Move to Pose Trajectory');
hold on
robot = QuadRobot(x(1),y(1),theta(1));
plot(robot(:,1),robot(:,2),'-');
