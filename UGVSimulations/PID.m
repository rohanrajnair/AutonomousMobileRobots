%% PID Implementation

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

t = [];

%% Robot Initial Pose

% Initial Orientation 
x(1) = 100;
y(1) = 100;

% Initial Orientation 
theta(1) = 2*pi*rand(1,1);

vels(1) = 0;

% Build Robot Model
robot = QuadRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])
    
%% Move Robot

%time step
dt = 0.1;


% scalars 
K_prop = 1;
K_p = 0.5;
K_i = 0.05; 
K_d = 0.02; 

i = 1;

threshold = 1;

% reference velocity
v_ref = 3; 

prev_error = 0;
integral = 0; 
u = 0;

k = 0; % number of goals 

t(1) = 0;

x_gs = [];
y_gs = [];

while (k < 3)
    % generate random goal coordinates 
    x_g = abs(200*rand(1,1));
    y_g =  abs(200*rand(1,1));
    while (abs(x_g - x(i)) > threshold)  && (abs(y_g - y(i)) > threshold)

        theta_d = atan2(y_g - y(i), x_g - x(i));
        
        % calcualting steering angle
        gamma = K_prop*atan2(sin(theta_d - theta(i)), cos(theta_d - theta(i)));
        
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
        t(i+1) = i; 
        i = i + 1;
        plot(robot(:,1),robot(:,2),'-',x,y,'-');
        hold on
        plot(x_g, y_g, 'r*');
        x_gs(k+1) = x_g;
        y_gs(k+1) = y_g; 
        hold off
        xlim([0 200])
        ylim([0 200])
        pause(0.01)
    end
    title('PID Trajectory');
    k = k + 1;
end 

pause(0.1);

hold on
robot = QuadRobot(x(1),y(1),theta(1));
plot(robot(:,1),robot(:,2),'-');
plot(x_gs(1), y_gs(1), 'b*');
plot(x_gs(2), y_gs(2), 'b*');
plot(x_gs(3), y_gs(3), 'b*');


figure();
plot(t, vels);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('PID Velocity'); 
