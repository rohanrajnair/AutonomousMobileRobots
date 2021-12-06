%% Kalman Filter Implementation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Rohan Nair
% AMR 2021 
% Date: 11/09/2021
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

vels = [];

t = [];

%time step
dt = 0.1;

%% Robot Initial Pose

% Initial Orientation 
x(1) = 20;
y(1) = 20;

vels(1) = 0;

% s1 = N(0,6), s2 = N(0,4), input noise v = N(0,1)

% state = [x; y; x_vel; y_vel]
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % state transition matrix

H = [1 0 0 0; 0 1 0 0; 1 0 0 0; 0 1 0 0]; % measurement matrix 

P = diag([0 0 0 0]); % uncertainty covariance

Q = diag([0 0 1 1]); % process noise

R = diag([6 4 6 4]); % measurement noise

xlim([0 100])
ylim([0 100])
    
%% Move Robot

% scalars 
K_prop = 1;
K_p = 0.5;
K_i = 0.05; 
K_d = 0.01; 

i = 1;

threshold = 1;

% reference velocity
v_ref = 3; 

prev_error = 0;
integral = 0; 
u = 0;

t(1) = 0;

vel_est = [];
vel_est(1) = 0;

% generate random goal coordinates 
x_g = 55;
y_g =  65;

theta(1) = atan2(y_g - y(1), x_g - x(1));

est_dist_to_goal = [];

est_dist_to_goal(1) = 50;

while (abs(x_g - x(i)) > threshold)  || (abs(y_g - y(i)) > threshold)

    
    theta_d = atan2(y_g - y(i), x_g - x(i));

    % calculating steering angle
    gamma = K_prop*atan2(sin(theta_d - theta(i)), cos(theta_d - theta(i)));

    % PID
    error = v_ref - vels(i); 
    integral = integral + error*dt; 
    derivative = (error - prev_error)/dt; 
    u = K_p * error + K_i * integral + K_d * derivative; 
    prev_error = error;
    
    % Kalman Filtering
    
    % state prediction
    prev_state = [x(i); y(i); vels(i)*cos(theta(i)); vels(i)*sin(theta(i))];
    curr_state = F * prev_state + [0; 0; randn; randn];
    s1_error = [6*randn; 6*randn];
    s2_error = [4*randn; 4*randn];
    s1_reading = [curr_state(1); curr_state(2)] + s1_error;
    s2_reading = [curr_state(1); curr_state(2)] + s2_error;
    z = H * curr_state + R*[randn; randn; 0; 0];
    error = z - H * curr_state;
    
    P = F * P * transpose(F) + Q;
    S = H * P * transpose(H) + R; % innovation
    K = P * transpose(H) * inv(S); % Kalman gain 
    
    % update state
    est_state = curr_state + K*error;
    P = (eye(4) - K * H) * P;
    
    vels(i+1) = vels(i) + u - 0.01*vels(i);
    
    vel_est(i+1) = sqrt(curr_state(3)^2 + curr_state(4)^2);
    
    steering_change = min(steering, gamma);

    x(i+1) = x(i) + vels(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vels(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + steering_change * dt;
    
    est_dist_to_goal(i) = sqrt((y_g - est_state(2))^2 + (x_g - est_state(1))^2);
    
    hold on
    plot(s1_reading(1), s1_reading(2), 'c.');
    plot(s2_reading(1), s2_reading(2), 'm.');
    plot(est_state(1), est_state(2), 'k.');
    plot(x(i), y(i), 'r.')
    plot(x_g, y_g, 'b*');
    hold off
    xlim([0 100])
    ylim([0 100])
    drawnow
    
    t(i+1) = i; 
    i = i + 1;
    
end
title('Kalman Filter Estimation for Go-to-Goal (Constant Velocity)');
text(x_g, y_g, ['(' num2str(x_g) ', ' num2str(y_g) ')'], 'VerticalAlignment','bottom','HorizontalAlignment','left')
text(x(1), y(1), ['(' num2str(x(1)) ', ' num2str(y(1)) ')'], 'VerticalAlignment','top','HorizontalAlignment','right')

legend('Sensor 1 Reading','Sensor 2 Reading', 'Estimated Position', 'Actual Position', 'Goal')
pause(0.1);

figure();
hold on
plot(t, vels);
plot(t, vel_est);
hold off
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Actual Velocity', 'Estimated Velocity');
title('Actual vs Estimated Velocity');

pause(0.1);

figure();
hold on
plot(t(1:length(t)-1), est_dist_to_goal);
xlabel('Time (s)')
ylabel('Distance (m)');
title('Estimated Distance to Goal');
hold off;



