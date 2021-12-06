%% Particle Filter Implementation

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

K_prop = 0.5;

%Initialize a vector of positions for the robot
x=[]; 
y=[];

theta = []; 

vels = [];
vels(1) = 3;

t = [];

%time step
dt = 0.1;

%% Robot Initial Pose

% Initial Orientation 
x(1) = abs(50*rand(1,1)) + 25;
y(1) = abs(50*rand(1,1)) + 25;

% Initial Orientation 
theta(1) = 2*pi*rand(1,1);

% Build Robot Model
robot = QuadRobot(x,y,theta(1));

% Landmarks

landmarks = [25 25; 25 70; 70 25; 70 70; 10 40; 80 60];

sigma = 8;

% initialize particles
particle_x =abs(rand(1,1000)*100);
particle_y=abs(rand(1,1000)*100);
particle_theta = ones(1,1000)*theta(1) + randn(1,1000)*0.5*pi/180;
particles = cat(1,particle_x, particle_y, particle_theta);

init_est = [mean(particles(1,:), 'all'), mean(particles(2,:), 'all'), mean(particles(3,:), 'all')];
init_est_robot = QuadRobot(init_est(1), init_est(2), init_est(3));

%figure();
hold on
plot(init_est_robot(:,1), init_est_robot(:,2), 'r', robot(:,1),robot(:,2),'k');
scatter(particle_x, particle_y, 'c.');
hold off
legend('Estimate', 'Actual Vehicle');
pause(0.01);

i = 1;
for steps = 1:10

    theta_d = theta(i) + 0.2;

    % calculating steering angle
    gamma = K_prop*atan2(sin(theta_d - theta(i)), cos(theta_d - theta(i)));

    vels(i+1) = vels(i) + randn*0.5;

    steering_change = min(steering, gamma);

    x(i+1) = x(i) + vels(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vels(i) * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + steering_change * dt + randn*0.5*pi/180; 
    robot = QuadRobot(x(i+1),y(i+1),theta(i+1));
    
    % simulate motion on particles w/ noise
    particles(1,:) = particles(1,:) + (vels(i) * cos(theta(i)) * dt) + randn(1,1000)*0.5;
    particles(2,:) = particles(2,:) + (vels(i) * sin(theta(i)) * dt) + randn(1,1000)*0.5;
    particles(3,:) = particles(3,:) + steering_change * dt + randn(1,1000)*0.5;
    
    t(i+1) = i; 
    i = i + 1;
    
    imp_weights = zeros(1,1000);
    
    % calculate importance weights
    for p = 1:1000
        prob = 1;
        for l = 1:size(landmarks,1)
            m = sqrt((x(i) - landmarks(l,1))^2 + (y(i) - landmarks(l,2))^2);
            d = sqrt((particles(1,p) - landmarks(l,1))^2 + (particles(2,p) - landmarks(l,2))^2);
            prob = prob * (1/sqrt(2*pi*sigma)) * exp(-0.5*((d-m)/sigma)^2);
        end
        imp_weights(p) = prob;
    end
    
    % normalize importance weights
    imp_weights = imp_weights ./ sum(imp_weights);
    
    % choosing most likely particles 
    filtered_particles = zeros(3,1000); % x, y, theta
    % choose index from uniform distribution
    idx = randi([1,1000]);
    beta = 0;
    for j = 1:1000
        beta = beta + 2*max(imp_weights)*rand;
        while (imp_weights(idx) < beta)
            beta = beta - imp_weights(idx);
            idx = mod(idx + 1, 1000) + 1;
        end
        filtered_particles(:,j) = particles(:,idx);
    end
    
    est = [mean(filtered_particles(1,:), 'all'), mean(filtered_particles(2,:), 'all'), mean(filtered_particles(3,:), 'all')];
    est_robot = QuadRobot(est(1), est(2), est(3));
    %figure();
    plot(est_robot(:,1), est_robot(:,2),'r', robot(:,1),robot(:,2),'k-',x,y,'-', landmarks(:,1), landmarks(:,2), 'ro');
    hold on
    scatter(filtered_particles(1,:), filtered_particles(2,:), 'c.')
    particles = filtered_particles;
    hold off
    xlim([0 100])
    ylim([0 100])
    legend('Estimate', 'Actual Vehicle');
    pause(0.01)
end