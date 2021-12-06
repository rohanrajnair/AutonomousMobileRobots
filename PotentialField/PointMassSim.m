
%% Point Mass Vehicle

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

% start and goal coordinates

q_start = [19, 80];
q_goal = [80, 20];

p1 = [30, 50];
p2 = [50, 50];
p3 = [50, 70];
p4 = [30, 70];

c = [(p1(1) + p2(1))/2, (p2(2) + p3(2))/2]; % calculating center of obstacle

d0 = 25;
xi = 5;
eta = 1000;

alpha = 0.05;

q_x(1) = q_start(1);
q_y(1) = q_start(2);

i = 1;
thresh = 2;

% check distance from goal
while (sqrt((q_x(i) - q_goal(1))^2 + (q_y(i) - q_goal(2))^2) > thresh)
    % updating current x and y
    
    dist = sqrt((q_x(i) - c(1))^2 + (q_y(i) - c(2))^2);
    dist = dist/5 + 0.01;
    
    % F = negative gradient of U 
    gx = -xi*(q_x(i) - q_goal(1)) + eta*(1/dist - 1/d0)*(1/dist^2)*(q_x(i) - c(1))/dist;
    gy = -xi*(q_y(i) - q_goal(2)) + eta*(1/dist - 1/d0)*(1/dist^2)*(q_y(i) - c(2))/dist;

    gx = min(abs(gx),5)*sign(gx);
    gy = min(abs(gy),5)*sign(gy);
    
    q_x(i+1) = q_x(i) + gx*alpha;
    q_y(i+1) = q_y(i) + gy*alpha;
    
    %hold on
    plot(q_x(i), q_y(i), 'r*')
    %hold off
    xlim([0 100])
    ylim([0 100])
    drawnow
    i = i + 1;
end
title("Point Mass Robot Trajectory")







