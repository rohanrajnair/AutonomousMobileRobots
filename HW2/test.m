
%% Attractive Potential Field

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

[x, y] = meshgrid (1:100, 1:100);

% goal coordinates

q_start = [10, 80];

q_goal = [80, 20];

xi = 100;

attractive_2D = xi*((x - q_goal(1)).^2 + (y - q_goal(2)).^2);

p1 = [30, 50];
p2 = [50, 50];
p3 = [50, 70];
p4 = [30, 70];

c = [(p1(1) + p2(1))/2, (p2(2) + p3(2))/2]; % calculating center of obstacle

d0 = 10;

% distance to center of obstacle
d = sqrt((x - c(1)).^2 + (y - c(2)).^2); 

% d = d/5 + 0.5;


eta = 500;

for i = 1:100
    for j = 1:100
        if d(i,j) > d0
            repulsive_2D(i,j) = 0;
        else
            repulsive_2D(i,j) = 0.5*eta*(1/d(i,j) - 1/d0)^2;
        end
    end
end


combined_field = attractive_2D + repulsive_2D;

contour(combined_field);
hold on

alpha = 0.1;

q_x(1) = q_start(1);
q_y(1) = q_start(2);

i = 1;

% % computing gradient in x and y directions
[gx, gy] = gradient(combined_field);
% 
%mag = sqrt(gx.^2 + gy.^2);

% % % normalizing gradients
% gx = gx./mag;
% gy = gy./mag;

thresh = 3;

% threshold = 0.1
while (sqrt((q_x(i) - q_goal(1))^2 + (q_y(i) - q_goal(2))^2) > thresh)
    % updating current x and y
    curr_gx = gx(round(q_x(i)), round(q_y(i)));
    curr_gy = gy(round(q_x(i)), round(q_y(i)));
    
    curr_gx = min(abs(curr_gx),5)*sign(curr_gx);
    curr_gy = min(abs(curr_gy),5)*sign(curr_gy);
    q_x(i+1) = q_x(i) - curr_gx * alpha;
    q_y(i+1) = q_y(i) - curr_gy * alpha;
    
%     hold on
    plot(q_x(i), q_y(i), 'r*')
%     hold off
    xlim([0 100])
    ylim([0 100])
    i = i + 1;
end







