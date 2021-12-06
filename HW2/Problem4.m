
%% Combined Potential Field

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
x_goal = 80;
y_goal = 20 ;

xi = 0.01;

attractive_2D = xi*((x - x_goal).^2 + (y - y_goal).^2);

p1 = [30, 50];
p2 = [50, 50];
p3 = [50, 70];
p4 = [30, 70];

c = [(p1(1) + p2(1))/2, (p2(2) + p3(2))/2]; % calculating center of obstacle

% distance to center of obstacle
d0 = 25;

% distance to center of obstacle
d = sqrt((x - c(1)).^2 + (y - c(2)).^2) + 0.01; 

d = d/5 + 0.5;

eta = 100;

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

s = surf(combined_field);
xlabel('x')
ylabel('y')
zlabel('z')
title('Combined Potential Field')

