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
x_goal = 80;
y_goal = 20 ;

xi = 0.01;

attractive_2D = xi*((x - x_goal).^2 + (y - y_goal).^2);

% finding minimum value of function
minVal = min(attractive_2D(:));
% idx of fmin 
minIdx = attractive_2D == minVal;

% plotting surface
s = surf(x, y, attractive_2D);
hold on
% offset in z direction to make text visible
hold off
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Attractive Potential Field')








