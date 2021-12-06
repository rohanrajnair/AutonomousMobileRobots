
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

for i = 1:100 % rows (y)
    for j = 1:100 % cols (x)
        if i > p3(2) % above obstacle
            if j < p4(1) % left of and above obstacle
                d(i,j) = sqrt((j - p4(1))^2 + (i - p4(2))^2); % dist to top left corner
            elseif j > p3(1) % right of and above obstacle
                d(i,j) = sqrt((j - p3(1))^2 + (i - p3(2))^2); % top right corner
            else % directly above obstacle
                d(i,j) = i - p3(2); % vertical distance 
            end
            
        elseif i < p1(2) % below obstacle
            if j < p1(1) % left of and below obstacle
                d(i,j) = sqrt((j - p1(1))^2 + (i - p1(2))^2); % bottom left corner
            elseif j > p2(1) % right of and below obstacle
                d(i,j) = sqrt((j - p2(1))^2 + (i - p2(2))^2); % bottom right corner
            else % directly below of obstacle
                d(i,j) = p1(2) - i; % vertical distance 
            end
        else
            if j > p2(1) % right of obstacle
                d(i,j) = j - p2(1);
            elseif j < p1(1)
                d(i,j) = p1(1) - j;
            else
                d(i,j) = 0.01;
            end
        end
    end
end
        

d = d./5 + 0.5;

eta = 100;
d0 = 25;

for i = 1:100
    for j = 1:100
        if d(i,j) > d0
            repulsive_2D(i,j) = 0;
        else
            repulsive_2D(i,j) = 0.5*eta*((1/(d(i,j)+0.01) - 1/d0)^2);
        end
    end
end



combined_field = attractive_2D + repulsive_2D;

s = surf(combined_field);
xlabel('x')
ylabel('y')
zlabel('z')
title('Combined Potential Field (Square Obstacle)')

