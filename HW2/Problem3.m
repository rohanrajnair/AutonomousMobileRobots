%% Radial Repulsive Potential Field

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

p1 = [30, 50];
p2 = [50, 50];
p3 = [50, 70];
p4 = [30, 70];

c = [(p1(1) + p2(1))/2, (p2(2) + p3(2))/2]; % calculating center of obstacle

d0 = 25;

% distance to center of obstacle
d = sqrt((x - c(1)).^2 + (y - c(2)).^2); 

d = d./5 + 0.5;

eta = 10;

for i = 1:100
    for j = 1:100
        if d(i,j) > d0
            repulsive_2D(i,j) = 0;
        else
            repulsive_2D(i,j) = 0.5*eta*((1/(d(i,j)+0.01) - 1/d0)^2);
        end
    end
end

s = surf(x, y, repulsive_2D);

grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Radial Repulsive Potential Field')


