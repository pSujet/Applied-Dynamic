%% 3-DOFs Scalar Robot Manipular Dynamics and Control
clc;
close all;
clear all;

%% Parameters
l1 = 1; %m
l2 = 1; %m
l3 = 0; %m
lcm1 = 0.5; %m
lcm2 = 0.5; %m
m1 = 50; %kg
m2 = 50; %kg
m3 = 50; %kg
Icm1 = 10; %kgm^2
Icm2 = 10; %kgm^2
Icm3 = 10; %kgm^2
kr1 = 100;
kr2 = 100;
kr3 = 100;
% u1
% u2
% u3
Imotor1 = 0.01; %kgm^2
Imotor2 = 0.01; %kgm^2
Imotor3 = 0.01; %kgm^2
g = 9.81; %kgm/s^2

%% Forward kinematic
[x1,y1,z1] = FK(-29.08,128.68,45.00,l1,l2,l3);
fprintf('position = (%4.5f,%4.5f,%4.5f)\n',x1,y1,z1)

%% Inverse kinematic
%output
[theta1,theta2,theta3] = IK(0.5,0.5,0.5,l1,l2,l3);
fprintf('motor angle = (%4.2f,%4.2f,%4.2f)\n',theta1,theta2,theta3)

%% Plot
q = linspace(0,2,100);
x1 = [2 1.5 1 0.5 0 ];
y1 = [0 1 1.5 0.5 1 ];
z1 = sqrt(4-x1.^2-y1.^2);
[theta1,theta2,theta3] = IK(x1,y1,z1,l1,l2,l3);
[x2,y2,z2] = FK(theta1,theta2,theta3,l1,l2,l3);
figure('Position',[320 200 800 400]);
subplot(1,2,1)
plot3(x1,y1,z1,'LineStyle', '-', 'LineWidth', 2,...
      'Color',[0 0 1],'Marker', 'o');
xlabel('PositionX [m]');
ylabel('PositionY [m]');
zlabel('PositionZ [m]');
title('Desired Trajectory');
grid on;

subplot(1,2,2)
plot3(x2,y2,z2,'LineStyle', '-', 'LineWidth', 2,...
      'Color',[1 0 0],'Marker', 'o');
xlabel('PositionX [m]');
ylabel('PositionY [m]');
zlabel('PositionZ [m]');
title('Calculated Trajectory');
grid on;

%% Equation of Motion

