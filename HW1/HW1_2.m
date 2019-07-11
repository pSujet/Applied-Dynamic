%% HW1_2 in x2y2z2 reference frame
close all;
clear all;
clc;

%% Set variable 
% Use x2y2z2 frame as a reference frame
syms a t w0 L
assume(t, 'real');
alpha = a*sin(3*t);
alpha_dot = 3*a*cos(3*t);
alpha_dot2 = -9*a*sin(3*t);
R = 0;
R_dot = 0;
R_dot2 = 0;
rho = [0;0;L];
v_rel = zeros(3,1);
a_rel = zeros(3,1);
ang_vel = [       0        -w0*cos(alpha) w0*sin(alpha)  ;...
           w0*cos(alpha)        0           -alpha_dot   ;...
           -w0*sin(alpha)    alpha_dot           0      ];
ang_acc = [              0             alpha_dot*w0*sin(alpha)   alpha_dot*w0*cos(alpha);...
           -alpha_dot*w0*sin(alpha)            0                        -alpha_dot2       ;...
           -alpha_dot*w0*cos(alpha)         alpha_dot2                       0           ];
velocity = vel(R_dot,v_rel,ang_vel,rho);
acceleration = acc(R_dot2,a_rel,ang_vel,rho,ang_acc,v_rel);
velocity
acceleration

%% Substitution 
L = 1; %m
a = 1; %1/s
w0 = 5; %rad/s
time = [0:0.01:10]; %s
for i = 1:length(time);
    t = time(i);
    vel_subs = subs(velocity);
    acc_subs = subs(acceleration);
    vel_mag(i) = norm(vel_subs);
    acc_mag(i) = norm(acc_subs);
end
vel_mag = double(vel_mag);
acc_mag = double(acc_mag);

%% Plot
plot(time,vel_mag,time,acc_mag);
legend('velocity [m/s]','acceleration [m/s^2]');
grid on;
xlabel('Time[s]');
title ('v-t, a-t');

%% Set function
function v = vel(R_dot,v_rel,ang_vel,rho)
v = R_dot+v_rel+ang_vel*rho;
end
function a = acc(R_dot2,a_rel,ang_vel,rho,ang_acc,v_rel)
a = R_dot2+a_rel+ang_vel*ang_vel*rho+ang_acc*rho+2*ang_vel*v_rel;
end