close all;
clear all;
clc;

%% Set variable 
% Use 123 frame as a reference frame
syms b s s_dot c th th_dot ome
R = 0;
R_dot = 0;
R_dot2 = 0;
rho = [b+(s+c)*sin(th);...
              0       ;...
       (s+c)*cos(th)];
v_rel = [(s+c)*th_dot*cos(th)+s_dot*sin(th);...
                         0                 ;...
        -(s+c)*sin(th)*th_dot+s_dot*cos(th)];
a_rel = [-(s+c)*th_dot^2*sin(th)+2*s_dot*th_dot*cos(th);...
                         0                     ;...
         -(s+c)*th_dot^2*cos(th)-2*s_dot*th_dot*sin(th)];
ang_vel = [0 -ome 0;...
           ome 0  0;...
            0  0  0 ];
ang_acc = 0;
velocity = vel(R_dot,v_rel,ang_vel,rho);
acceleration = acc(R_dot2,a_rel,ang_vel,rho,ang_acc,v_rel);
velocity
acceleration

%% Substitution 
b = 0.1; %m
s_dot = 1; %m/s
c = 0.5; %m
th_dot = 5; %rad/s
ome = 10; %rad/s
time = [0:0.01:10]; %s
for i = 1:length(time);
    th = th_dot*time(i); %rad
    s = s_dot*time(i); %m
    vel_subs = subs(velocity);
    acc_subs = subs(acceleration);
    vel_mag(i) = norm(vel_subs);
    acc_mag(i) = norm(acc_subs);
end
vel_mag = double(vel_mag);
acc_mag = double(acc_mag);
%% Plot
plot(time,vel_mag,time,acc_mag);
legend('velocity [m/s]','acceleration [m/s^2]','Location','northwest');
grid on;
xlabel('Time[s]');
title ('v-t,a-t');
%% Set function
function v = vel(R_dot,v_rel,ang_vel,rho)
v = R_dot+v_rel+ang_vel*rho;
end
function a = acc(R_dot2,a_rel,ang_vel,rho,ang_acc,v_rel)
a = R_dot2+a_rel+ang_vel*ang_vel*rho+ang_acc*rho+2*ang_vel*v_rel;
end