%% HW1_3 in xyz reference frame
close all;
clear all;
clc;

%% Set variable 
% Use xyz frame as a reference frame
syms a b w0 w1 w2 phi
assume(phi, 'real');
R = [b;0;0];
R_dot = [0;0;-w2*b];
R_dot2 = [-w2^2*b;0;0];
rho = [-a*sin(phi)  ;...
        a*cos(phi)  ;...
            0      ];
v_rel = [-a*w0*cos(phi) ;...
         -a*w0*sin(phi) ;...
               0       ];
a_rel = [a*w0^2*sin(phi);...
         -a*w0^2*cos(phi);...
               0       ];
ang_vel = [0    0   w2  ;...
           0    0   w1  ;...
          -w2  -w1  0  ];
ang_acc = zeros(3);
velocity = vel(R_dot,v_rel,ang_vel,rho);
acceleration = acc(R_dot2,a_rel,ang_vel,rho,ang_acc,v_rel);
velocity
acceleration

%% Substitution 
a = 0.1; %m
b = 1; %m
w0 = 10; %rad/s
w1 = 5; %rad/s
w2 = 15; %rad/s
time = [0:0.01:10]; %s
for i = 1:length(time);
    phi = w0*time(i); %rad
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