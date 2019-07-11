%% Three-Link Planar Robot Manipulator Control

clc;
clear all;
close all;

%% Parameters

params = [];
params.l1       = 1;        % Length of link 1 [m]
params.l2       = 1;        % Length of link 2 [m]
params.l3       = 0;        % Length of link 3 [m]
params.lcm1     = 0.5;      % CM position of link 1 [m]
params.lcm2     = 0.5;      % CM position of link 2 [m]
params.lcm3     = 0.5*0;      % CM position of link 3 [m]
params.m1       = 50;       % Mass of link 1 [kg]
params.m2       = 50;       % Mass of link 2 [kg]
params.m3       = 50*0;       % Mass of link 2 [kg]
params.I_lcm1   = 10;       % Moment of inertia of link 1 [kg.m^2]
params.I_lcm2   = 10;       % Moment of inertia of link 2 [kg.m^2
params.I_lcm3   = 10;       % Moment of inertia of link 3 [kg.m^2]]
params.kr1      = 100*0;      % Gear ratio of motor at joint 1 [-]
params.kr2      = 100*0;      % Gear ratio of motor at joint 2 [-]
params.kr3      = 100*0;      % Gear ratio of motor at joint 3 [-]
params.m_motor1 = 5*0;        % Mass of rotor at joint 1 [kg]
params.m_motor2 = 5*0;        % Mass of rotor at joint 2 [kg]
params.m_motor3 = 5*0;        % Mass of rotor at joint 3 [kg]
params.I_motor1 = 0.01*0;     % Moment of inertia of rotor at joing 1 [kg.m^2]
params.I_motor2 = 0.01*0;     % Moment of inertia of rotor at joing 2 [kg.m^2]
params.I_motor3 = 0.01*0;     % Moment of inertia of rotor at joing 3 [kg.m^2]
params.g        = 9.81;     % Gravity [m/s^2]

%% Setup Kinematics

Ts = 10^-4;
tmax = 20;
tspan = [0:Ts:tmax]';
n = length(tspan);

%% Desired Trajectory 
x1 =  1.0;
y1 =  0;
z1 =  1.5;
x2 =  1.0;
y2 =  0;
z2 =  -1.5;

Xd = x1 + (x2 - x1)/tspan(end)*tspan;
Yd = y1 + (y2 - y1)/tspan(end)*tspan;
Zd1 = z1 + (z2 - z1)/(tspan(end)/2)*[0:Ts:tmax/2]';
Zd2 = z2 + (z1 - z2)/(tspan(end)/2)*[0+Ts:Ts:tmax/2]';
Zd = [Zd1;Zd2];

% Inverse kinematics
[theta1_d,theta2_d,theta3_d] = IK(Xd,Yd,Zd,params);

%% Plot Kinematics Profiles

figure;
set(gcf, 'Position', [350 100 2000 1200]/2);
plot3(Xd,Yd,Zd,'LineWidth', 2, 'Color', 'g');
grid on;
view(0,0);
% figure;
% set(gcf, 'Position', [0 0 2560 1280]/2);
% for ii = 1:1
%     
%     subplot(2,3,1); 
%     plot(Xd, Yd, 'LineWidth', 2, 'Color', 'k'); xlabel('X [m]'); ylabel('Y [m]'); axis([-2 2 -2 2]); set(gca, 'FontSize', 16); grid on;
% 
%     subplot(2,3,4); 
%     plot(theta1_d*180/pi, theta2_d*180/pi, 'LineWidth', 2, 'Color', [1 0.5 0]); xlabel('Theta1d [deg]'); ylabel('Theta2d [deg]'); set(gca, 'FontSize', 16); grid on;
% 
%     subplot(2,3,2); plot(tspan, Xd, 'LineWidth', 2, 'Color', 'b'); xlabel('Time [s]'); ylabel('X [m]');set(gca, 'FontSize', 16); grid on;
%     subplot(2,3,5); plot(tspan, Yd, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('Y [m]');set(gca, 'FontSize', 16); grid on;
%     subplot(2,3,3); plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'Color', 'm'); xlabel('Time [s]'); ylabel('Theta1d [deg]');set(gca, 'FontSize', 16); grid on;
%     subplot(2,3,6); plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'Color', 'c'); xlabel('Time [s]'); ylabel('Theta2d [deg]');set(gca, 'FontSize', 16); grid on;
% 
% end


%% Desired Kinematics Profiles (Theta_d, Theta_d_dot, Theta_d_ddot

% Start at Zero Velocity
theta1_d_dot = [0; diff(theta1_d)/Ts];
theta2_d_dot = [0; diff(theta2_d)/Ts];
theta3_d_dot = [0; diff(theta3_d)/Ts];

% Start at Zero Acceleration
theta1_d_ddot = [0; diff(theta1_d_dot)/Ts]; 
theta2_d_ddot = [0; diff(theta2_d_dot)/Ts];
theta3_d_ddot = [0; diff(theta3_d_dot)/Ts];

Kinematics_Profiles = [];

Kinematics_Profiles.theta1_d      = theta1_d;
Kinematics_Profiles.theta1_d_dot  = theta1_d_dot;
Kinematics_Profiles.theta1_d_ddot = theta1_d_ddot;

Kinematics_Profiles.theta2_d      = theta2_d;
Kinematics_Profiles.theta2_d_dot  = theta2_d_dot;
Kinematics_Profiles.theta2_d_ddot = theta2_d_ddot;

Kinematics_Profiles.theta3_d      = theta3_d;
Kinematics_Profiles.theta3_d_dot  = theta3_d_dot;
Kinematics_Profiles.theta3_d_ddot = theta3_d_ddot;

figure;
set(gcf, 'Position', [350 100 2000 1200]/2);
for ii = 1:1
    
    % Theta1 --------------------------------------------------------------
    
    subplot(3,3,1); plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d [deg]'); set(gca, 'FontSize', 11); grid on;
    
    subplot(3,3,4); plot(tspan, theta1_d_dot*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d-dot [deg/s]'); set(gca, 'FontSize', 11); grid on;
    
    subplot(3,3,7); plot(tspan, theta1_d_ddot*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d-ddot [deg/s^2]'); set(gca, 'FontSize', 11); grid on;ylim([-5 max(theta1_d_ddot*180/pi)])
    
    % Theta2 --------------------------------------------------------------
    subplot(3,3,2); plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta2d [deg]'); set(gca, 'FontSize', 11); grid on;
    
    subplot(3,3,5); plot(tspan, theta2_d_dot*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta2d-dot [deg/s]'); set(gca, 'FontSize', 11); grid on;
    
    subplot(3,3,8); plot(tspan, theta2_d_ddot*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta2d-ddot [deg/s^2]'); set(gca, 'FontSize', 11); grid on; ylim([-30 5]);
    
    % Theta3 --------------------------------------------------------------
    subplot(3,3,3); plot(tspan, theta3_d*180/pi, 'LineWidth', 2, 'Color', [1  0  0]); 
    xlabel('Time'); ylabel('Theta3d [deg]'); set(gca, 'FontSize', 11); grid on;
    
    subplot(3,3,6); plot(tspan, theta3_d_dot*180/pi, 'LineWidth', 2, 'Color', [1  0  0]); 
    xlabel('Time'); ylabel('Theta3d-dot [deg/s]'); set(gca, 'FontSize', 11); grid on;
    
    subplot(3,3,9); plot(tspan, theta3_d_ddot*180/pi, 'LineWidth', 2, 'Color', [1  0  0]); 
    xlabel('Time'); ylabel('Theta3d-ddot [deg/s^2]'); set(gca, 'FontSize', 11); grid on; ylim([-30 5]);
end

%% Solve EOM (Open Loop)

%initial condition [position:velocity]
X0 = [pi/2*0;0;0;0;0;0];
   
fun = @(t,X)Neom_3DOF_planar_robot_manipulator(t, X, params);
[tout, Xout] = ode45(fun, tspan, X0, []);

theta1     = Xout(:,1);
theta1_dot = Xout(:,2);
theta2     = Xout(:,3);
theta2_dot = Xout(:,4);
theta3     = Xout(:,5);
theta3_dot = Xout(:,6);

%% Simulation Kinematics Plots

ind_plot = 1;
if(ind_plot)
    
figure;
set(gcf, 'Position', [350 100 2000 1200]/2);
grid on;
axis(2*[-1 1 -1 1 -1 1]);
grid on;
for ii = 1:100:length(theta1) 
    %link 1
     plot3([0 params.l1*cos(theta1(ii))*cos(theta3(ii))],...
          [0 params.l1*cos(theta1(ii))*sin(theta3(ii))],...
          [0 params.l1*sin(theta1(ii))],...
           'linewidth', 2, 'Color', 'r');        
    hold all;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    set(gca, 'FontSize', 16);
    view(0,0);
    %motor 2
     plot3(params.l1*cos(theta1(ii))*cos(theta3(ii)),...
           params.l1*cos(theta1(ii))*sin(theta3(ii)),...
           params.l1*sin(theta1(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    %link 2 
    plot3([params.l1*cos(theta1(ii))*cos(theta3(ii)) (params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*cos(theta3(ii))],...
          [params.l1*cos(theta1(ii))*sin(theta3(ii)) (params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*sin(theta3(ii))],...
          [params.l1*sin(theta1(ii)) params.l3+params.l1*sin(theta1(ii))+params.l2*sin(theta1(ii)+theta2(ii))],...
        'linewidth', 2, 'Color', 'g');
    %End effector
    plot3((params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*cos(theta3(ii)),...
          (params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*sin(theta3(ii)),...
           params.l3+params.l1*sin(theta1(ii))+params.l2*sin(theta1(ii)+theta2(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor','g', 'MarkerEdgeColor', 'g');
    %Axis
    plot3([-2 2],[0 0],[0 0],'LineWidth', 2, 'Color', 'k');
    plot3([0 0],[-2 2],[0 0],'LineWidth', 2, 'Color', 'k');
    plot3([0 0],[0 0] ,[-2 2],'LineWidth', 2, 'Color', 'k');
    pause( 0.1 );
    if (ii~= length(theta1))
        clf;
    else
        % Do nothing, Do not clear figure if not final post
    end
    
end

end % if(ind_plot)

%% Closed Loop EOM
clc;
close all;

X0 = [0;0;0;0;0;0];
   
fun_fb = @(t,X)eom_3DOF_planar_robot_manipulator_feedback(t, X, tspan, params, Xd, Yd, Kinematics_Profiles);
[tout_fb, Xout_fb] = ode45(fun_fb, tspan, X0, []);

theta1_fb     = Xout_fb(:,1);
theta1_dot_fb = Xout_fb(:,2);
theta2_fb     = Xout_fb(:,3);
theta2_dot_fb = Xout_fb(:,4);
theta3_fb     = Xout_fb(:,5);
theta3_dot_fb = Xout_fb(:,6);

l1 = params.l1;
l2 = params.l2;
l3 = params.l3;

X_fb = (l1*cos(theta1_fb) + l2*cos(theta1_fb+theta2_fb)).*cos(theta3_fb);
Y_fb = (l1*sin(theta1_fb) + l2*sin(theta1_fb+theta2_fb)).*sin(theta3_fb);
Z_fb = l3+l1*sin(theta1_fb)+l2*sin(theta1_fb+theta2_fb);

% ind_plot = 1;
% if(ind_plot)
% for ii = 1:1
%     figure;
%     set(gcf, 'Position', [0 0 2560 1280]/2);
    
    % X-Y
%     subplot(2,3,[1]); 
    plot3(Xd, Yd, Zd,'LineWidth', 2, 'Color', 'm'); 
    view(0,0);
    hold on;
    plot3([-2 2],[0 0],[0 0],'LineWidth', 2, 'Color', 'r');
    plot3([0 0],[-2 2],[0 0],'LineWidth', 2, 'Color', 'r');
    plot3([0 0],[0 0] ,[-2 2],'LineWidth', 2, 'Color', 'r');
    plot3(X_fb, Y_fb,Z_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    xlabel('X [m]'); ylabel('Y [m]');zlabel('Z [m]'); axis([-2 2 -2 2 -2 2]); set(gca, 'FontSize', 16); grid on;
    
%     % Theta1 - Theta2
%     subplot(2,3,[4]); 
%     plot(theta1_d*180/pi, theta2_d*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', [1 0.5 0]); 
%     hold on;
%     plot(theta1_fb*180/pi, theta2_fb*180/pi, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
%     xlabel('Theta1d [deg]'); ylabel('Theta2d [deg]'); set(gca, 'FontSize', 16); grid on;
% 
%     
%     % X 
%     subplot(2,3,2); 
%     plot(tspan, Xd, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'c'); 
%     hold on;
%     subplot(2,3,2); plot(tspan, X_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k');
%     xlabel('Time [s]'); ylabel('X [m]');set(gca, 'FontSize', 16); grid on;
%     
%     
%     % Y
%     subplot(2,3,5); 
%     plot(tspan, Yd, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g'); 
%     hold on;
%     plot(tspan, Y_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
%     xlabel('Time [s]'); ylabel('Y [m]');set(gca, 'FontSize', 16); grid on;
%     
%     subplot(2,3,3); 
%     plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'm'); 
%     hold on;
%     plot(tspan, theta1_fb*180/pi, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
%     xlabel('Time [s]'); ylabel('Theta1d [deg]');set(gca, 'FontSize', 16); grid on;
%     
%     
%     subplot(2,3,6); 
%     plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'c'); 
%     hold on;
%     plot(tspan, theta2_fb*180/pi, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
%     xlabel('Time [s]'); ylabel('Theta2d [deg]');set(gca, 'FontSize', 16); grid on;

    
    
% end % for ii = 1:1
% end % if(ind_plot)


%% Simulation+Control

ind_plot = 1;
if(ind_plot)
    
figure;
set(gcf, 'Position', [350 100 2000 1200]/2);
grid on;
axis(2*[-1 1 -1 1 -1 1]);
grid on;

for ii = 1:100:length(theta1_fb) 
    plot3([x1 x2],[y1 y2],[z1 z2],'linewidth', 3, 'Color', 'b');
    hold all;
    plot3(X_fb, Y_fb,Z_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    %link 1
     plot3([0 params.l1*cos(theta1_fb(ii))*cos(theta3_fb(ii))],...
          [0 params.l1*cos(theta1_fb(ii))*sin(theta3_fb(ii))],...
          [0 params.l1*sin(theta1_fb(ii))],...
           'linewidth', 2, 'Color', 'r');   
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    set(gca, 'FontSize', 16);
    view(0,0);
    %motor 2
     plot3(params.l1*cos(theta1_fb(ii))*cos(theta3_fb(ii)),...
           params.l1*cos(theta1_fb(ii))*sin(theta3_fb(ii)),...
           params.l1*sin(theta1_fb(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    %link 2 
    plot3([params.l1*cos(theta1_fb(ii))*cos(theta3_fb(ii)) (params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*cos(theta3_fb(ii))],...
          [params.l1*cos(theta1_fb(ii))*sin(theta3_fb(ii)) (params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*sin(theta3_fb(ii))],...
          [params.l1*sin(theta1_fb(ii)) params.l3+params.l1*sin(theta1_fb(ii))+params.l2*sin(theta1_fb(ii)+theta2_fb(ii))],...
        'linewidth', 2, 'Color', 'g');
    %End effector
    plot3((params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*cos(theta3_fb(ii)),...
          (params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*sin(theta3_fb(ii)),...
           params.l3+params.l1*sin(theta1_fb(ii))+params.l2*sin(theta1_fb(ii)+theta2_fb(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor','g', 'MarkerEdgeColor', 'g');
    %Axis
    plot3([-2 2],[0 0],[0 0],'LineWidth', 2, 'Color', 'k');
    plot3([0 0],[-2 2],[0 0],'LineWidth', 2, 'Color', 'k');
    plot3([0 0],[0 0] ,[-2 2],'LineWidth', 2, 'Color', 'k');
    pause( 0.1 );
    if (ii~= length(theta1_fb))
        clf;
    else
        % Do nothing, Do not clear figure if not final post
    end
    
end

end % if(ind_plot)



