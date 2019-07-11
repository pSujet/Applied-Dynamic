%% Two-Link Planar Robot Manipulator Control

clc;
clear all;
close all;


%% Parameters

params = [];
params.l1       = 1;        % Length of link 1 [m]
params.l2       = 1;        % Length of link 2 [m]
params.lcm1     = 0.5;      % CM position of link 1 [m]
params.lcm2     = 0.5;      % CM position of link 2 [m]
params.m1       = 50;       % Mass of link 1 [kg]
params.m2       = 50;       % Mass of link 2 [kg]
params.I_lcm1   = 10;       % Moment of inertia of link 1 [kg.m^2]
params.I_lcm2   = 10;       % Moment of inertia of link 2 [kg.m^2]
params.kr1      = 100*0;      % Gear ratio of motor at joint 1 [-]
params.kr2      = 100*0;      % Gear ratio of motor at joint 2 [-]
params.m_motor1 = 5*0;        % Mass of rotor at joint 1 [kg]
params.m_motor2 = 5*0;        % Mass of rotor at joint 2 [kg]
params.I_motor1 = 0.01*0;     % Moment of inertia of rotor at joing 1 [kg.m^2]
params.I_motor2 = 0.01*0;     % Moment of inertia of rotor at joing 2 [kg.m^2]
params.g        = 9.81;     % Gravity [m/s^2]

l1 = params.l1;
l2 = params.l2;
lcm1 = params.lcm1;
lcm2 = params.lcm2;


%% Setup

Ts = 10^-3;
tspan = [0:Ts:10]';
n = length(tspan);

% Desired Trajectory 
x1 =  1.0;
y1 =  1.5;
x2 =  1.0;
y2 = -1.5;

Xd = x1 + (x2 - x1)/tspan(end)*tspan;
Yd1 = y1 + (y2 - y1)/(tspan(end)/2)*[0:Ts:5]';
Yd2 = y2 + (y1 - y2)/(tspan(end)/2)*[0+Ts:Ts:5]';
Yd = [Yd1; Yd2];

theta2_d = acos( (Xd.^2 + Yd.^2 - l1^2 - l2^2)/(2*l1*l2) );
theta1_d = atan( Yd./Xd ) - atan( l2*sin(theta2_d)./(l1 + l2*cos(theta2_d)) );

figure;
set(gcf, 'Position', [0 0 2560 1280]/2);
for ii = 1:1
    
    subplot(2,3,[1]); 
    plot(Xd, Yd, 'LineWidth', 2, 'Color', 'k'); xlabel('X [m]'); ylabel('Y [m]'); axis([-2 2 -2 2]); set(gca, 'FontSize', 16); grid on;

    subplot(2,3,[4]); 
    plot(theta1_d*180/pi, theta2_d*180/pi, 'LineWidth', 2, 'Color', [1 0.5 0]); xlabel('Theta1d [deg]'); ylabel('Theta2d [deg]'); set(gca, 'FontSize', 16); grid on;

    subplot(2,3,2); plot(tspan, Xd, 'LineWidth', 2, 'Color', 'b'); xlabel('Time [s]'); ylabel('X [m]');set(gca, 'FontSize', 16); grid on;
    subplot(2,3,5); plot(tspan, Yd, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('Y [m]');set(gca, 'FontSize', 16); grid on;
    subplot(2,3,3); plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'Color', 'm'); xlabel('Time [s]'); ylabel('Theta1d [deg]');set(gca, 'FontSize', 16); grid on;
    subplot(2,3,6); plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'Color', 'c'); xlabel('Time [s]'); ylabel('Theta2d [deg]');set(gca, 'FontSize', 16); grid on;

end


%% Desired Kinematics Profiles (Theta_d, Theta_d_dot, Theta_d_ddot

theta1_d_dot = [0; diff(theta1_d)/Ts]; % Start at Zero Velocity
theta2_d_dot = [0; diff(theta2_d)/Ts];

theta1_d_ddot = [0; diff(theta1_d_dot)/Ts]; % Start at Zero Acceleration
theta2_d_ddot = [0; diff(theta2_d_dot)/Ts];

Kinematics_Profiles = [];

Kinematics_Profiles.theta1_d      = theta1_d;
Kinematics_Profiles.theta1_d_dot  = theta1_d_dot;
Kinematics_Profiles.theta1_d_ddot = theta1_d_ddot;

Kinematics_Profiles.theta2_d      = theta2_d;
Kinematics_Profiles.theta2_d_dot  = theta2_d_dot;
Kinematics_Profiles.theta2_d_ddot = theta2_d_ddot;

figure;
set(gcf, 'Position', [0 0 2560 1280]/2);
for ii = 1:1
    
    % Theta1 --------------------------------------------------------------
    
    subplot(3,2,1); plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d [deg]'); set(gca, 'FontSize', 16); grid on;
    
    subplot(3,2,3); plot(tspan, theta1_d_dot*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d-dot [deg/s]'); set(gca, 'FontSize', 16); grid on;
    
    subplot(3,2,5); plot(tspan, theta1_d_ddot*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d-ddot [deg/s^2]'); set(gca, 'FontSize', 16); grid on; ylim([-5 max(theta1_d_ddot*180/pi)]);
    
    % Theta2 --------------------------------------------------------------
    subplot(3,2,2); plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta1d [deg]'); set(gca, 'FontSize', 16); grid on;
    
    subplot(3,2,4); plot(tspan, theta2_d_dot*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta1d-dot [deg/s]'); set(gca, 'FontSize', 16); grid on;
    
    subplot(3,2,6); plot(tspan, theta2_d_ddot*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta1d-ddot [deg/s^2]'); set(gca, 'FontSize', 16); grid on; ylim([-30 5]);
    
end




%% Solve EOM (Open Loop)

X0 = [-pi/2*0;
       0;
       0;
       0];
   
fun = @(t,X)eom_2DOF_planar_robot_manipulator(t, X, params);
[tout, Xout] = ode45(fun, tspan, X0, []);

theta1     = Xout(:,1);
theta1_dot = Xout(:,2);
theta2     = Xout(:,3);
theta2_dot = Xout(:,4);




%% Kinematics Plots

ind_plot = 1;
if(ind_plot)
    
figure;
set(gcf, 'Position', [0 0 1280 1280]/2);
grid on;
axis(2*[-1 1 -1 1]);
grid on;
for ii = 1:100:length(theta1)

    
    line([0  params.l1*cos(theta1(ii))], [0 params.l1*sin(theta1(ii))], 'linewidth', 2, 'Color', 'g');
    hold all;
    plot(0, 0, 'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
    
    plot(params.l1*cos(theta1(ii)), params.l1*sin(theta1(ii)), ...
        'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
    
    line([params.l1*cos(theta1(ii))  params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii))], ...
         [params.l1*sin(theta1(ii))  params.l1*sin(theta1(ii))+params.l2*sin(theta1(ii)+theta2(ii))], ...
         'linewidth', 2, 'Color', 'b');
    
    plot(params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)), ...
         params.l1*sin(theta1(ii))+params.l2*sin(theta1(ii)+theta2(ii)), ...
        'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    
    line([-2 2] ,[0 0], 'LineWidth', 2, 'Color', 'k');
    line([0 0] ,[-2 2], 'LineWidth', 2, 'Color', 'k');
    pause( 0.1 );
    if (ii~= length(theta1))
        clf;
    else
        % Do nothing, Do not clear figure if not final post
    end
    axis(2*[-1 1 -1 1]);
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    set(gca, 'FontSize', 16);

end

end % if(ind_plot)



%% Closed Loop EOM
clc;
close all;

X0 = [0;
      0;
      0;
      0];
   
fun_fb = @(t,X)eom_2DOF_planar_robot_manipulator_feedback(t, X, tspan, params, Xd, Yd, Kinematics_Profiles);
[tout_fb, Xout_fb] = ode45(fun_fb, tspan, X0, []);

theta1_fb     = Xout_fb(:,1);
theta1_dot_fb = Xout_fb(:,2);
theta2_fb     = Xout_fb(:,3);
theta2_dot_fb = Xout_fb(:,4);

X_fb = l1*cos(theta1_fb) + l2*cos(theta1_fb+theta2_fb);
Y_fb = l1*sin(theta1_fb) + l2*sin(theta1_fb+theta2_fb);
%


ind_plot = 1;
if(ind_plot)
for ii = 1:1
    figure;
    set(gcf, 'Position', [0 0 2560 1280]/2);
    
    % X-Y
    subplot(2,3,[1]); 
    plot(Xd, Yd, 'LineWidth', 2, 'Color', 'm'); 
    line([-2 2], [0 0],  'LineWidth', 2, 'Color', 0.5*[1 1 1]);
    line([0 0],  [-2 2], 'LineWidth', 2, 'Color', 0.5*[1 1 1]);
    hold on;
    plot(X_fb, Y_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    xlabel('X [m]'); ylabel('Y [m]'); axis([-2 2 -2 2]); set(gca, 'FontSize', 16); grid on;
    
    % Theta1 - Theta2
    subplot(2,3,[4]); 
    plot(theta1_d*180/pi, theta2_d*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', [1 0.5 0]); 
    hold on;
    plot(theta1_fb*180/pi, theta2_fb*180/pi, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    xlabel('Theta1d [deg]'); ylabel('Theta2d [deg]'); set(gca, 'FontSize', 16); grid on;

    
    % X 
    subplot(2,3,2); 
    plot(tspan, Xd, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'c'); 
    hold on;
    subplot(2,3,2); plot(tspan, X_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k');
    xlabel('Time [s]'); ylabel('X [m]');set(gca, 'FontSize', 16); grid on;
    
    
    % Y
    subplot(2,3,5); 
    plot(tspan, Yd, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g'); 
    hold on;
    plot(tspan, Y_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    xlabel('Time [s]'); ylabel('Y [m]');set(gca, 'FontSize', 16); grid on;
    
    subplot(2,3,3); 
    plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'm'); 
    hold on;
    plot(tspan, theta1_fb*180/pi, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    xlabel('Time [s]'); ylabel('Theta1d [deg]');set(gca, 'FontSize', 16); grid on;
    
    
    subplot(2,3,6); 
    plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'c'); 
    hold on;
    plot(tspan, theta2_fb*180/pi, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    xlabel('Time [s]'); ylabel('Theta2d [deg]');set(gca, 'FontSize', 16); grid on;

    
    
end % for ii = 1:1
end % if(ind_plot)


%%
ind_plot = 1;
if(ind_plot)
    
figure;
set(gcf, 'Position', [0 0 1280 1280]/2);
grid on;
axis(2*[-1 1 -1 1]);
grid on;
for ii = 1:100:length(theta1_fb)

    
    line([0  params.l1*cos(theta1_fb(ii))], [0 params.l1*sin(theta1_fb(ii))], 'linewidth', 2, 'Color', 'g');
    hold all;
    line([x1 x2], [y1 y2], 'LineWidth', 2, 'Color', 'r');
    plot(X_fb, Y_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k');
    plot(0, 0, 'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
    
    plot(params.l1*cos(theta1_fb(ii)), params.l1*sin(theta1_fb(ii)), ...
        'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
    
    line([params.l1*cos(theta1_fb(ii))  params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii))], ...
         [params.l1*sin(theta1_fb(ii))  params.l1*sin(theta1_fb(ii))+params.l2*sin(theta1_fb(ii)+theta2_fb(ii))], ...
         'linewidth', 2, 'Color', 'b');
    
    plot(params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)), ...
         params.l1*sin(theta1_fb(ii))+params.l2*sin(theta1_fb(ii)+theta2_fb(ii)), ...
        'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    
    line([-2 2] ,[0 0], 'LineWidth', 2, 'Color', 0.5*[1 1 1]);
    line([0 0] ,[-2 2], 'LineWidth', 2, 'Color', 0.5*[1 1 1]);
    pause( 0.1 );
    if (ii~= length(theta1_fb))
        clf;
    else
        % Do nothing, Do not clear figure if not final post
    end
    axis(2*[-1 1 -1 1]);
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    set(gca, 'FontSize', 16);

end

end % if(ind_plot)



