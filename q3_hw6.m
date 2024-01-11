%% HW5
clc
clear all
close all

%% Bullet 1
% define geometric parameters 
syms L_1 L_2 theta_1 theta_2 XE YE 

% specify values for the link lengths of the robot
L1 = sym('x');
L2 = sym('y');

% define X and Y coordinates of the end-effector 
XE_RHS = L_1*cos(theta_1) + L_2*cos(theta_1+theta_2)
YE_RHS = L_1*sin(theta_1) + L_2*sin(theta_1+theta_2)

% Find Inverse Kinematics 

% define the foward kinematics equation 
XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;

% solve for theta1 and theta2
S = solve([XE_EQ YE_EQ], [theta_1 theta_2]) 

% show the pair of solutions for theta1 and theta2
simplify(S.theta_1)
simplify(S.theta_2)

% convert the solutions into Matlab functions you can use later 
TH1_MLF{1} = matlabFunction(S.theta_1(1),'Vars',[L_1 L_2 XE YE]);
TH1_MLF{2} = matlabFunction(S.theta_1(2),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{1} = matlabFunction(S.theta_2(1),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{2} = matlabFunction(S.theta_2(2),'Vars',[L_1 L_2 XE YE]);

% find theta 1 and theta 2 for points (1,0) and (0,2) 
disp('Theta 1 for point (1,0) is:'); 
rad2deg(TH1_MLF{1}(1,1,1,0)) % theta 1 for point (1,0)
disp('Theta 2 for point (1,0) is:'); 
rad2deg(TH2_MLF{1}(1,1,1,0)) % theta 2 for point (1,0)
disp('Theta 1 for point (0,2) is:'); 
rad2deg(TH1_MLF{1}(1,1,0,2)) % theta 1 for point (0,2)
disp('Theta 2 for point (0,2) is:'); 
rad2deg(TH2_MLF{1}(1,1,0,2)) % theta 2 for point (0,2)

%% Construct a straight-line trajectory and divide it into N = 50 points

% Define the starting and ending points of the line
Point1 = [1, 0];
Point2 = [0, 2];

% Number of points (N)
N = 50;
L1 = 1;
L2 = 1;

% Generate equally spaced points along the line
xPoints = linspace(Point1(1), Point2(1), N);
yPoints = linspace(Point1(2), Point2(2), N);

% Plot the trajectory
plot(xPoints, yPoints, '-o');
xlabel('X-axis');
ylabel('Y-axis');
title('Straight-line Trajectory with 50 Points');
grid on;

%% Use inverse kinematics for each of the points (there are 50 points)

N = 50

% Iterate through each point in the trajectory - find inverse kinematics 
for i = 1:N
    x = xPoints(i); % Current x point 
    y = yPoints(i); % Current y point 
    
    % Calculate inverse kinematics 
    theta1_1(1,i) = TH1_MLF{1}(L1, L2, x, y);
    theta2_1(1,i) = TH2_MLF{1}(L1, L2, x, y);

    theta1_2(1,i) = TH1_MLF{2}(L1, L2, x, y);
    theta2_2(1,i) = TH2_MLF{2}(L1, L2, x, y);
end

%% Calculate the trajectories 

% Initialize arrays to store the velocities 
theta_1_1_dot = zeros(1, N);
theta_2_1_dot = zeros(1, N); 
theta_1_2_dot = zeros(1, N);
theta_2_2_dot = zeros(1, N);

% Calculate the velocities for each point in the trajectory
for i = 2:N
    dt = 3/50; % time step between each point 
    theta_1_1_dot(i-1) = (theta1_1(i) - theta1_1(i-1)) / dt;
    theta_2_1_dot(i-1) = (theta2_1(i) - theta2_1(i-1)) / dt;
    theta_1_2_dot(i-1) = (theta1_2(i) - theta1_2(i-1)) / dt;
    theta_2_2_dot(i-1) = (theta2_2(i) - theta2_2(i-1)) / dt;
end

for i = 1:N
    end_effector_xVal(i) = theta_1_2_dot(i);
    end_effector_yVal(i) = theta_2_2_dot(i);
end 

%% Plot the trajectory velocity profiles 

% set initial points to 0 for plot
theta_1_1_dot(1) = 0;
theta_2_1_dot(1) = 0;
theta_1_2_dot(1) = 0;
theta_2_2_dot(1) = 0;

% set final points to 0 for plot 
theta_1_1_dot(end) = 0;
theta_2_1_dot(end) = 0;
theta_1_2_dot(end) = 0;
theta_2_2_dot(end) = 0;

% Plot 

figure;
time = linspace(0, 3, 50);

subplot(2, 1, 1);
plot(time, theta_1_1_dot);
hold on
plot(time, theta_2_1_dot);
hold off
grid on
title('Joint Velocity Profile for Solution 1');
xlabel('Time (seconds)');
ylabel('Velocity (degrees)');

subplot(2, 1, 2);
plot(time, theta_1_2_dot);
hold on
plot(time, theta_2_2_dot);
hold off
grid on
title('Joint Velocity Profile for Solution 2');
xlabel('Time (seconds)');
ylabel('Velocity (degrees)');

%% Animate the motion (simulate) - set up a loop 

figure; 
animation_speed = 0.1;

for i=1:N
    clf;
    hold on;
    plot([0, theta_1_1_dot(i), theta_2_1_dot(i)], [0, theta_1_2_dot(i), theta_2_2_dot(i)]);
    plot(end_effector_xVal(1:i), end_effector_yVal(1:i));
    grid on; 
    title('2-link manipulator')
    xlim([-2 2]);
    ylim([-2,2]);
    pause(animation_speed);
    hold off;
end

%% Bullet 2

% Define the starting and ending points of the line
Point1 = [1, 0];
Point2 = [0, 2];

% Number of points (N)
N = 50;
L1 = 1;
L2 = 1;
t = 3;      % dont use tf, it thinks its a transfer function
dt = 3/50;

% Initialize arrays to store the velocities 
theta1 = zeros(1, N);
theta2 = zeros(1, N); 

joint1_xVal = zeros(1, N);
joint1_yVal = zeros(1, N);
joint2_xVal = zeros(1, N);
joint2_yVal = zeros(1, N);

end_effector_xVal = zeros(1, N);
end_effector_yVal = zeros(1, N);

% Iterate through each point in the trajectory - find inverse kinematics 
    
% Calculate inverse kinematics 
for i = 1:N
    x = xPoints(i); % Current x point 
    y = yPoints(i); % Current y point 
    
    % Calculate inverse kinematics 
    theta1_1(1,i) = TH1_MLF{1}(L1, L2, x, y);
    theta2_1(1,i) = TH2_MLF{1}(L1, L2, x, y);

    theta1_2(1,i) = TH1_MLF{2}(L1, L2, x, y);
    theta2_2(1,i) = TH2_MLF{2}(L1, L2, x, y);
end

%% Interpolate the joint trajectory using a 3rd-order polynomial

% constraint equations 
time = linspace(0, 3, N);

a0_theta1 = theta1_1;
a1_theta1 = 0;
a2_theta1 = (3*(theta1_2 - theta1_1)) / (t^2);
a3_theta1 = (-2*(theta1_2 - theta1_1)) / (t^3); 

a0_theta2 = theta2_1;
a1_theta2 = 0;
a2_theta2 = 3*(theta2_2 - theta2_1) / (t^2);
a3_theta2 = -2*(theta2_2 - theta2_1) / (t^3);  

% using a 3rd-order polynomial

% cubic polynomial for thetas
theta1 = a0_theta1 + a1_theta1*t + a2_theta1*t^2 + a3_theta1*t^3;
theta2 = a0_theta2 + a1_theta2*t + a2_theta2*t^2 + a3_theta2*t^3;

%% Calculate the positions - shown in animation 

% calc the joint positions for each joint angle
for i = 1:N
    joint1_xVal(i) = L1 * cos(theta1(i));
    joint1_yVal(i) = L1 * sin(theta1(i));
    joint2_xVal(i) = L1 * cos(theta1(i)) + L2 * cos(theta1(i) + theta2(i));
    joint2_yVal(i) = L1 * sin(theta1(i)) + L2 * sin(theta1(i) + theta2(i));
end 

% calc the end-effector positions for each joint angle
for i = 1:N
    end_effector_xVal(i) = joint2_xVal(i);
    end_effector_yVal(i) = joint2_yVal(i);
end 

%% Calculate the velocities 

theta1_velocity = [0, diff(theta1) / dt];
theta2_velocity = [0, diff(theta2) / dt];
end_effector_xVelocity = [0, diff(end_effector_xVal) / dt];
end_effector_yVelocity = [0, diff(end_effector_yVal) / dt];

%% Plot the trajectory velocity profiles 

% set initial points to 0 for plot
theta1_velocity(1) = 0;
theta2_velocity(1) = 0;
end_effector_xVelocity(1) = 0;
end_effector_yVelocity(1) = 0;

% set final points to 0 for plot 
theta1_velocity(end) = 0;
theta2_velocity(end) = 0;
end_effector_xVelocity(end) = 0;
end_effector_yVelocity(end) = 0;

% plot joint velocities 

figure;
time = linspace(0, 3, 50);

subplot(2, 1, 1);
plot(time, theta1_velocity);
hold on
plot(time, theta2_velocity);
grid on
title('Joint Velocity Profile for Theta 1 and 2');
xlabel('Time (seconds)');
ylabel('Velocity (degrees)');
legend('theta1_velocity' , 'theta2_velocity');

% plot end-effector velocities 

subplot(2, 1, 2);
plot(time, end_effector_xVelocity);
hold on
plot(time, end_effector_yVelocity);
grid on
title('Joint Velocity Profile for End-Effector 1 and 2');
xlabel('Time (seconds)');
ylabel('Velocity (degrees)');
legend('end_effector_xVelocity' , 'end_effector_yVelocity');

%% Animate the motion (simulate) - set up a loop 

figure; 
animation_speed = 0.1;

for i=1:N
    clf;
    hold on;
    plot([0, joint1_xVal(i), joint2_xVal(i)], [0, joint1_yVal(i), joint2_yVal(i)]);
    plot(end_effector_xVal(1:i), end_effector_yVal(1:i));
    grid on; 
    title('2-link manipulator')
    xlim([-2 2]);
    ylim([-2,2]);
    pause(animation_speed);
    hold off;
end
