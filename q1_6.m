% define geometric parameters
syms a_1 a_2 theta_1 theta_2 

% substitute values
a_1 = 1;
a_2 = 1;
theta_1 = pi/4 - sqrt(2)/2;
theta_2 = -pi/2 + sqrt(2);

% forward kinematics
x1 = a_1 * cos(theta_1);
y1 = a_1 * sin(theta_1);
x2 = x1 + cos(theta_1 + theta_2);
y2 = y1 + sin(theta_1 + theta_2);

% plot the 2 link manipulator 
figure;
plot([0, x1], [0,y1], 'b-', 'LineWidth', 2); % link 1
hold on;
plot([x1, x2], [y1, y2], 'ro-', 'LineWidth', 2); % link 2

plot(x1, y1, 'ko', 'MarkerSize', 9); % joint 1
plot(x2, y2, 'ko', 'MarkerSize', 9); % joint 2

axis equal;
xlim([-2,2]);
ylim([-2,2]);

% organize 
title('Planar 2-link Manipulator with 2 Revolute Joints Configuration');
xlabel('X-axis');
ylabel('Y-axis');
grid on; 