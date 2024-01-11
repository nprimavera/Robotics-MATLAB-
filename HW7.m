%% Forward and Inverse Kinematics of a 2R Planar Manipulator

% Define the Geometric Parameters 
syms L_1 L_2 theta_1 theta_2 XE YE 

% Specify Values for the Link Lengths of the Robot
L_1 = sym('x');
L_2 = sym('y');

% Define X and Y coordinates of the End-Effector 
disp('The Forward Kinematics are:')
XE_RHS = L_1*cos(theta_1) + L_2*cos(theta_1+theta_2)
YE_RHS = L_1*sin(theta_1) + L_2*sin(theta_1+theta_2)

% Find Inverse Kinematics 

% Define the Foward Kinematics Equation 
XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;

% Solve for theta1 and theta2
S = solve([XE_EQ YE_EQ], [theta_1 theta_2]);

% Show the pair of solutions for theta1 and theta2
disp('The Inverse Kinematics are:')
simplify(S.theta_1)
simplify(S.theta_2)

% Convert the solutions into MATLAB functions you can use later 
TH1_MLF{1} = matlabFunction(S.theta_1(1),'Vars',[L_1 L_2 XE YE]);
TH1_MLF{2} = matlabFunction(S.theta_1(2),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{1} = matlabFunction(S.theta_2(1),'Vars',[L_1 L_2 XE YE]);
TH2_MLF{2} = matlabFunction(S.theta_2(2),'Vars',[L_1 L_2 XE YE]);

%% Simulate the motion of this 2R Planar Manipulator 

% Define the Geometric Parameters 
syms m_1 m_2 lc1 lc2 I_1 I_2 T_1 T_2 t h k V V_1 V_2 Torque1 Torque2 states  % everything else 
syms theta theta_dot theta_1_dot theta_2_dot theta_1_dot_dot theta_2_dot_dot % all the theta symbolic variables 
syms Jv_c1_T Jv_c2_T Vc1_T Vc2_T theta_dot_T   % all the transpose variables 

% Parameters - remain constant throughout
L_1 = 1;            % link 1 length (m)
L_2 = 1;            % link 2 length (m)
lc1 = L_1/2;        % distance to center of mass from joint 1 (m)
lc2 = L_2/2;        % distance to center of mass from joint 2 (m)
m_1 = 2;            % mass of link 1 (kg)
m_2 = 2;            % mass of link 2 (kg)
g = 9.81;           % gravitational constant (m/s^2)
I_1 = m_1*L_1^2;    % moment of inertia for link 1 (kg*m^2)
I_2 = m_2*L_2^2;    % moment of inertia for link 2 (kg*m^2)
    
% Initial Conditions of this 2R Planar Manipulator System
theta = [theta_1; theta_2];     % put theta 1 & 2 into a column matrix 
theta_1 = pi/6;                 % initial value of theta 1
theta_2 = pi/4;                 % initial value of theta 2
time = [0, 10];                 % simulate the mechanism for 10 sec
w_1_initial = 0;                % initial angular velocities are 0
w_2_initial = 0;

% Input Torque for each motor - given in the question 
Torque1 = 0.5*sin(time)
Torque2 = sin(0.5*time + pi/3)

% Use Jacobian expressions to help compute the kinetic energy              
Jv_c1 = [-lc1*sin(theta_1), 0; lc1*cos(theta_1), 0; 0, 0] % Jacobian value 1
Jv_c2 = [-L_1*sin(theta_1)-lc2*sin(theta_1 + theta_2), -lc2*sin(theta_1 + theta_2); L_1*cos(theta_1)+lc2*cos(theta_1+theta_2), lc2*cos(theta_1+theta_2); 0, 0] % Jacobain value 2

% Define theta_dot 
theta_dot = [theta_1_dot; theta_2_dot]   % put theta 1 & 2 into a column matrix 

% Define initial conditions in a column matrix - will need for ODE solver
initial_conditions = [theta_1; theta_2; theta_1_dot; theta_2_dot] 

% Compute the kinetic energy 
Vc1 = Jv_c1*theta_dot         % joint 1 KE
Vc2 = Jv_c2*theta_dot         % joint 2 KE 

% Transpose the matrices 
% how to transpose a matrix: transposed_matrix = original_matrix';
Jv_c1_T = Jv_c1'
Jv_c2_T = Jv_c2'
Vc1_T = Vc1'
Vc2_T = Vc2'
theta_dot_T = theta_dot'
        
% Translational part of kinetic energy
% 1/2*m_1*Vc1_T*Vc1 + 1/2*m_2*Vc2_T*Vc2 == 1/2*theta_dot(m_1*Jv_c1_T*Jv_c1 + m_2*Jv_c2_T*Jv_c2);
        
% Angular Velocities
w_1 = theta_1_dot*k                    % joint 1 angular velocity
w_2 = (theta_1_dot + theta_2_dot)*k    % joint 2 angular velocity 
k = [0; 0; 1] 
        
% Rotational Kinetic Energy of the Overall System
rotational_KE = 1/2*theta_dot_T*(I_1*[1, 0; 0, 0] + I_2*[1, 1; 1, 1])*theta_dot
        
% Inertia Matrix
% D(theta) = (m_1*Jv_c1_T*Jv_c1 + m_2*Jv_c2_T*Jv_c2 + [I_1+I_2, I_2; I_2, I_2]);
d_11 = m_1*lc1^2 + m_2*(L_1^2 + lc2^2 + 2*L_1*lc2*cos(theta_2)) + I_1 + I_2
d_12 = m_2*(lc2^2 + L_1*lc2*cos(theta_2) + I_2)
d_21 = m_2*(lc2^2 + L_1*lc2*cos(theta_2) + I_2)
d_22 = m_2*lc2^2 + I_2
        
% Compute the Christoffel symbols - formula in textbook - i just used the direct answers for simplicity and time
h = -m_2*L_1*lc2*sin(theta_2);
c_111 = 0;
c_121 = h; %  h = -m_2*L_1*lc2*sin(theta_2) - this expression simplifies to h
c_211 = h; %  h = -m_2*L_1*lc2*sin(theta_2)
c_221 = h;
c_112 = -h;
c_122 = 0;
c_212 = 0;
c_222 = 0;
        
% Potential Energy - PE = V 
V_1 = m_1*g*lc1*sin(theta_1)
V_2 = m_2*g*(L_1*sin(theta_1) + lc2*sin(theta_1 + theta_2))
V = V_1 + V_2 % == (m_1*lc1 + m_2*L_1)*g*sin(theta_1) + m_2*lc2*g*sin(theta_1 + theta_2);

theta_1_dot = V_1
theta_2_dot = V_2
        
% Phi is the derivative of PE / derivative of theta_1 or theta_2
phi_1 = (m_1*lc1 + m_2*L_1)*g*cos(theta_1) + m_2*lc2*g*cos(theta_1 + theta_2)
phi_2 = m_2*lc2*cos(theta_1 + theta_2)

theta_1_dot_dot = phi_1
theta_2_dot_dot = phi_2
        
% Dynamic Equations of the system
T_1 = d_11*theta_1_dot_dot + d_12*theta_2_dot_dot + c_121*theta_1_dot*theta_2_dot + c_211*theta_2_dot*theta_1_dot +c_221*theta_2^2 + phi_1
T_2 = d_21*theta_1_dot_dot + d_22*theta_2_dot_dot +c_112*theta_1_dot^2 + phi_2
C = [h*theta_2_dot, h*theta_2_dot+h*theta_1_dot; -h*theta_1_dot, 0]

% Simulate the system

function dxdt = manipulatorODE(t, x)
 
    % Initial variables 
    theta_1 = x(1);
    theta_2 = x(2); 
    theta_1_dot = x(3); 
    theta_2_dot = x(4);

    % Parameters 
    L_1 = 1;            % link 1 length (m)
    L_2 = 1;            % link 2 length (m)
    lc1 = L_1/2;        % distance to center of mass from joint 1 (m)
    lc2 = L_2/2;        % distance to center of mass from joint 2 (m)
    m_1 = 2;            % mass of link 1 (kg)
    m_2 = 2;            % mass of link 2 (kg)
    g = 9.81;           % gravitational constant (m/s^2)
    I_1 = m_1*L_1^2;    % moment of inertia for link 1 (kg*m^2)
    I_2 = m_2*L_2^2;    % moment of inertia for link 2 (kg*m^2)
    w_1_initial = 0;    % angular velocity of joint 1 is zero initially 
    w_2_initial = 0;    % angular velocity of joint 2 is zero initially

    % Dynamic Equations 
    T_1 = d_11*theta_1_dot_dot + d_12*theta_2_dot_dot + c_121*theta_1_dot*theta_2_dot + c_211*theta_2_dot*theta_1_dot +c_221*theta_2^2 + phi_1;
    T_2 = d_21*theta_1_dot_dot + d_22*theta_2_dot_dot +c_112*theta_1_dot^2 + phi_2;

    % Solve for accelerations 
    theta_1_dot_dot = (m_1*lc1 + m_2*L_1)*g*cos(theta_1) + m_2*lc2*g*cos(theta_1 + theta_2);
    theta_2_dot_dot = m_2*lc2*cos(theta_1 + theta_2);

    % Build the state derivative vector
    dxdt = [theta_1_dot; theta_2_dot; theta_1_dot_dot; theta_2_dot_dot]

    % Use the ODE 45 solver to Simulate 

    % Initial Conditions 
    initial_values = [theta_1; theta_2; w_1_initial; w_2_initial]; 
    time = [0, 10];
    
    % Utilize the ODE 45 solver 
    [t, result] = ode45(@manipulatorODE, time, initial_values)
    
    % Store the results 
    theta_1_simulated = result(:, 1)
    theta_2_simulated = result(:, 2)
    w_1_simulated = result(:, 3)
    w_2_simulated = result(:, 4)

    % Plot results 
    figure
    plot(t, theta_1_simulated, 'r', t, theta_2_simulated, 'b');
    xlabel('Time (s)');
    ylabel('Joint Angles');
    legend('Theta 1', 'Theta 2');

end
 
%% Independent Joint PD Controller - Goal is to control the motion of the system from an initial pose using torque controllers

% Joint PD Controller = proportional-derivative (PD) controller w/ two independent joints

function simulate_using_independent_joint_PD_controllers()

    % Parameters - remain constant throughout
    L_1 = 1;            % link 1 length (m)
    L_2 = 1;            % link 2 length (m)
    lc1 = L_1/2;        % distance to center of mass from joint 1 (m)
    lc2 = L_2/2;        % distance to center of mass from joint 2 (m)
    m_1 = 2;            % mass of link 1 (kg)
    m_2 = 2;            % mass of link 2 (kg)
    g = 9.81;           % gravitational constant (m/s^2)
    time = [0 10];      % time of simulation 
    I_1 = m_1*L_1^2;    % moment of inertia for link 1 (kg*m^2)
    I_2 = m_2*L_2^2;    % moment of inertia for link 2 (kg*m^2)
    
    % Initial Parameters at t = 0
    theta_1_initial = pi/3;         % initial joint 1 angle (rad)
    theta_2_initial = pi/4;         % initial joint 2 angle (rad)
    theta_1_dot_initial = 0;        % initial joint 1 velocity (rad/s)
    theta_2_dot_initial = 0;        % initial joint 2 velocity (rad/s) 
    
    % Final Parameters at t 
    theta_1_final = pi/2;           % final joint 1 angle (rad)
    theta_2_final= pi/3;            % final joint 2 angle (rad)
    theta_1_dot_final = 0;          % final joint 1 velocity (rad/s)
    theta_2_dot_final = 0;          % final joint 2 velocity (rad/s) 
    
    % Generalize/Organize the thetas into matrices - do not need
    theta_initial = [theta_1_initial; theta_2_initial];
    theta_dot_initial = [theta_1_dot_initial; theeta_2_dot_initial];
    theta_final = [theta_1_final; theta_2_final];
    theta_dot_final = [theta_1_dot_final; theta_2_dot_final]; 
 
    % Define initial conditions in a column matrix 
    initial_states = [theta_1_initial; theta_2_initial; theta_1_dot_initial; theta_2_dot_initial]; 

    % Denote the Controller Gains for joint 1 and joint 2
    syms Kp1 Kd1 Kp2 Kd2 
    
    % Choose the value of the Gains - determine how much the controller responds to the error (difference b/w desired and actual positions) and its rate of change 
    Kp1 = 100;  % proportional gain for joint 1
    Kd1 = 25;   % derivative gain for joint 1 
    Kp2 = 100;  % proportional gain for joint 2
    Kd2 = 25;   % derivative gain for joint 2 

    % Simulate using ODE 45 solver 
    [t_initial, states_initial] = ode45(@(t, y) dynamics(t, y, L_1, L_2, m_1, m_2, lc1, lc2, I_1, I_2, g, theta_1_final, theta_2_final, Kp1, Kd1, Kp2, Kd2), time, initial_states)
    
    % Plot - initial gains 
    figure 

    subplot (2, 1, 1);
    plot(t_initial, rad2deg(states_initial(:, 1)), 'r-', t_initial, states_initial(:, 2), 'b-');    % Joint Angles 
    xlabel('Time (s)');
    ylabel('Joint Angles (degrees)');
    legend('Joint 1', 'Joint 2');
    title('Independent Joint PD Controller Simulation - Joint Angles');
    
    subplot (2, 1, 2);
    plot(t_initial, rad2deg(states_initial(:, 3)), 'r--', t_initial, states_initial(:, 4), 'b--');  % Joint Velocities 
    xlabel('Time (s)');
    ylabel('Joint Velocities (deg/s)');
    legend('Joint 1', 'Joint 2');
    title('Independent Joint PD Controller Simulation - Joint Velocities');
    
    % Effect of Controller Gains - plot the response for a variety of controller gains and compare results 
    
    % Additional values of controller gains 
    gains = [
        struct('Kp1', 150, 'Kd1', 50, 'Kp2', 150, 'Kd2', 50), % higher gains
        struct('Kp1', 50, 'Kd1', 10, 'Kp2', 50, 'Kd2', 10), % lower gains 
    ]

    % Store the simulation results for the different gains in a matrix 
    results = [];

    % Store the original simulation results 
    results(1).Time = t_initial;                    % t_initial is the time 
    results(1).Output = rad2deg(states_initial);    % states_initial is the output 

    % Run simulations with different gains 
    for i = 1:length(gains)
        [Time, Output] = ode45(@(t, y) dynamics(t, y, L_1, L_2, m_1, m_2, lc1, lc2, I_1, I_2, g, theta_1_final, theta_2_final, gains(i).Kp1, gains(i).Kd1, gains(i).Kp2, gains(i).Kd2), time, initial_states);
        results(i+1).Time = Time;
        results(i+1).Output = Output; 
    end

    % Plot - comparison of the different gains 
    figure
    hold on;
    
    colors = ['r', 'g', 'b'];
    legend = {'Original Gains', 'Lower Gains', 'Higher Gains'};

    for i = 1:length(results)
        plot(results(i).Time, results(i).Output(:, 1), 'Color', colors(i), 'DisplayName', [legends{i} ' - Joint 1']);
        plot(results(i).Time, results(i).Output(:, 2), 'Color', colors(i), 'LineStyle', '--', 'DisplayName', [legends{i} ' - Joint 2']);
    end
    xlabel('Time (s)');
    ylabel('Joint Angles (deg)');
    legend('Joint 1', 'Joint 2');
    title('Comparison of Joint Angles with Different PD Gains');
    legend ('show');
    hold off;

    % Animate the simulation
    [t, states] = ode45(@(t, y) dynamics(t, y, L_1, L_2, m_1, m_2, lc1, lc2, I_1, I_2, g, theta_1_final, theta_2_final, Kp1, Kd1, Kp2, Kd2), time, initial_states)
    
    figure
    axis equal;
    grid on;
    xlim([-2*(L_1+L_2) 2*(L_1+L_2)]);
    ylim([-2*(L_1+L_2) 2*(L_1+L_2)]);
    xlabel('X-position (m)');
    ylabel('Y-position (m)');
    title('2R Planar Manipulator Simulation');
    hold on;

    for i = 1:length(t)
        joint1_position = [L_1*cos(rad2deg(states(i, 1))), L_1*sin(rad2deg(states(i, 1)))];
        joint2_position = joint1_position + [L_2*cos(rad2deg(states(i, 1)) + rad2deg(states(i, 2))), L_2*sin(rad2deg(states(i, 1)) + rad2deg(states(i, 2)))];

        if exist ('h', 'var'), delete(h);   % update the link positions and remove the previous plot 
        
        end

        h(1) = plot([0, joint1_position(1)], [0, joint1_position(2)], 'r', 'LineWidth', 2);
        h(2) = plot([joint1_position(1), joint2_position(1)], [joint1_position(2), joint2_position(2)], 'b', 'LineWidth', 2);
        h(3) = plot(joint1_position(1), joint1_position(2), 'ko', 'MarkerFaceColor', 'k');
        h(4) = plot(joint2_position(1), joint2_position(2), 'ko', 'MarkerFaceColor', 'k');
        
        draw now;
        pause(0.05)

    end

    hold off;

end

% Dynamic Functions
function dydt = dynamics(t, y, L_1, L_2, m_1, m_2, lc1, lc2, I_1, I_2, g, theta_1_final, theta_2_final, Kp1, Kd1, Kp2, Kd2);

    % Initial variables 
    theta_1 = y(1);
    theta_2 = y(2); 
    theta_1_dot = y(3); 
    theta_2_dot = y(4); 

    % PD control torques 
    torque1 = Kp1*(theta_1_final - theta_1) + Kd1*(-theta_1_dot)
    torque2 = Kp2*(theta_2_final - theta_2) + Kd2*(-theta_2_dot)

    % Inertia Equations 
    d_11 = m_1*lc1^2 + m_2*(L_1^2 + lc2^2 + 2*L_1*lc2*cos(theta_2)) + I_1 + I_2;
    d_12 = d_21 == m_2*(lc2^2 + L_1*lc2*cos(theta_2) + I_2);
    d_22 = m_2*lc2^2 + I_2;
    D = [m_1*lc1^2 + m_2*(L_1^2 + lc2^2 + 2*L_1*lc2*cos(theta_2)) + I_1 + I_2, m_2*(lc2^2 + L_1*lc2*cos(theta_2) + I_2); m_2*(lc2^2 + L_1*lc2*cos(theta_2) + I_2), m_2*lc2^2 + I_2];
    
    % Coriolis and Centrifugal matrix - C(theta, theta_dot)
    h = -m_2*L_1*lc2*sin(theta_2);
    C = [h*theta_2_dot, h*theta_2_dot+h*theta_1_dot; -h*theta_1_dot, 0];

    % Gravity matrix - G(theta)
    G = [(m_1*lc1 + m_2*L_1)*g*cos(theta_1) + m_2*lc2*g*cos(theta_1+theta_2); m_2*lc2*g*cos(theta_1 + theta_2)];

    % Joint Accelerations
    theta_dot_dot = D / ([torque1; torque2] - C*[theta_1_dot; theta_2_dot] - G);

    % Find derivatives of intial variables 
    dydt = [theta_1_dot; theta_2_dot; theta_dot_dot];

end

% Effect of Controller Gains 
% Kp = proportional gain - affects response speed --> higher values make systems go faster, but they are more likely to destabilize 
% Kd = derivative gain - affects stability --> higher values provide damping to limit destabilization, but they run the risk of getting overdamped 

% Trade off b/w speed of response and stability
% Lower gains yield a slow and stable response 
% Original gains yield a normal speed and a balanced response - best combination 
% Higher gains yield a fast and unstable response 


%% Computed Torque Controller 

function simulate_using_computed_torque_controllers()

    % Can copy alot from previous example 

    % Parameters - remain constant throughout
    L_1 = 1;            % link 1 length (m)
    L_2 = 1;            % link 2 length (m)
    lc1 = L_1/2;        % distance to center of mass from joint 1 (m)
    lc2 = L_2/2;        % distance to center of mass from joint 2 (m)
    m_1 = 2;            % mass of link 1 (kg)
    m_2 = 2;            % mass of link 2 (kg)
    g = 9.81;           % gravitational constant (m/s^2)
    time = [0 10];      % time of simulation 
    I_1 = m_1*L_1^2;    % moment of inertia for link 1 (kg*m^2)
    I_2 = m_2*L_2^2;    % moment of inertia for link 2 (kg*m^2)
   
    % Initial Parameters at t = 0
    theta_1_initial = pi/3;         % initial joint 1 angle (rad)
    theta_2_initial = pi/4;         % initial joint 2 angle (rad)
    theta_1_dot_initial = 0;        % initial joint 1 velocity (rad/s)
    theta_2_dot_initial = 0;        % initial joint 2 velocity (rad/s) 
    initial_conditions = [theta_1_initial; theta_2_initial; theta_1_dot_initial; theta_2_dot_initial]; 
    
    % Final Parameters at t 
    theta_1_final = pi/2;           % final joint 1 angle (rad)
    theta_2_final= pi/3;            % final joint 2 angle (rad)
    theta_1_dot_final = 0;          % final joint 1 velocity (rad/s)
    theta_2_dot_final = 0;          % final joint 2 velocity (rad/s) 
    
    % Simulate 
    [t, y] = ode45(@(t, y) manipulator_dynamics(t, y, theta_1_final, theta_2_final, m_1, m_2, L_1, L_2, lc1, lc2, I_1, I_2, g), time, initial_conditions);
    
    % Plot
    plot(t, y);

    % Animate the motion 
    animate_manipulator(t, y, L_1, L_2);

end

function dydt = manipulator_dynamics(t, y, theta_1_final, theta_2_final, m_1, m_2, L_1, L_2, lc1, lc2, I_1, I_2, g);

    % Initial variables 
    theta_1 = y(1);
    theta_2 = y(2); 
    theta_1_dot = y(3);
    theta_2_dot = y(4); 

    % Gains - used values from original gains above 
    Kp = [100, 0; 0, 100];  % Proportional Gain 
    Kd = [25, 0; 0, 25];    % Derivative Gain 

    % Calculate the error - b/w actual and desired positions 
    error = [theta_1_final - theta_1; theta_2_final - theta_2];
    error_dot = [0 - theta_1_dot; 0 - theta_2_dot];     % desired velocities are 0 since we start and stop at rest 
    % theta_1_dot_dot = theta_2_dot_dot = 0

    % Calculate acceleration 
    theta_final_dot_dot = Kp*error + Kd*eror_dot;

    % Compute the Mass, Corriolis, and Gravity Matrices 
    M = [m_1*lc1^2 + I_1 + m_2*(L_1^2 + lc2^2 + 2*L_1*lc2*cos(theta_2)), m_2*(lc2^2 + L_1*lc2*cos(theta_2)); m_2*(lc2^2 + L_1*lc2*cos(theta_2)), m_2*lc2^2 + I_2];
    V = [-m_2*L_1*lc2*theta_2_dot^2*sin(theta_2); m_2*lc2*g*cos(theta_1 + theta_2)];
    G = [(m_1*lc1 + m_2*L_1)*g*cos(theta_1) + m_2*lc2*g*cos(theta_1 + theta_2); m_2*lc2*g*cos(theta_1 + theta_2)];

    % Calculate the torque
    torque = M*theta_dot_dot + V + G;

    % Calculate the acceleration 
    theta_dot_dot = M / (torque - V - G);

    % Calculate the derivaties 
    dydt = [theta_1_dot; theta_2_dot; theta_dot_dot]

end

function animate(t, y, L_1, L_2)
    figure;
    axis equal;
    grid on;
    xlim([-2*(L_1+L_2) 2*(L_1+L_2)]);
    ylim([-2*(L_1+L_2) 2*(L_1+L_2)]);
    xlabel('X-position (m)');
    ylabel('Y-position (m)');
    title('2R Planar Manipulator Simulation');
    hold on;

    for i = 1:length(t)
        joint1_position = [L_1*cos(rad2deg(y(i, 1))), L_1*sin(rad2deg(y(i, 1)))];
        joint2_position = joint1_position + [L_2*cos(rad2deg(y(i, 1)) + rad2deg(y(i, 2))), L_2*sin(rad2deg(y(i, 1)) + rad2deg(y(i, 2)))];

        if exist ('h', 'var'), delete(h);   % update the link positions and remove the previous plot 
        end

        h(1) = plot([0, joint1_position(1)], [0, joint1_position(2)], 'r', 'LineWidth', 2);
        h(2) = plot([joint1_position(1), joint2_position(1)], [joint1_position(2), joint2_position(2)], 'b', 'LineWidth', 2);
        h(3) = plot(joint1_position(1), joint1_position(2), 'ko', 'MarkerFaceColor', 'k');
        h(4) = plot(joint2_position(1), joint2_position(2), 'ko', 'MarkerFaceColor', 'k');

        drawnow;
        pause(0.05);
    end

    hold off;
end
